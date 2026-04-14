#include <stdlib.h>
#include <math.h>
#include "velocity_ctrl.h"
#include "SCurveTrajectory.h"

/*
   Velocity Control Module

   Provides jerk-limited velocity tracking that outputs a position setpoint.

   Strategy:
   The demanded velocity (from CAN / joystick) is the target.  Each ISR tick
   we decide what jerk to apply so the commanded velocity tracks the demand
   smoothly, subject to j_max and a_max constraints.

   The core decision each tick is a three-way choice:

     1. Do I need to start braking NOW to avoid overshooting the demanded velocity?
        → Compute the velocity I'd reach if I started decelerating from this instant.
        → If that "braking velocity" would overshoot the demand, apply braking jerk.

     2. Am I below the demand and have room to accelerate?
        → Apply jerk in the direction of the demand.

     3. Am I at the demand with low acceleration?
        → Coast (hold jerk at zero, bleed off residual acceleration).

   This produces S-shaped velocity ramps (smooth acceleration onset and offset)
   with no switching-time computation — it's fully reactive each tick.

   Position limits:
   Optional software position limits can be configured.  When the accumulated
   position approaches a limit while moving toward it, the filter computes the
   minimum stopping distance and begins decelerating automatically, regardless
   of what the CAN demand says.  This prevents the arm from hitting hard stops.
*/



/*  Helpers  */


/*
   computeStoppingDistance
   Returns the distance needed to decelerate from current velocity v
   to velocity 0, respecting a_max and j_max constraints.
   If the motor has nonzero acceleration in the direction of travel,
   accounts for the velocity gained while ramping acceleration down
   to zero before starting the actual braking profile.
*/
static float_t computeStoppingDistance(float_t v, float_t a, float_t a_max, float_t j_max){

    float_t av = fabsf(v);
    if (av < 1e-6f && fabsf(a) < 1e-6f) return 0.0f;

    /* First: if acceleration is in the same direction as velocity,
       we must ramp it to zero before we can start braking.
       Compute the velocity gained during that ramp-down. */
    float_t v_effective = av;

    /* Check if acceleration is adding to velocity magnitude */
    bool accel_adds = (v > 0.0f && a > 0.0f) || (v < 0.0f && a < 0.0f);
    float_t d_pre = 0.0f;

    if (accel_adds){
        float_t aa = fabsf(a);
        float_t t_to_zero_a = aa / j_max;
        /* Velocity gained while ramping a to zero */
        float_t v_gained = aa * t_to_zero_a - 0.5f * j_max * t_to_zero_a * t_to_zero_a;
        if (v_gained < 0.0f) v_gained = 0.0f;
        /* Distance during this pre-braking phase */
        d_pre = av * t_to_zero_a + 0.5f * aa * t_to_zero_a * t_to_zero_a
              - (1.0f / 6.0f) * j_max * t_to_zero_a * t_to_zero_a * t_to_zero_a;
        v_effective = av + v_gained;
    }

    /* Now compute distance to stop from v_effective with a starting at 0 */
    float_t v_thresh = (a_max * a_max) / j_max;
    float_t d_stop;
    if (v_effective >= v_thresh){
        d_stop = (v_effective / 2.0f) * (v_effective / a_max + a_max / j_max);
    } else {
        d_stop = v_effective * sqrtf(v_effective / j_max);
    }

    return d_pre + d_stop;
}

/*
   computeBrakingVelocity
   If we start applying max braking jerk right now, what velocity will
   we reach by the time acceleration reaches zero?
   This tells us whether we need to start braking to avoid overshooting
   the demanded velocity.
*/
static float_t computeBrakingVelocity(float_t v, float_t a, float_t j_max){

    /* Time to bring acceleration to zero: t = |a| / j_max */
    float_t t = fabsf(a) / j_max;

    /* Velocity change during that time: Δv = a*t + 0.5*j_brake*t^2
       where j_brake opposes a */
    float_t j_brake = (a > 0.0f) ? -j_max : j_max;
    float_t v_at_zero_accel = v + a * t + 0.5f * j_brake * t * t;

    return v_at_zero_accel;
}



/*  Public API                                                         */

// // This is for testing
//VelCtrlHandle* velCtrlInit(float_t currentPos){
//
//    VelCtrlHandle *h = malloc(sizeof(VelCtrlHandle));
//    if (!h) return NULL;
//
//    h->a_max = A_MAX;
//    h->v_max = MAX_VEL;   /* Use the absolute ceiling; can be overridden */
//    h->j_max = J_MAX;
//    h->Ts    = SAMPLING_TIME;
//
//    h->cmd_vel   = 0.0f;
//    h->cmd_accel = 0.0f;
//    h->cmd_pos   = currentPos;
//
//    h->demanded_vel = 0.0f;
//
//    h->pos_min       = 0.0f;
//    h->pos_max       = 0.0f;
//    h->pos_limits_en = false;
//
//    h->is_active = false;
//
//    return h;
//}

void velCtrlInitStatic(VelCtrlHandle *h, float_t currentPos){

    h->a_max = A_MAX;
    h->v_max = MAX_VEL;
    h->j_max = J_MAX;
    h->Ts    = SAMPLING_TIME_PLANNER;

    h->cmd_vel   = 0.0f;
    h->cmd_accel = 0.0f;
    h->cmd_pos   = currentPos;

    h->demanded_vel = 0.0f;

    h->pos_min       = 0.0f;
    h->pos_max       = 0.0f;
    h->pos_limits_en = false;

    h->is_active = false;
}

void velCtrlSetDemand(VelCtrlHandle *h, float_t demandedVel){

    /* Clamp to velocity limits */
    if (demandedVel >  h->v_max) demandedVel =  h->v_max;
    if (demandedVel < -h->v_max) demandedVel = -h->v_max;

    h->demanded_vel = demandedVel;
}


void velCtrlSetPositionLimits(VelCtrlHandle *h, float_t minPos, float_t maxPos){

    h->pos_min = minPos;
    h->pos_max = maxPos;
    h->pos_limits_en = true;
}


void velCtrlClearPositionLimits(VelCtrlHandle *h){

    h->pos_limits_en = false;
}


void velCtrlStart(VelCtrlHandle *h, VelocityFilter *tracker){

    /* Inherit current motor state for seamless transition */
    h->cmd_vel   = tracker->omega;
    h->cmd_accel = tracker->accel;
    h->cmd_pos   = tracker->theta;
    h->demanded_vel = 0.0f;
    h->is_active = true;
}


void velCtrlStop(VelCtrlHandle *h){

    /* Set demand to zero — the ISR will smoothly decelerate.
       Caller should keep ticking until cmd_vel reaches zero. */
    h->demanded_vel = 0.0f;

    /* Don't clear is_active here: the caller checks velCtrlGetVelocity()
       and deactivates once the motor has actually stopped. */
}


float_t velCtrl_ISRStep(VelCtrlHandle *h, VelocityFilter *tracker){

    if (!h->is_active) return h->cmd_pos;

    float_t dt    = h->Ts;
    float_t j_max = h->j_max;
    float_t a_max = h->a_max;

    float_t v = h->cmd_vel;
    float_t a = h->cmd_accel;
    float_t demand = h->demanded_vel;


    /*  Position limit override                                      */
    /*  If approaching a limit, override demand to zero and force    */
    /*  deceleration before hitting the wall.                        */

    if (h->pos_limits_en){
        float_t pos = h->cmd_pos;

        /* Only check limits if the motor has meaningful velocity.
           Tiny velocities (below one jerk step) don't need limit override
           because the braking distance is negligible. */
        float_t vel_deadzone = j_max * dt * dt;

        if (v > vel_deadzone){
            float_t d_stop = computeStoppingDistance(v, a, a_max, j_max);
            /* Approaching upper limit while moving positive */
            if ((pos + d_stop) >= h->pos_max){
                demand = 0.0f;
            }
        }
        if (v < -vel_deadzone){
            float_t d_stop = computeStoppingDistance(v, a, a_max, j_max);
            /* Approaching lower limit while moving negative */
            if ((pos - d_stop) <= h->pos_min){
                demand = 0.0f;
            }
        }
        /* Clamp demand direction if already at or past a limit */
        if (pos >= h->pos_max && demand > 0.0f) demand = 0.0f;
        if (pos <= h->pos_min && demand < 0.0f) demand = 0.0f;
    }


    /*  Jerk decision                                                */
    /*                                                               */
    /*  Velocity error: how far the current velocity is from demand. */
    /*  Braking velocity: the velocity we'd reach if we start        */
    /*  decelerating now (bringing accel to zero).                   */
    /*                                                               */
    /*  The logic:                                                   */
    /*   - If braking velocity would overshoot demand → brake        */
    /*   - Else if we're below demand → accelerate toward it         */
    /*   - Else → coast (bleed off residual accel)                   */


    float_t v_error = demand - v;
    float_t j = 0.0f;

    /* Dead zone threshold for velocity.  When velocity is this close to
       demand, stop using the bang-bang braking/acceleration logic and
       switch to proportional bleed-off of residual acceleration.
       The threshold must be wide enough to capture the velocity overshoot
       from a braking cycle, otherwise the motor oscillates in a limit
       cycle across the demand. */
    float_t v_deadzone = 0.5f * j_max * dt;   /* ~0.01 rad/s at 1kHz */

    if (fabsf(v_error) < v_deadzone){
        /* Velocity is at demand, bleed off residual acceleration.
           Apply exactly the jerk needed to zero acceleration in one tick.
           This handles any acceleration magnitude smoothly without
           falling into the bang-bang braking logic. */
        if (fabsf(a) > 0.5f * j_max * dt * dt){
            j = -a / dt;
            if (j >  j_max) j =  j_max;
            if (j < -j_max) j = -j_max;
        } else {
            j = 0.0f;
            a = 0.0f;
        }
    }
    else {
        /* Compute where we'd end up if we started braking now */
        float_t v_brake = computeBrakingVelocity(v, a, j_max);

        /* Would braking overshoot the demand? */
        bool must_brake;
        if (v_error >= 0.0f){
            must_brake = (v_brake > demand + v_deadzone);
        } else {
            must_brake = (v_brake < demand - v_deadzone);
        }

        if (must_brake){
            /* Brake: apply jerk that opposes current acceleration */
            if (a > 0.0f){
                j = -j_max;
            } else if (a < 0.0f){
                j = j_max;
            } else {
                j = (v_error < 0.0f) ? -j_max : j_max;
            }
        }
        else {
            /* Accelerate toward demand */
            if (v_error > 0.0f){
                j = j_max;
            } else {
                j = -j_max;
            }
        }
    }

    /*  Integrate: jerk → acceleration → velocity → position  */

    /* Apply jerk */
    float_t a_new = a + j * dt;

    /* Clamp acceleration to a_max */
    if (a_new >  a_max) a_new =  a_max;
    if (a_new < -a_max) a_new = -a_max;

    /* Recalculate effective jerk after clamping */
    float_t j_eff = (a_new - a) / dt;

    /* Integrate velocity */
    float_t v_new = v + a * dt + 0.5f * j_eff * dt * dt;

    /* Clamp velocity to v_max */
    if (v_new >  h->v_max) v_new =  h->v_max;
    if (v_new < -h->v_max) v_new = -h->v_max;

    /* Integrate position (exact constant-jerk formula) */
    float_t p_new = h->cmd_pos + v * dt + 0.5f * a * dt * dt
                  + (1.0f / 6.0f) * j_eff * dt * dt * dt;

    /* Clamp position to limits if enabled */
    if (h->pos_limits_en){
        if (p_new > h->pos_max) p_new = h->pos_max;
        if (p_new < h->pos_min) p_new = h->pos_min;
    }


    /*  Store state    */


    h->cmd_vel   = v_new;
    h->cmd_accel = a_new;
    h->cmd_pos   = p_new;

    /* Write to tracker so the rest of the system sees consistent state.
       In simulation, this IS the motor.  On hardware, the position
       written here is the setpoint that the PID tracks. */
    tracker->theta_prev = tracker->theta;
    tracker->omega_prev = tracker->omega;
    tracker->accel_prev = tracker->accel;
    tracker->theta = p_new;
    tracker->omega = v_new;
    tracker->accel = a_new;
    tracker->jerk  = j_eff;

    return p_new;
}



/*  Getters */


float_t velCtrlGetVelocity(VelCtrlHandle *h){
    return h->cmd_vel;
}

float_t velCtrlGetAccel(VelCtrlHandle *h){
    return h->cmd_accel;
}

float_t velCtrlGetPosition(VelCtrlHandle *h){
    return h->cmd_pos;
}
