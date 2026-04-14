#include <string.h>
#include "planner.h"
#include "SCurveTrajectory.h"
#include <math.h>
#define SIM_MODE

static PosCtrlHandle plan_buffer_0;
static PosCtrlHandle plan_buffer_1;
static VelocityFilter tracker_buffer;

/* Shared volatile pointers — accessed by both ISR and foreground */
volatile PosCtrlHandle *paths_planned[2];
volatile uint8_t active_plan;
volatile uint8_t inactive_plan;
volatile VelocityFilter *motorTracker;

float_t trajTime = 0.0f;                   // overall time of current ramp
volatile bool plan_ready = false;           // flag for ISR to know a new plan is waiting
float_t targetSetpoint = 0.0f;             // stored target for wandering/forced-braking replans


// Struct prototypes
static switchingTimes currTrajCutoff;

// Forward declarations for functions in SCurveTrajectory.c
extern VelocityFilter* velocityFilterInit(void);
extern float_t selectJerk(uint8_t profilePhase, float_t jerk);
extern void updateVelocityFilter(VelocityFilter *pHandle, PosCtrlHandle *PHandleTEMP);

// Internal helper prototypes (not exposed in header)
static float_t computeDecelDistance(float_t v, float_t a_max, float_t j_max);


/*  plannerInit                                                       */
/*  Static allocation version for STM32.  No malloc.                  */
/*  Call once from main() before starting TIM6.                       */
int plannerInit(void){

    /* Zero-init and configure plan buffer 0 */
    memset(&plan_buffer_0, 0, sizeof(PosCtrlHandle));
    plan_buffer_0.a_max = A_MAX;
    plan_buffer_0.v_max = 0.5f;   /* safe default — call setMaxVelocity() to change */
    plan_buffer_0.j_max = J_MAX;
    plan_buffer_0.dir   = 0;
    plan_buffer_0.theta = 0.0f;

    /* Zero-init and configure plan buffer 1 */
    memset(&plan_buffer_1, 0, sizeof(PosCtrlHandle));
    plan_buffer_1.a_max = A_MAX;
    plan_buffer_1.v_max = 0.3f;
    plan_buffer_1.j_max = J_MAX;
    plan_buffer_1.dir   = 0;
    plan_buffer_1.theta = 0.0f;

    paths_planned[0] = &plan_buffer_0;
    paths_planned[1] = &plan_buffer_1;
    active_plan   = 0;
    inactive_plan = 1;

    /* Zero-init the motor tracker */
    memset(&tracker_buffer, 0, sizeof(VelocityFilter));
    tracker_buffer.alpha_coeff = VEL_FILTER_COEFFICIENT;
    tracker_buffer.Ts          = SAMPLING_TIME_PLANNER;
    motorTracker = &tracker_buffer;

    return 0;
}

/*
This is the main function in planner, that is called once a new setpoint is realized in CAN_Processing Script.
Depending on the current state of the ongoing plan, this section will determine how to adjust the current plan to
accomodate the new setpoint. The new plan which is devised will be created in the "inactive_plan" and the flag
"plan_ready" will be set to true, indicating to the ISR that upon the next interrupt to switch over plans to the
newly prepared one.
*/
void buildNewCurve(float newSetpoint){

    PosCtrlHandle *currPlan = (PosCtrlHandle *)paths_planned[active_plan];
    VelocityFilter *tracker = (VelocityFilter *)motorTracker;

    // Store the target so wandering or forced-braking replan can replace
    targetSetpoint = newSetpoint;

    // If the motor currently has significant acceleration, the virtual history
    // approach produces a trajectory that assumes a=0 at start — but the ISR
    // would integrate from the actual (nonzero) acceleration, causing mismatch.
    // Instead, defer replanning: let the ISR drive acceleration to zero first,
    // then the main loop replans from the clean constant-velocity state.
    float_t accelThreshold = currPlan->j_max * SAMPLING_TIME_PLANNER;
    if (currPlan->isTrajExecuting && fabsf(tracker->accel) > accelThreshold){
        currPlan->tooFastPending = true;
        return;
    }

    // At this point either (a) no trajectory is executing, or
    // (b) a trajectory is executing but acceleration is ~0 (phase IV / constant velocity).
    // In both cases, the motor state has a ≈ 0 so we can plan directly.
    float_t newInitialVel = tracker->omega;
    float_t newInitialPos = tracker->theta;

    calculateNewRamp((PosCtrlHandle *)paths_planned[inactive_plan],
                     (VelocityFilter *)motorTracker,
                     newInitialPos, newInitialVel, newSetpoint);
}

/*
Given a currenlty executing ramp, this function will return what would have been the initial velocity of the ramp,
should it had started with no acceleration. This velocity is essential in order to compute a new ramp, which relies
on the assumption that at the start the motion has no acceleration.
*/
virtualHistory calculateVirtualHistory(PosCtrlHandle *currentMotionPlan, VelocityFilter *motorTracker){

    // Extract current motor state
    float_t sc = motorTracker->theta;
    float_t vc = motorTracker->omega;
    float_t ac = motorTracker->accel;

    // Constraint limits
    float_t jmax = currentMotionPlan->j_max;
    float_t amax = currentMotionPlan->a_max;

    // Virtual initial parameters (to be computed)
    float_t virt_v0 = vc;   // default to current state
    float_t virt_s0 = sc;

    // Calculate virtual history based on current phase
    // Refer to paper Section 4, equations (38)-(45)
    switch(currentMotionPlan->profilePhase){
        case 1:
            // Phase I: 0 < ac < amax, jc = +jmax
            virt_v0 = vc - (ac / 2.0f) * (ac / jmax);
            virt_s0 = sc - (ac / 6.0f) * (ac / jmax) * (ac / jmax);
            break;

        case 2:
            // Phase II: ac = amax, jc = 0
            virt_v0 = vc - (amax / 2.0f) * (amax / jmax);
            virt_s0 = sc - (amax / 6.0f) * (amax / jmax) * (amax / jmax);
            break;

        case 3:
            // Phase III: 0 < ac < amax, jc = -jmax
            virt_v0 = vc - (ac / 2.0f) * (ac / jmax);
            virt_s0 = sc - (ac / 6.0f) * (ac / jmax) * (ac / jmax);
            break;

        case 4:
            virt_v0 = vc;
            virt_s0 = sc;
            break;
        case 5:
            // Phase V: -amax < ac < 0, jc = -jmax   (eq 42, 43)
            currentMotionPlan->isPastTooFast = false;
            virt_v0 = vc + (ac / 2.0f) * (ac / jmax);   // note: ac is negative, so this adds |ac|²/(2*jmax)
            if (virt_v0 > vc){
                currentMotionPlan->isPastTooFast = true;
                virt_s0 = 0.0f; // won't be used
            }
            else{
                virt_s0 = sc - (ac / 6.0f) * (ac / jmax) * (ac / jmax) + virt_v0 * (ac / jmax);
            }
            break;

        case 6:
            currentMotionPlan->isPastTooFast = false;
            virt_v0 = vc + (amax / 2.0f) * (amax / jmax);
            if (virt_v0 > vc){
                currentMotionPlan->isPastTooFast = true;
                virt_s0 = 0.0f;
            }
            else{
                virt_s0 = sc + (amax / 6.0f) * (amax / jmax) * (amax / jmax) - virt_v0 * (amax / jmax);
            }
            break;
        case 7:
            // Phase VII: -amax < ac < 0, jc = +jmax  (same form as V: eq 42, 43)
            currentMotionPlan->isPastTooFast = false;
            virt_v0 = vc + (ac / 2.0f) * (ac / jmax);
            if (virt_v0 > vc){
                currentMotionPlan->isPastTooFast = true;
                virt_s0 = 0.0f;
            }
            else{
                virt_s0 = sc - (ac / 6.0f) * (ac / jmax) * (ac / jmax) + virt_v0 * (ac / jmax);
            }
            break;
        default:
            break;
    }

    // FIX: return by value, not pointer to local
    virtualHistory result = {virt_v0, virt_s0};
    return result;
}
/*
   computeDecelDistance   (helper)
   Computes the distance needed to decelerate from velocity v to 0
   under constraints a_max and j_max. This is Δs_47 from the paper (eq 17)
   but generalized to any peak velocity v (not just v_max).

*/
static float_t computeDecelDistance(float_t v, float_t a_max, float_t j_max){
    float_t v_thresh = (a_max * a_max) / j_max;
    if (v >= v_thresh){
        // Reaches full -a_max during decel
        // Δs = (v/2) * (v/a_max + a_max/j_max)
        return (v / 2.0f) * (v / a_max + a_max / j_max);
    } else {
        // Jerk-limited decel, doesn't reach -a_max
        // Δs = v * sqrt(v / j_max)
        return v * sqrtf(v / j_max);
    }
}

/*
Given the initial conditions of the motor, assuming 0 acceeleration, and the setpoint, this function returns
the critical cutoff times for each of the 7 steps in the S-Curve. This will be used to inform which current phase of
the ramp the motor is in, and thus the jerk to apply in order to get to the target position. Note if an initial acceleration
is present, then calculateVirtualHistory muust first be called in order to properly simulate the proper condition.
*/
switchingTimes determineSwitchingTimes(float_t targetPos, float_t s0, float_t v0, PosCtrlHandle *currentMotionPlan,  VelocityFilter *motorTracker){

    float_t a_max = currentMotionPlan->a_max;
    float_t j_max = currentMotionPlan->j_max;
    float_t v_max = currentMotionPlan->v_max;

    // Direction normalization: work in the positive-toward-target frame
    float_t dir  = (targetPos - s0 >= 0) ? 1.0f : -1.0f;
    float_t v0n  = dir * v0;                    // velocity component toward target (+ = toward)
    float_t sEn  = dir * (targetPos - s0);      // distance to target (>= 0)

    // Moving AWAY from target: brake to stop, then replan.
    // Use actual motor velocity for the braking profile, not the virtual v0,
    // because the braking trajectory must match the motor's real kinetic state.
    if (v0n < 0.0f) {
        currTrajCutoff.isWandering = true;
        float_t actualVel = motorTracker->omega;
        return decellerate2Stop(targetPos, s0, actualVel, currentMotionPlan, motorTracker);
    }

    // Useful threshold: velocity at which a_max is just reached during a jerk ramp
    float_t v_thresh_a = (a_max * a_max) / j_max;

    // Phase durations (to be filled)
    float_t dt1 = 0.0f, dt2 = 0.0f, dt3 = 0.0f, dt4 = 0.0f;
    float_t dt5 = 0.0f, dt6 = 0.0f, dt7 = 0.0f;

    // CASE E: Must decelerate immediately: can't stop in time
    //         (minimum stopping distance exceeds remaining distance)

    float_t deltaSmin = computeDecelDistance(v0n, a_max, j_max);

    if (sEn < deltaSmin) {
        // Can't stop before target, must overshoot then come back (wander).
        // Use actual motor velocity for braking profile.
        currTrajCutoff.isWandering = true;
        float_t actualVel = motorTracker->omega;
        return decellerate2Stop(targetPos, s0, actualVel, currentMotionPlan, motorTracker);
    }

    // From here: v0n >= 0 and we have enough distance. Classify into cases c, b, a, d.

    // Precompute the deceleration distance from v_max to 0 (Δs_47 at full speed)
    // Paper eq (17): Δs_47 = (v_max/2) * (v_max/a_max + a_max/j_max)
    float_t deltaS47_vmax = computeDecelDistance(v_max, a_max, j_max);

    // Velocity deficit to reach v_max — determines case classification
    float_t dv = v_max - v0n;

    // CASE C: v_max reached WITHOUT hitting a_max
    //         (Δv is so small that the jerk ramp alone covers it)
    //         Paper: Δt1 < a_max/j_max, i.e. dv < v_thresh_a

    if (dv > 0.0f && dv < v_thresh_a) {
        // Phase I and III are shortened, no phase II
        float_t dt1_c = sqrtf(dv / j_max);        // eq (30): Δt1 = sqrt((v_max - v0)/j_max)
        float_t dt3_c = dt1_c;                     // symmetric

        // Δs_03 for case c: eq (31)
        float_t deltaS03_c = (v_max + v0n) * dt1_c;

        if (sEn >= deltaS03_c + deltaS47_vmax) {
            // Case c confirmed: all distance checks pass
            dt1 = dt1_c;
            dt2 = 0.0f;                            // no constant-accel phase
            dt3 = dt3_c;
            dt4 = (sEn - deltaS03_c - deltaS47_vmax) / v_max;  // eq (18)

            // Decel from v_max: symmetric to accel but v_max to 0
            if (v_max >= v_thresh_a) {
                dt5 = a_max / j_max;
                dt6 = v_max / a_max - a_max / j_max;
                dt7 = a_max / j_max;
            } else {
                dt5 = sqrtf(v_max / j_max);
                dt6 = 0.0f;
                dt7 = dt5;
            }

            goto fill_struct;
        }
        // If distance insufficient for case c, fall through to case b check
    }

    // CASE B: a_max IS reached, but v_max is NOT
    //         (phases I, II, III present; IV omitted; v3 < v_max)
    //         Condition: dv >= v_thresh_a (so a_max is reached) AND
    //                    not enough distance for full v_max

    if (dv >= v_thresh_a) {
        // Check if there's enough distance for case a first (both a_max and v_max reached)
        // Δs_03 for case a: eq (15)
        float_t deltaS03_a = (v_max / 2.0f) * ((v_max - 2.0f * v0n) / a_max + a_max / j_max);

        if (sEn >= deltaS03_a + deltaS47_vmax) {
            // CASE A: Both a_max and v_max reached so full 7-phase profile
            dt1 = a_max / j_max;                                     // eq (10)
            dt2 = (v_max - 2.0f * v0n) / a_max - a_max / j_max;    // eq (14), simplified from (v2-v1)/a_max
            if (dt2 < 0.0f) dt2 = 0.0f;                             // safety clamp
            dt3 = dt1;                                               // symmetric
            dt4 = (sEn - deltaS03_a - deltaS47_vmax) / v_max;      // eq (18)

            // Decel phases (V-VII) from v_max
            dt5 = a_max / j_max;
            dt6 = v_max / a_max - a_max / j_max;
            if (dt6 < 0.0f) dt6 = 0.0f;
            dt7 = a_max / j_max;

            goto fill_struct;
        }
        else {
            // CASE B: a_max reached but not enough distance for v_max
            // We need to solve for the peak velocity v3 < v_max
            // From paper eqs (19)-(26):
            //   dt1 = dt3 = a_max / j_max
            //   v1 = v0n + (a_max/2)*(a_max/j_max)                 (eq 20)
            //   v3 = v1 + a_max * dt2, and v3 is also the peak = decel start velocity
            //   sEn = Δs_03(v3) + Δs_47(v3)
            //
            // Solving for v3 from eq (24):
            //   v3 = (a_max/2) * sqrt( (a_max/j_max - v0n/a_max)^2 + 4*sEn/a_max - a_max^2/(2*j_max^2) + v0n/2 )
            //   ... actually the paper gives eq (24) directly. Let's implement it.
            //
            // Simpler approach: solve the quadratic from sEn = Δs_03(v3) + Δs_47(v3)
            // where both are functions of v3, with dt1=dt3=dt5=dt7=a_max/j_max

            float_t T_j = a_max / j_max;   // jerk time for phases I, III, V, VII
            float_t v1 = v0n + 0.5f * a_max * T_j;  // velocity at end of phase I (eq 20)

            float_t A = 1.0f / a_max;
            float_t B = a_max / j_max - v0n / a_max;
            float_t discriminant = B * B + 4.0f * A * sEn;

            float_t v3;
            if (discriminant < 0.0f) {
                // Shouldn't happen if case e already filtered, but safety
                v3 = v0n;
            } else {
                v3 = (-B + sqrtf(discriminant)) / (2.0f * A);
            }

            // Clamp to v_max (shouldn't exceed, but numerical safety)
            if (v3 > v_max) v3 = v_max;
            if (v3 < v0n) v3 = v0n;  // can't go below starting speed

            // Now compute phase durations
            dt1 = T_j;
            dt2 = (v3 - v1 - 0.5f * a_max * T_j) / a_max;  // time in constant accel
            // Simplify: v3 = v1 + a_max*dt2 + 0.5*a_max*T_j  (from phase II + phase III)
            // Actually: v1 = v0n + 0.5*a_max*T_j
            //           v3 = v1 + a_max*dt2 + 0.5*a_max*T_j  (phase II gives a_max*dt2, phase III gives 0.5*a_max*T_j)
            // So dt2 = (v3 - v1 - 0.5*a_max*T_j) / a_max
            //        = (v3 - v0n - 0.5*a_max*T_j - 0.5*a_max*T_j) / a_max
            //        = (v3 - v0n - a_max*T_j) / a_max
            //        = (v3 - v0n)/a_max - T_j
            dt2 = (v3 - v0n) / a_max - T_j;
            if (dt2 < 0.0f) dt2 = 0.0f;
            dt3 = T_j;
            dt4 = 0.0f;    // no cruising phase in case b

            // Decel from v3 to 0
            if (v3 >= v_thresh_a) {
                dt5 = T_j;
                dt6 = v3 / a_max - T_j;
                if (dt6 < 0.0f) dt6 = 0.0f;
                dt7 = T_j;
            } else {
                dt5 = sqrtf(v3 / j_max);
                dt6 = 0.0f;
                dt7 = dt5;
            }

            goto fill_struct;
        }
    }
    else if (dv > 0.0f) {
        // dv > 0 but dv < v_thresh_a, and case c distance check failed above.
        // This means we can't even reach v_max with a short jerk ramp, and there isn't
        // enough distance for a cruising phase. Falls into case d territory.
    }

    // CASE D: Neither a_max nor v_max can be reached
    //         (very short move or conditions for b and c not met)
    //         Paper: suboptimal strategy — hold v0 constant for a distance,
    //         then decelerate.

    {
        if (v0n < 0.001f) {
            // Starting from rest with very short distance:
            // Pure triangular jerk profile — accelerate then decelerate
            // Peak velocity v_peak where: 2 * v_peak * sqrt(v_peak / j_max) = sEn
            // v_peak^(3/2) = sEn * sqrt(j_max) / 2
            // v_peak = (sEn * sqrt(j_max) / 2)^(2/3)
            float_t v_peak = powf(sEn * sqrtf(j_max) / 2.0f, 2.0f / 3.0f);
            if (v_peak > v_max) v_peak = v_max;

            float_t dt_accel = sqrtf(v_peak / j_max);

            dt1 = dt_accel;
            dt2 = 0.0f;
            dt3 = dt_accel;
            dt4 = 0.0f;
            dt5 = dt_accel;
            dt6 = 0.0f;
            dt7 = dt_accel;
        }
        else {
            // Paper's suboptimal strategy: hold constant v0 for computed distance, then decel
            // Case d1 (v0 >= a_max^2/j_max): Δt4 = sEn/v0n - (1/2)*(v0n/a_max + a_max/j_max)  (eq 34)
            // Case d2 (v0 < a_max^2/j_max):  Δt4 = sEn/v0n - sqrt(v0n/j_max)                  (eq 35)

            if (v0n >= v_thresh_a) {
                // Case d1
                dt4 = sEn / v0n - 0.5f * (v0n / a_max + a_max / j_max);
            } else {
                // Case d2
                dt4 = sEn / v0n - sqrtf(v0n / j_max);
            }
            if (dt4 < 0.0f) dt4 = 0.0f;

            // No accel phases (already at v0, just cruise then decel)
            dt1 = 0.0f;
            dt2 = 0.0f;
            dt3 = 0.0f;

            // Decel from v0n to 0
            if (v0n >= v_thresh_a) {
                dt5 = a_max / j_max;
                dt6 = v0n / a_max - a_max / j_max;
                if (dt6 < 0.0f) dt6 = 0.0f;
                dt7 = a_max / j_max;
            } else {
                dt5 = sqrtf(v0n / j_max);
                dt6 = 0.0f;
                dt7 = dt5;
            }
        }
    }

fill_struct:
    // Convert phase durations to absolute (cumulative) switching times
    currTrajCutoff.isWandering = false;
    currTrajCutoff.t0 = 0.0f;
    currTrajCutoff.t1 = dt1;
    currTrajCutoff.t2 = dt1 + dt2;
    currTrajCutoff.t3 = dt1 + dt2 + dt3;
    currTrajCutoff.t4 = dt1 + dt2 + dt3 + dt4;
    currTrajCutoff.t5 = dt1 + dt2 + dt3 + dt4 + dt5;
    currTrajCutoff.t6 = dt1 + dt2 + dt3 + dt4 + dt5 + dt6;
    currTrajCutoff.t7 = dt1 + dt2 + dt3 + dt4 + dt5 + dt6 + dt7;

    return currTrajCutoff;
}


/*
This function is used to decellerate to a stop. It can be called either to stop the motor, or if the motor needs to switch directions
to get to a setpoint. This happens if the next setpoint is in the opposite direction, or if the motor cannot slow down in time to get to
the desired target, and must stop and reprogram.
*/
switchingTimes decellerate2Stop(float_t targetPos, float_t s0, float_t v0,  PosCtrlHandle *currentMotionPlan, VelocityFilter *motorTracker){

    // Parameters kept for interface consistency; not needed for pure braking
    (void)targetPos;
    (void)s0;
    (void)motorTracker;

    float_t a_max = currentMotionPlan->a_max;
    float_t j_max = currentMotionPlan->j_max;

    float_t v = fabsf(v0);
    float_t dt5 = 0.0f;
    float_t dt6 = 0.0f;
    float_t dt7 = 0.0f;

    // Determine braking profile based on whether a_max is reached during braking
    float_t v_thresh = (a_max * a_max) / j_max;
    if (v >= v_thresh) {
        // Case e1: reaches max deceleration
        dt5 = a_max / j_max;
        dt6 = v / a_max - a_max / j_max;
        dt7 = a_max / j_max;
    } else {
        // Case e2: jerk-limited, does not reach max deceleration
        dt5 = sqrtf(v / j_max);
        dt6 = 0.0f;
        dt7 = dt5;
    }

    // Populate switching times — phases I-IV are zero (decel only)
    currTrajCutoff.t0 = 0.0f;
    currTrajCutoff.t1 = 0.0f;
    currTrajCutoff.t2 = 0.0f;
    currTrajCutoff.t3 = 0.0f;
    currTrajCutoff.t4 = 0.0f;
    currTrajCutoff.t5 = dt5;
    currTrajCutoff.t6 = dt5 + dt6;
    currTrajCutoff.t7 = dt5 + dt6 + dt7;
    currTrajCutoff.isWandering = true;   // caller has already set this, but be explicit

    return currTrajCutoff;
}

/*
   calculateNewRamp
   Takes the computed switching times and populates the inactive plan handle
   with everything the ISR needs to execute the trajectory.
 */
void calculateNewRamp(PosCtrlHandle *newPlan, VelocityFilter *motorTracker, float_t s0, float_t v0, float_t targetPos){

    // Compute the switching times for this move
    switchingTimes times = determineSwitchingTimes(targetPos, s0, v0, newPlan, motorTracker);

    // Direction of motion.
    // For normal trajectories: direction is toward the target.
    // For wandering (braking) profiles: direction must be the direction of
    // current velocity, so the decel phases (5-7) produce jerk that opposes
    // the motor's motion and brings it to a stop.
    float_t dir;
    if (times.isWandering){
        // Use actual motor velocity direction for braking
        dir = (motorTracker->omega >= 0.0f) ? 1.0f : -1.0f;
    } else {
        dir = (targetPos - s0 >= 0.0f) ? 1.0f : -1.0f;
    }

    // Populate the plan's switching times array [t0, t1, ... t7]
    newPlan->profileSwitchingTimes[0] = times.t0;
    newPlan->profileSwitchingTimes[1] = times.t1;
    newPlan->profileSwitchingTimes[2] = times.t2;
    newPlan->profileSwitchingTimes[3] = times.t3;
    newPlan->profileSwitchingTimes[4] = times.t4;
    newPlan->profileSwitchingTimes[5] = times.t5;
    newPlan->profileSwitchingTimes[6] = times.t6;
    newPlan->profileSwitchingTimes[7] = times.t7;

    // Set trajectory metadata
    newPlan->dir = (int)dir;
    newPlan->isWandering = times.isWandering;
    newPlan->isTrajExecuting = true;
    newPlan->isPastTooFast = false;
    newPlan->tooFastPending = false;
    newPlan->wanderReplanPending = false;
    newPlan->profilePhase = 0;   // will be set to 1 on first ISR step

    // Use the actual motor position as the starting point for the ISR.
    // The virtual s0 was only needed for switching time computation.
    // Using virtual s0 here would cause a position discontinuity.
    newPlan->theta = motorTracker->theta;

    // Signal the ISR that a new plan is ready to swap in
    plan_ready = true;
}

/*
   PosCtrl_ISRStep
   Called at 1 kHz from the timer interrupt (TIM6). Executes one timestep of the
   active trajectory plan:
     1. Swap plans if a new one is ready
     2. Determine current phase from elapsed time
     3. Select jerk for this phase
     4. Integrate jerk -> accel -> velocity -> position
     5. Update the velocity filter with the actual encoder reading
     6. Raise any replan flags for the main loop
*/
void PosCtrl_ISRStep(void){

    PosCtrlHandle *plan = (PosCtrlHandle *)paths_planned[active_plan];
    VelocityFilter *tracker = (VelocityFilter *)motorTracker;

    // Swap plans if a new one is waiting
    if (plan_ready){
        // Swap active and inactive indices
        uint8_t temp = active_plan;
        active_plan = inactive_plan;
        inactive_plan = temp;

        plan = (PosCtrlHandle *)paths_planned[active_plan];
        plan_ready = false;
        trajTime = 0.0f;    // reset trajectory clock for the new plan
    }

    // If no trajectory is executing, nothing to do
    if (!plan->isTrajExecuting){
        return;
    }

    float_t t_start = trajTime;
    float_t t_final = plan->profileSwitchingTimes[7];

    // "Too fast" correction per paper Section 4, cases v/vi/vii:
    // Apply jerk to drive acceleration toward zero. Once |a| is small enough,
    // the main loop will detect this and replan from the current (v, s) with a≈0.
    // This MUST be checked before the t_final completion check, because we are
    // overriding the original trajectory — the original switching times are stale.
    if (plan->tooFastPending){

        float_t dt = SAMPLING_TIME_PLANNER;
        float_t s_curr = plan->theta;
        float_t v_curr = tracker->omega;
        float_t a_curr = tracker->accel;
        float_t j = 0.0f;

        // Apply jerk to reduce |acceleration| toward zero
        if (a_curr < -0.5f * plan->j_max * dt){
            // Negative acceleration: apply positive jerk to bring it up toward 0
            j = plan->j_max;
            plan->profilePhase = 7;
        }
        else if (a_curr > 0.5f * plan->j_max * dt){
            // Positive acceleration: apply negative jerk to bring it down toward 0
            j = -plan->j_max;
            plan->profilePhase = 3;
        }
        else{
            // Acceleration is essentially zero — snap to zero and let main loop replan
            j = 0.0f;
            a_curr = 0.0f;
            plan->profilePhase = 4;
            // Clear the too-fast state so main loop can replan
            plan->isPastTooFast = false;
        }

        // Exact integration of constant-jerk kinematics
        s_curr = s_curr + v_curr * dt + 0.5f * a_curr * dt * dt + (1.0f / 6.0f) * j * dt * dt * dt;
        v_curr = v_curr + a_curr * dt + 0.5f * j * dt * dt;
        a_curr = a_curr + j * dt;

        // On hardware: s_curr is the commanded position (setpoint for PID).
        // The tracker should read the actual encoder, not the command.
        plan->theta = s_curr;          // commanded position (always needed)
//        plan->profilePhase = lastPhase;

        #ifdef SIM_MODE
            // In simulation, the ISR IS the motor — write state directly
            tracker->theta_prev = tracker->theta;
            tracker->omega_prev = tracker->omega;
            tracker->accel_prev = tracker->accel;
            tracker->theta = s_curr;
            tracker->omega = v_curr;
            tracker->accel = a_curr;
            tracker->jerk  = (a_curr - tracker->accel_prev) / SAMPLING_TIME_PLANNER;
        #else
            // On hardware, read the actual encoder
            updateVelocityFilter(tracker, plan);
        #endif

        trajTime += dt;
        return;
    }

    // If the active plan is complete, stop cleanly and raise any main-loop work flags.
    if (t_start >= t_final){
        plan->isTrajExecuting = false;
        plan->profilePhase = 0;
        tracker->omega = 0.0f;
        tracker->accel = 0.0f;
        tracker->jerk  = 0.0f;

        if (plan->isWandering){
            plan->isWandering = false;
            plan->wanderReplanPending = true;
        }
        return;
    }

    // Execute one ISR tick, but split it at any internal switching boundaries
    // crossed within the tick so each sub-interval uses the correct constant jerk.
    float_t t_tick_end = t_start + SAMPLING_TIME_PLANNER;
    if (t_tick_end > t_final){
        t_tick_end = t_final;
    }

    float_t t_curr = t_start;
    float_t s_curr = plan->theta;
    float_t v_curr = tracker->omega;
    float_t a_curr = tracker->accel;
    uint8_t lastPhase = 0;

    while (t_curr < t_tick_end){
        uint8_t phase = 7;
        if      (t_curr < plan->profileSwitchingTimes[1]) phase = 1;
        else if (t_curr < plan->profileSwitchingTimes[2]) phase = 2;
        else if (t_curr < plan->profileSwitchingTimes[3]) phase = 3;
        else if (t_curr < plan->profileSwitchingTimes[4]) phase = 4;
        else if (t_curr < plan->profileSwitchingTimes[5]) phase = 5;
        else if (t_curr < plan->profileSwitchingTimes[6]) phase = 6;
        else                                              phase = 7;

        float_t t_boundary = plan->profileSwitchingTimes[phase];
        float_t t_next = (t_boundary < t_tick_end) ? t_boundary : t_tick_end;

        // Guard against zero-length sub-steps from equal switching times.
        if (t_next <= t_curr){
            t_curr = t_tick_end;
            break;
        }

        float_t dt = t_next - t_curr;
        float_t j = selectJerk(phase, plan->j_max) * (float_t)plan->dir;

        // Exact integration of constant-jerk kinematics over this sub-step.
        s_curr = s_curr + v_curr * dt + 0.5f * a_curr * dt * dt + (1.0f / 6.0f) * j * dt * dt * dt;
        v_curr = v_curr + a_curr * dt + 0.5f * j * dt * dt;
        a_curr = a_curr + j * dt;

        t_curr = t_next;
        lastPhase = phase;
    }

    trajTime = t_tick_end;

    // On hardware: s_curr is the commanded position (setpoint for PID).
    // plan->theta stores the command; updateVelocityFilter reads the
    // actual encoder and computes real velocity/accel/jerk derivatives.
    plan->theta = s_curr;
    plan->profilePhase = lastPhase;

    #ifdef SIM_MODE
        // In simulation, the ISR IS the motor — write state directly
        tracker->theta_prev = tracker->theta;
        tracker->omega_prev = tracker->omega;
        tracker->accel_prev = tracker->accel;
        tracker->theta = s_curr;
        tracker->omega = v_curr;
        tracker->accel = a_curr;
        tracker->jerk  = (a_curr - tracker->accel_prev) / SAMPLING_TIME_PLANNER;
    #else
        // On hardware, read the actual encoder
        updateVelocityFilter(tracker, plan);
    #endif

    // If the tick landed exactly on the end of the ramp, finish cleanly.
    if (trajTime >= t_final){
        plan->isTrajExecuting = false;
        plan->profilePhase = 0;
        tracker->omega = 0.0f;
        tracker->accel = 0.0f;
        tracker->jerk  = 0.0f;

        if (plan->isWandering){
            plan->isWandering = false;
            plan->wanderReplanPending = true;
        }
    }
}



/*  Planner_MainStep                                                  */
/*  Foreground service function — call from the main while(1) loop.   */
/*                                                                    */
/*  Responsibilities:                                                 */
/*   1. Handle a newly requested setpoint                             */
/*   2. Handle deferred wandering replans                             */
/*   3. Handle deferred too-fast replans                              */
/*                                                                    */
/*  The ISR should stay focused on executing the active motion        */
/*  profile.  All replanning work happens here in thread context.     */

void Planner_MainStep(volatile bool *newCommandPending, float requestedSetpoint){

    PosCtrlHandle *activePlan = (PosCtrlHandle *)paths_planned[active_plan];

    // A new command arrived so try to build the new curve in foreground context
    if (*newCommandPending){
        *newCommandPending = false;
        buildNewCurve(requestedSetpoint);
        return;
    }

    // wandering stop completed in the ISR, so foreground code now creates the follow-up ramp
    if (activePlan->wanderReplanPending){
        activePlan->wanderReplanPending = false;
        buildNewCurve(requestedSetpoint);
        return;
    }

    //The ISR is running forced braking to bring acceleration to zero.
    // Once |a| is small enough, replan from the current motor state.
    if (activePlan->tooFastPending){
        VelocityFilter *tracker = (VelocityFilter *)motorTracker;
        float_t accelThreshold = activePlan->j_max * SAMPLING_TIME_PLANNER;
        if (fabsf(tracker->accel) <= accelThreshold){
            activePlan->tooFastPending = false;
            // Motor is now at approximately constant velocity (a ≈ 0).
            // Replan directly from actual motor state.
            calculateNewRamp((PosCtrlHandle *)paths_planned[inactive_plan],
                             (VelocityFilter *)motorTracker,
                             tracker->theta, tracker->omega, targetSetpoint);
        }
    }
}
