#include <string.h>
#include "planner.h"
#include "SCurveTrajectory.h"
#include <math.h>         

#define POS_TOL 0.002 // arnd 0.1 degrees
#define VEL_TOL 0.02 //rad /s

#ifndef __disable_irq
#define __disable_irq() do{}while(0)
#endif
#ifndef __enable_irq
#define __enable_irq()  do{}while(0)
#endif

//Function prototypes:
static void build_ACD(PosCtrlHandle *out, float theta0, float w0, float target, float Ts);


// plan_active always points to one of these two.
static PosCtrlHandle plan_bank[2];
static uint8_t       bank_idx = 0;         // 0 or 1
// Definition matches header: volatile pointer
volatile PosCtrlHandle *  plan_active = &plan_bank[0];
volatile uint8_t         plan_ready  = 0; // ISR will clear this one when it acknowledges

// Internal flags
static volatile uint8_t replan_request = 0;   // set by Planner_RequestReplan()
static float omega_cruise = 0.08727f;             // default cruise speed [rad/s], adjust to taste
static float A_max = 5.0f;                    // [rad/s^2] default; override via Planner_SetLimits
static float J_max = 500.0f;                  // [rad/s^3] default; override via Planner_SetLimits

//External shared state
extern volatile PosCtrlHandle SCurveTrajectory;   // active live state (θ/ω/a updated in ISR)
extern volatile VelocityFilter motorTracker;      // provides ω, accel estimates
extern volatile float positionSetpoint;           // latest target angle [rad]
extern volatile bool  newSetpointDetected;        // set by CAN handler

// helper function
static inline int8_t sign_db(float x, float band) {
    return (x > band) - (x < -band);
}

// // Build a complete S-curve plan from the snapshot (θ0, ω0) to target.
// static void build_plan(PosCtrlHandle *out,
//                        float theta0, float omega0, float accel0,
//                        float target, float Ts)
// {
//     memset(out, 0, sizeof(*out));

//     out->SamplingTime       = Ts;
//     out->StartingAngle      = theta0;
//     out->FinalAngle         = target;
//     out->AngleStep          = target - theta0;

//     // Choose a duration. Simple first pass: use your estimator (ms) then convert to s.
//     const float T_ms = getEstimatedTrajectoryTime(theta0, target, omega_cruise);
//     const float T_s  = (T_ms <= 1.0f) ? 0.009f : (T_ms * 0.001f); // minimum 9*Ts guard

//     // Compute 9-segment parameters
//     computeTrajectoryParameters(out, theta0, T_s, out->AngleStep);

//     // Seed current state from snapshot
//     out->Theta        = theta0;
//     out->Omega        = omega0;
//     out->Acceleration = accel0;

//     // Physical/comfort limits (optional: clamp computed jerk/accel to these)
//     out->A_MAX  = A_max;
//     out->J_MAX  = J_max;

//     // Guidance tolerances and sign
//     out->posTol = (out->posTol > 0.0f) ? out->posTol : 0.002f;  // ~0.11°
//     out->velTol = (out->velTol > 0.0f) ? out->velTol : 0.02f;   // rad/s
//     out->remainingDistance = out->AngleStep;
//     out->sign     = sign_db(out->remainingDistance, out->posTol);
//     out->sign_prev= out->sign;

//     out->isExecutingTrajectory = true;
// }

// Publish plan_pending into inactive bank and switch plan_active atomically.
// ISR does not swap pointers; it only looks at plan_ready to reset its timing.
static void publish_plan(const PosCtrlHandle *pending)
{
    const uint8_t next = bank_idx ^ 1;
    __disable_irq();
    plan_bank[next] = *pending;       // small struct copy
    bank_idx        = next;
    plan_active     = &plan_bank[bank_idx];
    plan_ready      = 1;              // ISR will clear after acknowledging
    __enable_irq();
}

// Public API

void Planner_Init(void)
{
    // Initialize the two banks to a benign hold state
    // Planner_Init()
    memset(&plan_bank[0], 0, sizeof(plan_bank));
    plan_bank[0].SamplingTime = (SCurveTrajectory.SamplingTime>0.0f)?SCurveTrajectory.SamplingTime: SAMPLING_TIME;
    plan_bank[0].posTol = 0.002f;
    plan_bank[0].velTol = 0.02f;
    plan_bank[0].isExecutingTrajectory = false;
    plan_bank[1] = plan_bank[0];
    bank_idx     = 0;
    plan_active  = &plan_bank[0];
    plan_ready   = 0;

    // Default knobs (can be overridden later)
    omega_cruise = (SCurveTrajectory.CruiseSpeed > 0.0f) ? SCurveTrajectory.CruiseSpeed : 0.08727f;
    A_max = (SCurveTrajectory.A_MAX > 0.0f) ? SCurveTrajectory.A_MAX : A_max;
    J_max = (SCurveTrajectory.J_MAX > 0.0f) ? SCurveTrajectory.J_MAX : J_max;
}

void Planner_RequestReplan(void)
{
    replan_request = 1;
}

void Planner_SetCruiseSpeed(float omega_cruise_rad_s)
{
    omega_cruise = (omega_cruise_rad_s > 0.0f) ? omega_cruise_rad_s : omega_cruise;
}

void Planner_SetLimits(float a_max_rad_s2, float j_max_rad_s3)
{
    if (a_max_rad_s2 > 0.0f) A_max = a_max_rad_s2;
    if (j_max_rad_s3 > 0.0f) J_max = j_max_rad_s3;
}

void Planner_BackgroundTask(void)
{
    // Consume flags 
    bool need = false;

    if (newSetpointDetected) {
        newSetpointDetected = false;
        need = true;
    }
    if (replan_request) {
        replan_request = 0;
        need = true;
    }
    if (!need) return;

    // Snapshot current state once (no heavy work in IRQ context)
    const float Ts     = (SCurveTrajectory.SamplingTime > 0.0f) ? SCurveTrajectory.SamplingTime : SAMPLING_TIME;
    const float theta0 = SCurveTrajectory.Theta;     // latest angle
    const float omega0 = motorTracker.omega;         // filtered velocity
    const float target = positionSetpoint;

    // Decide strategy (simple version): always re-build a full plan from current state to target.
    // If you want ACCEL/CRUISE/DECEL truncation or BRAKE-then-relaunch for opposite direction,
    // compute that logic here and adjust the duration you pass into computeTrajectoryParameters().
    PosCtrlHandle pending;
    build_ACD(&pending, theta0, omega0, target, Ts);

    // Atomically publish for the ISR
    publish_plan(&pending);
}


// Dealing with edge cases 

// This function is used to select which segment length A to use for deccel/braking
static float choose_A(float A_max, float J_max, float Ts)
{
    // Nominal A = time to ramp accel from 0 to A_max with jerk J_max
    float A = A_max / J_max;                // [s]
    // Guardrails
    if (A < 5.0f*Ts) A = 5.0f*Ts;          // ensure multiple samples per sub-step
    // Snap A to multiples of Ts to align segment boundaries
    A = floorf(A / Ts) * Ts;
    if (A < Ts) A = Ts;
    return A;
}

/*
This function is used to simulate how long it owuld take to slow down, given its using 
the max acceleration to slow down.
*/
static float decel_distance_from_speed(float w0, float J, float A, float Ts)
{
    // Symmetric jerk-limited braking from w0 -> 0, a starts at 0
    float a = 0.0f, w = w0, x = 0.0f;
    int maxIter = (int)(10.0f * A / Ts);   // 10× sub-step duration worth of samples
    if (maxIter < 100) maxIter = 100;      // minimum safety floor

    enum { PH_RAMP_DOWN, PH_HOLD_NEG_A, PH_RAMP_UP } ph = PH_RAMP_DOWN;

    // Simulate how the motor planner works here
    for (int i = 0; i < maxIter && w > 0.0f; ++i) {
        float j = 0.0f;
        switch (ph) {
            case PH_RAMP_DOWN: j = -J; a += j*Ts; if (a <= -A) { a = -A; ph = PH_HOLD_NEG_A; } break;
            case PH_HOLD_NEG_A: j = 0.0f;
                // heuristic switch to ramp-up when remaining w is small enough
                if (w <= (A*A)/J) ph = PH_RAMP_UP;
                break;
            case PH_RAMP_UP: j = +J; a += j*Ts; if (a >= 0.0f) { a = 0.0f; } break;
        }
        w += a*Ts; if (w < 0.0f) w = 0.0f;
        x += w*Ts;
        if (a == 0.0f && w == 0.0f) break;
    }
    return x; // radians
}


typedef enum { PLAN_ACCEL_CRUISE_DECEL, PLAN_CRUISE_DECEL, PLAN_DECEL_ONLY, PLAN_BRAKE_TO_ZERO } PlanKind;

static PlanKind decide_kind(float delta, float w0, float posTol, float velTol)
{
    const float sgn = (delta > posTol) ? +1.0f : (delta < -posTol) ? -1.0f : 0.0f;
    if (sgn == 0.0f) return (fabsf(w0) > velTol) ? PLAN_DECEL_ONLY : PLAN_DECEL_ONLY;

    // Opposite direction?
    if (w0 * sgn < -velTol) return PLAN_BRAKE_TO_ZERO;

    // Same direction: choose CRUISE_DECEL if already near cruise; else ACCEL_CRUISE_DECEL
    return (fabsf(w0) >= omega_cruise) ? PLAN_CRUISE_DECEL : PLAN_ACCEL_CRUISE_DECEL;
}


// Integrate a candidate S-profile and return covered distance (rad).
// modes: accel(3A) -> cruise(N*A) -> decel(3A). If skip_accel==true, we keep jerk=0 in the accel window.
// If cruiseN == 0 there is no constant-omega plateau.
static float preview_distance(float w0, float Jabs, float A, float Ts,
                              int cruiseN, bool skip_accel)
{
    const float J = Jabs;
    float a = 0.0f, w = w0, x = 0.0f;
    const int nA = (int)roundf(A / Ts);

    // 3A accel lobe
    for (int seg=0; seg<3; ++seg) {
        for (int i=0; i<nA; ++i) {
            float j = 0.0f;
            if (!skip_accel) {
                if (seg==0) j = +J;
                if (seg==1) j =  0.0f;
                if (seg==2) j = -J;
            }
            a += j*Ts;
            w += a*Ts;
            x += w*Ts;
        }
    }

    // cruise N*A (zero jerk, zero accel)
    for (int c=0; c<cruiseN*nA; ++c) {
        x += w*Ts;
    }

    // 3A decel lobe (mirror)
    for (int seg=0; seg<3; ++seg) {
        for (int i=0; i<nA; ++i) {
            float j = 0.0f;
            if      (seg==0) j = -J;
            else if (seg==1) j =  0.0f;
            else             j = +J;
            a += j*Ts;
            w += a*Ts;
            if (w < 0.0f) w = 0.0f; // don't go negative due to discretization
            x += w*Ts;
        }
    }
    return x;
}


void build_ACD(PosCtrlHandle *out, float theta0, float w0, float target, float Ts)
{
    const float delta = target - theta0;
    const float sgn   = (delta >= 0.0f) ? +1.0f : -1.0f;

    // 1) Choose A and J
    float A  = choose_A(A_max, J_max, Ts);
    float J  = sgn * J_max;

    // 2) Decide initial macro kind from current state
    PlanKind kind = decide_kind(delta, w0, out->posTol, out->velTol);

    // 3) If opposite direction: publish BRAKE plan now (3A decel) and return; planner can chain the next plan after completion.
    if (kind == PLAN_BRAKE_TO_ZERO) {
        // Keep accel lobe as "decel only" toward zero: treat target=theta0 and just decelerate speed to ~0.
        // Implement as a DECEL_ONLY profile:
        // - accel window: jerk=0 (skip_accel=true)
        // - cruiseN = 0
        // - decel lobe active
        out->StartingAngle = theta0;
        out->FinalAngle    = theta0;          // position hold while braking
        out->AngleStep     = 0.0f;
        out->SamplingTime  = Ts;
        out->SubStepDuration = A;
        out->SubStep[0] = 1*A; out->SubStep[1] = 2*A; out->SubStep[2] = 3*A;
        out->SubStep[3] = 6*A; out->SubStep[4] = 7*A; out->SubStep[5] = 8*A;
        out->MovementDuration = (3/*accel*/ + 0/*cruise*/ + 3/*decel*/) * A; // = 6A
        out->Jerk = J;
        out->CruiseSpeed = 0.0f;            // we’re braking, no cruise
        out->Theta = theta0; out->Omega = w0; out->Acceleration = 0.0f;
        out->isExecutingTrajectory = true;
        return;
    }

    // 4) Same direction: decide if immediate DECEL or ACCEL/CRUISE/DECEL
    const float need = fabsf(delta);
    const float Ddec = decel_distance_from_speed(fabsf(w0), J_max, A_max, Ts);

    bool skip_accel = false;
    if (need <= Ddec) {
        // DECEL only (truncate accel+cruise)
        skip_accel = true;
    } else if (fabsf(w0) >= omega_cruise) {
        // Already at cruise: keep cruising, then decel
        skip_accel = true;
    } else {
        // We do ACCEL -> (CRUISE?) -> DECEL
        skip_accel = false;
    }

    // 5) Pick cruiseN by previewing distance and adjusting
    int cruiseN = 0;
    float covered = preview_distance(fabsf(w0), J_max, A, Ts, cruiseN, skip_accel);
    while (covered < need && cruiseN < 10000) {
        cruiseN += 1;
        covered = preview_distance(fabsf(w0), J_max, A, Ts, cruiseN, skip_accel);
    }

    // 6) Fill the plan with chosen A, J, cruiseN
    out->SamplingTime = Ts;
    out->StartingAngle = theta0;
    out->FinalAngle    = target;
    out->AngleStep     = delta;
    out->SubStepDuration = A;
    out->SubStep[0] = 1*A; out->SubStep[1] = 2*A; out->SubStep[2] = 3*A;
    // Place cruise length as 3A + cruiseN*A window (we’ll implement via jerk selector later)
    out->SubStep[3] = (3 + cruiseN)*A;         // first decel substep start time
    out->SubStep[4] = (4 + cruiseN)*A;
    out->SubStep[5] = (5 + cruiseN)*A;

    out->MovementDuration = (float)(3 + cruiseN + 3) * A;  // total = (6 + cruiseN)A
    out->Jerk = sgn * J_max;
    out->CruiseSpeed = 0.0f; // not strictly needed; you can compute if you want

    // Seed current state
    out->Theta        = theta0;
    out->Omega        = w0;
    out->Acceleration = 0.0f;
    out->A_MAX = A_max; out->J_MAX = J_max;
    out->isExecutingTrajectory = true;

    // Store a flag somewhere (e.g., out->skipAccel) for your `selectJerk()`:
    out->skipAccel = skip_accel;
    out->cruiseN   = cruiseN;
}


