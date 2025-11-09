#include <string.h>
#include "planner.h"
#include "SCurveTrajectory.h"


// plan_active always points to one of these two.
static PosCtrlHandle plan_bank[2];
static uint8_t       bank_idx = 0;         // 0 or 1
PosCtrlHandle       *plan_active = &plan_bank[0];
volatile uint8_t     plan_ready  = 0;      // ISR will clear after it acknowledges

// Internal flags/knobs
static volatile uint8_t replan_request = 0;   // set by Planner_RequestReplan()
static float omega_cruise = 0.08727f;             // default cruise speed [rad/s], adjust to taste
static float A_max = 5.0f;                    // [rad/s^2] default; override via Planner_SetLimits
static float J_max = 500.0f;                  // [rad/s^3] default; override via Planner_SetLimits

// ---------- External shared state (declared in SCurveTrajectory.c) ----------
extern volatile PosCtrlHandle SCurveTrajectory;   // active live state (θ/ω/a updated in ISR)
extern volatile VelocityFilter motorTracker;      // provides ω, accel estimates
extern volatile float positionSetpoint;           // latest target angle [rad]
extern volatile bool  newSetpointDetected;        // set by CAN handler

// ---------- Small helpers ----------

static inline int8_t sign_db(float x, float band) {
    return (x > band) - (x < -band);
}

// Build a complete S-curve plan from the snapshot (θ0, ω0) to target.
// Uses your existing computeTrajectoryParameters() and time estimator.
static void build_plan(PosCtrlHandle *out,
                       float theta0, float omega0, float accel0,
                       float target, float Ts)
{
    memset(out, 0, sizeof(*out));

    out->SamplingTime       = Ts;
    out->StartingAngle      = theta0;
    out->FinalAngle         = target;
    out->AngleStep          = target - theta0;

    // Choose a duration. Simple first pass: use your estimator (ms) then convert to s.
    const float T_ms = getEstimatedTrajectoryTime(theta0, target, omega_cruise);
    const float T_s  = (T_ms <= 1.0f) ? 0.009f : (T_ms * 0.001f); // minimum 9*Ts guard

    // Compute 9-segment parameters (fills MovementDuration, SubStepDuration, SubStep[], Jerk, CruiseSpeed, etc.)
    computeTrajectoryParameters(out, theta0, T_s, out->AngleStep);

    // Seed current state from snapshot
    out->Theta        = theta0;
    out->Omega        = omega0;
    out->Acceleration = accel0;

    // Physical/comfort limits (optional: clamp computed jerk/accel to these)
    out->A_MAX  = A_max;
    out->J_MAX  = J_max;

    // Guidance tolerances and sign
    out->posTol = (out->posTol > 0.0f) ? out->posTol : 0.002f;  // ~0.11°
    out->velTol = (out->velTol > 0.0f) ? out->velTol : 0.02f;   // rad/s
    out->remainingDistance = out->AngleStep;
    out->sign     = sign_db(out->remainingDistance, out->posTol);
    out->sign_prev= out->sign;

    out->isExecutingTrajectory = true;
}

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

// ---------- Public API ----------

void Planner_Init(void)
{
    // Initialize the two banks to a benign hold state
    memset(&plan_bank[0], 0, sizeof(plan_bank));
    plan_bank[0].SamplingTime  = (SCurveTrajectory.SamplingTime > 0.0f) ? SCurveTrajectory.SamplingTime : SAMPLING_TIME;
    plan_bank[0].FinalAngle    = plan_bank[0].StartingAngle;
    plan_bank[0].posTol        = 0.002f;
    plan_bank[0].velTol        = 0.02f;
    plan_bank[0].isExecutingTrajectory = false;

    plan_bank[1] = plan_bank[0];
    plan_active  = &plan_bank[0];
    bank_idx     = 0;
    plan_ready   = 0;
    replan_request = 0;

    // Default knobs (can be overridden later)
    omega_cruise = (SCurveTrajectory.CruiseSpeed > 0.0f) ? SCurveTrajectory.CruiseSpeed : 0.5f;
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
    // Consume flags (keep them short-lived)
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
    const float accel0 = motorTracker.accel;         // filtered accel
    const float target = positionSetpoint;

    // Decide strategy (simple version): always re-build a full plan from current state to target.
    // If you want ACCEL/CRUISE/DECEL truncation or BRAKE-then-relaunch for opposite direction,
    // compute that logic here and adjust the duration you pass into computeTrajectoryParameters().
    PosCtrlHandle pending;
    build_plan(&pending, theta0, omega0, accel0, target, Ts);

    // Atomically publish for the ISR
    publish_plan(&pending);
}
