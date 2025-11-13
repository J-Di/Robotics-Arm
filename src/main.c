// main.c — desktop simulator harness for your S-curve planner + ISR @1kHz
// Build:  gcc -O2 -std=c11 main.c SCurveTrajectory.c planner.c -lm -o sim
// Run:    ./sim
// Output: sim_log.csv (time_s, target, theta, omega, accel, isExec, cruiseN, skipAccel)


#include "SCurveTrajectory.h"
#include "planner.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "SCurveTrajectory.h"
#include "planner.h"  // <- MUST be included before calling Planner_* APIs


// --- CPU/RTOS stubs ----------------------------------------------------------
// Your planner uses these; on desktop they can be no-ops.
#ifndef __disable_irq
#define __disable_irq() do{}while(0)
#endif
#ifndef __enable_irq
#define __enable_irq()  do{}while(0)
#endif

// --- Simulation knobs ---------------------------------------------------------
#define SIM_DT_S        (SAMPLING_TIME)   // 1 kHz tick (from your header)
#define SIM_TOTAL_S     (8.f)            // total simulation time
#define LOG_FILENAME    "sim_log.csv"



// Scenario timeline (seconds) and targets (radians).
// Feel free to tweak these or add more events.
typedef struct { float t_s; float target_rad; } CmdEvent;
static const CmdEvent kEvents[] = {
    { 0.00f,  0.0f               },  // start at 0 rad
    { 0.20f,  (float)(40.0 * M_PI/180.0) },  // +40°
    // { 1.20f,  (float)(10.0 * M_PI/180.0) },  // +10°
    // { 2.00f,  (float)(-20.0 * M_PI/180.0)},  // -20° (opposite direction)
    // { 3.20f,  (float)(60.0 * M_PI/180.0) },  // +60°
    // { 4.50f,  (float)( 0.0)              },  // back to 0°
};

// --- Externals provided by your modules --------------------------------------
// (These are defined in SCurveTrajectory.c and planner.c)
extern volatile PosCtrlHandle SCurveTrajectory;
extern volatile VelocityFilter motorTracker;
extern volatile bool  newSetpointDetected;
extern volatile float positionSetpoint;

extern volatile uint8_t plan_ready;          // planner publishes new plan
extern volatile PosCtrlHandle * plan_active; // ISR consumes this pointer

// --- Helpers -----------------------------------------------------------------
static inline uint32_t sec_to_ticks(float t_s) {
    return (uint32_t)llroundf(t_s / SIM_DT_S);
}

static void apply_command(float target_rad) {
    positionSetpoint = target_rad;
    newSetpointDetected = true;  // planner will pick up in next background tick
}

static void banner(void) {
    printf("\nS-curve planner simulation @ 1 kHz\n");
    printf("  SAMPLING_TIME = %.6f s\n", (double)SIM_DT_S);
    printf("  Total time    = %.2f s\n", (double)SIM_TOTAL_S);
    printf("  Log file      = %s\n\n", LOG_FILENAME);
}

// --- Main --------------------------------------------------------------------
int main(void)
{
    banner();

    // 1) Initialize trajectory state + filter + planner
    STrajectoryInit((PosCtrlHandle*)&SCurveTrajectory, (VelocityFilter*)&motorTracker);
    Planner_Init();

    // 2) Open CSV log
    FILE *log = fopen(LOG_FILENAME, "w");
    if (!log) {
        perror("fopen");
        return 1;
    }
    fprintf(log, "t_s,target_rad,theta,omega,accel,jerk,isExec,cruiseN,skipAccel\n");

    // 3) Prime with first command (if any event at t=0)
    for (size_t i = 0; i < sizeof(kEvents)/sizeof(kEvents[0]); ++i) {
        if (fabsf(kEvents[i].t_s) < 1e-6f) {
            apply_command(kEvents[i].target_rad);
            break;
        }
    }

    // 4) Simulation loop (acts like your 1 kHz timer + medium-frequency task)
    const uint32_t total_ticks = sec_to_ticks(SIM_TOTAL_S);
    uint32_t next_event_idx = 0;

    // Precompute event ticks
    uint32_t event_ticks[sizeof(kEvents)/sizeof(kEvents[0])];
    for (size_t i = 0; i < sizeof(kEvents)/sizeof(kEvents[0]); ++i) {
        event_ticks[i] = sec_to_ticks(kEvents[i].t_s);
    }


    for (uint32_t tick = 0; tick <= total_ticks; ++tick) {
        float t_s = tick * SIM_DT_S;

        // a) Inject command events at their scheduled ticks
        if (next_event_idx < sizeof(kEvents)/sizeof(kEvents[0]) &&
            tick == event_ticks[next_event_idx]) {
            apply_command(kEvents[next_event_idx].target_rad);
            next_event_idx++;
        }

        // b) Background planner (your main loop task)
        Planner_BackgroundTask();

        // c) "ISR": run the 1 kHz control tick
        PosCtrl_ISRStep();

        // d) Log current state to CSV (acts like MC_ProgramPositionCommandMotor1)
        volatile PosCtrlHandle *P = plan_active; // snapshot pointer once
        fprintf(log, "%.6f,%.7f,%.7f,%.7f,%.7f,%.7f,%d,%d,%d\n",
                (double)t_s,
                (double)positionSetpoint,
                (double)P->Theta,
                (double)motorTracker.omega,
                (double)motorTracker.accel,
                P->Jerk,
                P->isExecutingTrajectory ? 1 : 0,
                P->cruiseN,
                P->skipAccel ? 1 : 0);
    }

    fclose(log);

    printf("Done. Open '%s' in your favorite plotter (Excel, Python, etc.).\n", LOG_FILENAME);
    printf("Suggested quick check: plot theta vs time, overlay target.\n");
    return 0;
}
