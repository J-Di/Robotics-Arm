#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "SCurveTrajectory.h"
#include "velocity_ctrl.h"

/* Globals required by SCurveTrajectory.h externs */
volatile float positionSetpoint   = 0.0f;
volatile bool  newSetpointDetected = false;

/* Velocity demand event */
typedef struct {
    float time_s;
    float velocity;   /* rad/s */
} VelEvent;

static void runVelScenario(const char *name, VelEvent *events, int numEvents,
                           float duration_s, const char *csvPath,
                           float posMin, float posMax, bool useLimits){

    printf("\n  Scenario: %s\n", name);
    printf("  Duration: %.2f s  |  Events: %d  |  Output: %s\n",
           duration_s, numEvents, csvPath);

    VelocityFilter *tracker = velocityFilterInit();
    VelCtrlHandle  *vc      = velCtrlInit(0.0f);
    if (!tracker || !vc){
        fprintf(stderr, "  ERROR: init failed\n");
        return;
    }

    /* Override v_max to match planner tests */
    vc->v_max = 1.0f;

    if (useLimits){
        velCtrlSetPositionLimits(vc, posMin, posMax);
        printf("  Position limits: [%.1f, %.1f] deg\n",
               posMin * 180.0f / (float)M_PI, posMax * 180.0f / (float)M_PI);
    }

    velCtrlStart(vc, tracker);

    FILE *f = fopen(csvPath, "w");
    if (!f){ fprintf(stderr, "  ERROR: cannot open %s\n", csvPath); return; }

    fprintf(f, "time_s,position,velocity,acceleration,jerk,demand,cmd_vel\n");

    int totalTicks = (int)(duration_s / SAMPLING_TIME);
    int nextEvent  = 0;

    for (int tick = 0; tick < totalTicks; tick++){
        float t = tick * SAMPLING_TIME;

        /* Inject velocity demands */
        while (nextEvent < numEvents && t >= events[nextEvent].time_s){
            float vel = events[nextEvent].velocity;
            printf("  t=%.3f s -> demand = %.4f rad/s (%.2f deg/s)\n",
                   t, vel, vel * 180.0f / (float)M_PI);
            velCtrlSetDemand(vc, vel);
            nextEvent++;
        }

        /* Run one ISR tick */
        velCtrl_ISRStep(vc, tracker);

        /* Log */
        fprintf(f, "%.6f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
                t, tracker->theta, tracker->omega, tracker->accel,
                tracker->jerk, vc->demanded_vel, vc->cmd_vel);
    }

    fclose(f);
    printf("  Final: pos=%.4f deg  vel=%.6f rad/s\n",
           tracker->theta * 180.0f / (float)M_PI, tracker->omega);
    printf("  Done. %d ticks logged.\n", totalTicks);

    free(tracker);
    free(vc);
}

int main(void){
    printf("==============================================\n");
    printf("  Velocity Control Module — Test Harness\n");
    printf("==============================================\n");
    printf("\nConstraints:\n");
    printf("  A_MAX = %.1f rad/s^2\n", (float)A_MAX);
    printf("  J_MAX = %.1f rad/s^3\n", (float)J_MAX);
    printf("  V_MAX = 1.0000 rad/s  (~57 deg/s)\n");
    printf("  Ts    = %.4f s  (%.0f Hz)\n\n", SAMPLING_TIME, 1.0f / SAMPLING_TIME);

    /* Scenario 1: Step demand — 0 to full speed, hold, then stop */
    {
        VelEvent events[] = {
            { 0.01f,  0.5f },     /* ramp to 0.5 rad/s */
            { 1.50f,  0.0f },     /* stop */
        };
        runVelScenario("Step: 0 → 0.5 rad/s → stop",
                       events, 2, 3.0f, "test/vel_step.csv",
                       0, 0, false);
    }

    /* Scenario 2: Direction reversal */
    {
        VelEvent events[] = {
            { 0.01f,  0.8f },     /* forward */
            { 1.00f, -0.8f },     /* reverse */
            { 2.00f,  0.0f },     /* stop */
        };
        runVelScenario("Reversal: +0.8 → -0.8 → stop",
                       events, 3, 3.5f, "test/vel_reverse.csv",
                       0, 0, false);
    }

    /* Scenario 3: Gradual speed changes (simulating joystick ramp) */
    {
        VelEvent events[] = {
            { 0.01f,  0.2f },
            { 0.30f,  0.5f },
            { 0.60f,  0.8f },
            { 0.90f,  1.0f },     /* full speed */
            { 1.50f,  0.5f },     /* ease off */
            { 2.00f,  0.0f },     /* stop */
        };
        runVelScenario("Gradual: ramp up then ease off",
                       events, 6, 3.5f, "test/vel_gradual.csv",
                       0, 0, false);
    }

    /* Scenario 4: Rapid direction changes */
    {
        VelEvent events[] = {
            { 0.01f,  0.6f },
            { 0.40f, -0.4f },
            { 0.80f,  0.8f },
            { 1.20f, -0.6f },
            { 1.60f,  0.0f },
        };
        runVelScenario("Rapid: quick direction changes",
                       events, 5, 3.0f, "test/vel_rapid.csv",
                       0, 0, false);
    }

    /* Scenario 5: Position limits — move toward a wall */
    {
        float limit = 45.0f * (float)M_PI / 180.0f;   /* 45 degrees */
        VelEvent events[] = {
            { 0.01f,  1.0f },     /* full speed toward +45° limit */
            { 3.00f,  0.0f },     /* (should have auto-stopped before this) */
        };
        runVelScenario("Limit: full speed toward 45° wall",
                       events, 2, 4.0f, "test/vel_limit.csv",
                       -limit, limit, true);
    }

    printf("\nDone. CSV files in test/\n");
    return 0;
}


// to run
// gcc -Wall -Wextra -o vel_test vel_test.c velocity_ctrl.c SCurveTrajectory.c -lm
//./vel_test