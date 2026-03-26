#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "planner.h"
#include "SCurveTrajectory.h"

// Define a path for the tests

/* *
   These externs are declared in SCurveTrajectory.h — main.c owns them.
   * */
volatile float positionSetpoint   = 0.0f;
volatile bool  newSetpointDetected = false;

/* *
   Setpoint event: "at simulation time T, inject setpoint S"
   * */
#define MAX_EVENTS 32

typedef struct {
    float time_s;       // when to inject (seconds from sim start)
    float setpoint;     // position setpoint in radians
} SetpointEvent;

/* *
   CSV logger — writes one row per ISR tick
   * */
static FILE *logFile = NULL;

static void logHeader(void){
    fprintf(logFile, "time_s,position,velocity,acceleration,jerk,phase,setpoint,dir\n");
}

static void logRow(float time_s, float setpoint){
    VelocityFilter *tr = (VelocityFilter *)motorTracker;
    PosCtrlHandle  *pl = (PosCtrlHandle *)paths_planned[active_plan];

    fprintf(logFile, "%.6f,%.8f,%.8f,%.8f,%.8f,%d,%.8f,%d\n",
            time_s,
            tr->theta,
            tr->omega,
            tr->accel,
            tr->jerk,
            pl->profilePhase,
            setpoint,
            pl->dir);
}

/* *
   Run one simulation scenario
   * */
static void runScenario(const char *name,  SetpointEvent *events, int numEvents, float duration_s, const char *csvPath){

    printf("\n Scenario: %s n", name);
    printf("  Duration: %.2f s  |  Events: %d  |  Output: %s\n", duration_s, numEvents, csvPath);

    // Set a reasonable velocity limit BEFORE plannerInit (which reads it)
    // 1.0 rad/s ≈ 57 deg/s — good for demonstration
    setMaxVelocity(1.0f);

    // (Re)init the planner for a fresh run
    // Since plannerInit mallocs, and we may run multiple scenarios,
    // we just re-init (leaks the old allocs, but fine for a sim tool).
    if (plannerInit() != 0){
        fprintf(stderr, "  ERROR: plannerInit failed\n");
        return;
    }

    logFile = fopen(csvPath, "w");
    if (!logFile){
        fprintf(stderr, "  ERROR: cannot open %s\n", csvPath);
        return;
    }
    logHeader();

    int totalTicks = (int)(duration_s / SAMPLING_TIME);
    int nextEvent  = 0;
    float currentSetpoint = 0.0f;

    for (int tick = 0; tick < totalTicks; tick++){
        float t = tick * SAMPLING_TIME;

        // Check if it's time to inject the next setpoint
        while (nextEvent < numEvents && t >= events[nextEvent].time_s){
            currentSetpoint = events[nextEvent].setpoint;
            printf("  t=%.3f s -> setpoint = %.4f rad (%.2f deg)\n",
                   t, currentSetpoint, radToDegrees(currentSetpoint));
            buildNewCurve(currentSetpoint);
            nextEvent++;
        }

        // Run one ISR tick
        PosCtrl_ISRStep();

        // Log state
        logRow(t, currentSetpoint);
    }

    fclose(logFile);
    logFile = NULL;
    printf("  Done. %d ticks logged.\n", totalTicks);
}

/* *
   Built-in test scenarios
   * */

// Scenario 1: Simple point-to-point from rest
static void scenario_simple(void){
    SetpointEvent events[] = {
        { 0.01f, degreesToRad(45.0f) }
    };
    runScenario("Simple 0 -> 45 deg",
                events, 1, 3.0f, "sim_simple.csv");
}

// Scenario 2: Mid-motion setpoint change — same direction, farther
static void scenario_extend(void){
    SetpointEvent events[] = {
        { 0.01f, degreesToRad(30.0f) },
        { 0.80f, degreesToRad(60.0f) },
    };
    runScenario("Extend: 30 deg -> 60 deg mid-motion",
                events, 2, 4.0f, "sim_extend.csv");
}

// Scenario 3: Mid-motion setpoint change — same direction, closer
static void scenario_shorten(void){
    SetpointEvent events[] = {
        { 0.01f, degreesToRad(60.0f) },
        { 0.80f, degreesToRad(25.0f) },
    };
    runScenario("Shorten: 60 deg -> 25 deg mid-motion",
                events, 2, 4.0f, "sim_shorten.csv");
}

// Scenario 4: Reverse direction mid-motion (wandering case)
static void scenario_reverse(void){
    SetpointEvent events[] = {
        { 0.01f, degreesToRad(45.0f) },
        { 0.80f, degreesToRad(-20.0f) },
    };
    runScenario("Reverse: 45 deg -> -20 deg mid-motion",
                events, 2, 5.0f, "sim_reverse.csv");
}

// Scenario 5: Very short move (case d)
static void scenario_short(void){
    SetpointEvent events[] = {
        { 0.01f, degreesToRad(0.5f) }
    };
    runScenario("Short move: 0 -> 0.5 deg",
                events, 1, 2.0f, "sim_short.csv");
}

// Scenario 6: Multiple rapid setpoint changes
static void scenario_rapid(void){
    SetpointEvent events[] = {
        { 0.01f, degreesToRad(30.0f) },
        { 0.50f, degreesToRad(50.0f) },
        { 1.00f, degreesToRad(20.0f) },
        { 1.80f, degreesToRad(60.0f) },
    };
    runScenario("Rapid changes: 30 -> 50 -> 20 -> 60 deg",
                events, 4, 5.0f, "sim_rapid.csv");
}

/* *
   Interactive mode: user types setpoints at runtime
   * */
static void interactiveMode(void){
    printf("\n=== Interactive Mode ===\n");
    printf("Enter setpoint events. Each line: <time_seconds> <position_degrees>\n");
    printf("Example: 0.01 45.0   (at 10ms, command 45 degrees)\n");
    printf("Blank line = done entering events.\n\n");

    SetpointEvent events[MAX_EVENTS];
    int count = 0;
    char line[128];

    while (count < MAX_EVENTS){
        printf("  Event %d: ", count + 1);
        if (!fgets(line, sizeof(line), stdin)) break;
        if (line[0] == '\n' || line[0] == '\r') break;

        float t, deg;
        if (sscanf(line, "%f %f", &t, &deg) == 2){
            events[count].time_s   = t;
            events[count].setpoint = degreesToRad(deg);
            count++;
        } else {
            printf("    (could not parse — use: <time> <degrees>)\n");
        }
    }

    if (count == 0){
        printf("No events entered.\n");
        return;
    }

    float duration = 5.0f;
    printf("\nSim duration in seconds [5.0]: ");
    if (fgets(line, sizeof(line), stdin) && line[0] != '\n'){
        sscanf(line, "%f", &duration);
    }

    char filename[128] = "/test/sim_interactive.csv";
    printf("Output CSV filename [sim_interactive.csv]: ");
    if (fgets(line, sizeof(line), stdin) && line[0] != '\n'){
        sscanf(line, "%127s", filename);
    }

    runScenario("Interactive", events, count, duration, filename);
}

/* *
   Main menu
   * */
int main(void){
    printf("================================================\n");
    printf("  S-Curve Trajectory Planner — Simulation Tool\n");
    printf("================================================\n");
    printf("\nConstraints:\n");
    printf("  A_MAX = %.1f rad/s^2\n", (float)A_MAX);
    printf("  J_MAX = %.1f rad/s^3\n", (float)J_MAX);
    printf("  V_MAX = 1.0000 rad/s  (~57 deg/s, set before each scenario)\n");
    printf("  Ts    = %.4f s  (%.0f Hz)\n\n", SAMPLING_TIME, 1.0f / SAMPLING_TIME);

    printf("Scenarios:\n");
    printf("  1) Simple:  0 -> 45 deg from rest\n");
    printf("  2) Extend:  30 -> 60 deg mid-motion\n");
    printf("  3) Shorten: 60 -> 25 deg mid-motion (must decel)\n");
    printf("  4) Reverse: 45 -> -20 deg mid-motion (wandering)\n");
    printf("  5) Short:   0 -> 0.5 deg (case d)\n");
    printf("  6) Rapid:   30 -> 50 -> 20 -> 60 deg\n");
    printf("  7) Interactive: build your own\n");
    printf("  0) Run ALL built-in (1-6)\n");
    printf("\nChoice: ");

    char line[64];
    if (!fgets(line, sizeof(line), stdin)){
        return 1;
    }

    int choice = atoi(line);

    switch (choice){
        case 1: scenario_simple();  break;
        case 2: scenario_extend();  break;
        case 3: scenario_shorten(); break;
        case 4: scenario_reverse(); break;
        case 5: scenario_short();   break;
        case 6: scenario_rapid();   break;
        case 7: interactiveMode();  break;
        case 0:
            scenario_simple();
            scenario_extend();
            scenario_shorten();
            scenario_reverse();
            scenario_short();
            scenario_rapid();
            break;
        default:
            printf("Invalid choice.\n");
            return 1;
    }

    printf("\nDone. Run: python3 plot_trajectory.py <csv_file>\n");
    return 0;
}