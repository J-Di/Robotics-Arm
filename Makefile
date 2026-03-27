# S-Curve Trajectory Planner — Simulation Makefile
CC      = gcc
CFLAGS  = -Wall -Wextra -g -Ilib
LDFLAGS = -lm

SRC_DIR   = src
BUILD_DIR = build
TEST_DIR  = test
TARGET    = sim
VEL_TARGET = vel_test

SRCS = $(SRC_DIR)/main.c \
       $(SRC_DIR)/SCurveTrajectory.c \
       $(SRC_DIR)/planner.c

OBJS = $(BUILD_DIR)/main.o \
       $(BUILD_DIR)/SCurveTrajectory.o \
       $(BUILD_DIR)/planner.o

VEL_SRCS = $(SRC_DIR)/vel_test.c \
           $(SRC_DIR)/velocity_ctrl.c \
           $(SRC_DIR)/SCurveTrajectory.c

VEL_OBJS = $(BUILD_DIR)/vel_test.o \
           $(BUILD_DIR)/velocity_ctrl.o \
           $(BUILD_DIR)/SCurveTrajectory_vel.o

# Default: build both simulators
all: $(BUILD_DIR) $(TARGET) $(VEL_TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# --- S-Curve position planner ---

$(TARGET): $(OBJS)
	@echo "  [LINK]  $(TARGET)"
	$(CC) $(OBJS) -o $(TARGET) $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@echo "  [CC]    $<"
	$(CC) $(CFLAGS) -c $< -o $@

# --- Velocity control test ---

$(VEL_TARGET): $(VEL_OBJS)
	@echo "  [LINK]  $(VEL_TARGET)"
	$(CC) $(VEL_OBJS) -o $(VEL_TARGET) $(LDFLAGS)

# SCurveTrajectory is shared between both targets but compiled into
# separate object files to avoid collisions in the build directory.
$(BUILD_DIR)/SCurveTrajectory_vel.o: $(SRC_DIR)/SCurveTrajectory.c
	@echo "  [CC]    $< (velocity)"
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(TARGET) $(VEL_TARGET) \
		sim_*.csv sim_*.png \
		$(TEST_DIR)/sim_*.csv $(TEST_DIR)/sim_*.png \
		$(TEST_DIR)/vel_*.csv $(TEST_DIR)/vel_*.png

# Run all built-in S-curve scenarios
run: $(TARGET)
	./$(TARGET) <<< "0"

# Run velocity control tests
run-vel: $(VEL_TARGET)
	./$(VEL_TARGET)

# Plot all generated CSVs
plot:
	python3 plot_trajectory.py $(TEST_DIR)/sim_*.csv

.PHONY: all clean run run-vel plot