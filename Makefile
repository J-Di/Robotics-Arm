# S-Curve Trajectory Planner — Simulation Makefile
CC      = gcc
CFLAGS  = -Wall -Wextra -g -Ilib
LDFLAGS = -lm

SRC_DIR   = src
BUILD_DIR = build
TARGET    = sim

SRCS = $(SRC_DIR)/main.c \
       $(SRC_DIR)/SCurveTrajectory.c \
       $(SRC_DIR)/planner.c

OBJS = $(BUILD_DIR)/main.o \
       $(BUILD_DIR)/SCurveTrajectory.o \
       $(BUILD_DIR)/planner.o

# Default: build the simulator
all: $(BUILD_DIR) $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): $(OBJS)
	@echo "  [LINK]  $(TARGET)"
	$(CC) $(OBJS) -o $(TARGET) $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@echo "  [CC]    $<"
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(TARGET) sim_*.csv sim_*.png

# Run all built-in scenarios
run: $(TARGET)
	./$(TARGET) <<< "0"

# Plot all generated CSVs
plot:
	python3 plot_trajectory.py sim_*.csv

.PHONY: all clean run plot
