# Minimal Makefile: src/*.c, headers in lib/, output in build/
APP        := app
SRC_DIR    := src
INC_DIR    := lib
BUILD_DIR  := build
OBJ_DIR    := $(BUILD_DIR)/obj

CC         := gcc
STD        := -std=c17
WARN       := -Wall -Wextra -Wpedantic
INCLUDES   := -I$(INC_DIR)

# Find all C sources (nested dirs too)
SRCS       := $(shell find $(SRC_DIR) -type f -name '*.c')
OBJS       := $(SRCS:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

# Flags
CFLAGS_RELEASE := $(STD) $(WARN) $(INCLUDES) -O2 -g
CFLAGS_DEBUG   := $(STD) $(WARN) $(INCLUDES) -O0 -g3
LDFLAGS        :=
LDLIBS         := -lm       # <-- math library for floorf/roundf/llroundf

.PHONY: all debug clean run

all: CFLAGS := $(CFLAGS_RELEASE)
all: $(BUILD_DIR)/$(APP)

debug: CFLAGS := $(CFLAGS_DEBUG)
debug: $(BUILD_DIR)/$(APP)

$(BUILD_DIR)/$(APP): $(OBJS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

# Build objects mirroring src/ tree
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

run: all
	./$(BUILD_DIR)/$(APP)

clean:
	rm -rf $(BUILD_DIR)
