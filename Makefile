# === Project Settings =========================================================
TARGET := sim

# Directory layout
SRCDIR := src
INCDIR := lib
OBJDIR := build

# Automatically find all .c files in /src
SRC := $(wildcard $(SRCDIR)/*.c)
OBJ := $(patsubst $(SRCDIR)/%.c,$(OBJDIR)/%.o,$(SRC))

# Compiler / flags
CC      := gcc
CFLAGS  := -std=c11 -O2 -Wall -Wextra -I$(INCDIR)
LDFLAGS := -lm

# === Rules ====================================================================

.PHONY: all clean run

all: $(TARGET)

# Link
$(TARGET): $(OBJ)
	@echo "  [LINK]  $@"
	$(CC) $(OBJ) -o $@ $(LDFLAGS)

# Compile each .c file
$(OBJDIR)/%.o: $(SRCDIR)/%.c | $(OBJDIR)
	@echo "  [CC]    $<"
	$(CC) $(CFLAGS) -c $< -o $@

# Ensure build folder exists
$(OBJDIR):
	@mkdir -p $(OBJDIR)

# Clean build outputs
clean:
	@echo "  [CLEAN]"
	rm -rf $(OBJDIR) $(TARGET)

# Run the simulation (after build)
run: all
	./$(TARGET)

# Convenience
print-%:
	@echo '$*=$($*)'
