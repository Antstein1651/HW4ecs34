# Compiler
CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17 -g -Iinclude

# Directories
SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

# Target executable
TARGET = $(BIN_DIR)/transportation_planner

# Source files
SRCS = $(wildcard $(SRC_DIR)/*.cpp)

# Object files
OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))

# Header dependencies
DEPS = $(wildcard include/*.h)

# Default target
all: $(TARGET)

# Create necessary directories
$(OBJ_DIR) $(BIN_DIR):
	mkdir -p $@

# Rule to build the final executable
$(TARGET): $(OBJS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Rule to compile object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp $(DEPS) | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Run tests (modify this based on your test framework)
test: $(TARGET)
	./run_tests.sh  # Change this to the actual test command

# Clean build files
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

# Phony targets
.PHONY: all clean test