# Compiler and Flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -g
LDFLAGS = -lgtest -lgtest_main -pthread

# Directories
SRC_DIR = src
INCLUDE_DIR = include
TEST_DIR = testsrc
BUILD_DIR = build
BIN_DIR = bin

# Source Files
SRC_FILES = $(wildcard $(SRC_DIR)/*.cpp)
INCLUDE_FILES = $(wildcard $(INCLUDE_DIR)/*.h)
TEST_FILES = $(wildcard $(TEST_DIR)/*.cpp)

# Object Files
OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(SRC_FILES))
TEST_OBJ_FILES = $(patsubst $(TEST_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(TEST_FILES))

# Executables
TEST_EXEC = $(BIN_DIR)/run_tests

# Create Directories if not exist
$(shell mkdir -p $(BUILD_DIR) $(BIN_DIR))

# Compile Object Files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp $(INCLUDE_FILES)
	$(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -c $< -o $@

$(BUILD_DIR)/%.o: $(TEST_DIR)/%.cpp $(INCLUDE_FILES)
	$(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -c $< -o $@

# Link Executable for Tests
$(TEST_EXEC): $(OBJ_FILES) $(TEST_OBJ_FILES)
	$(CXX) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

# Run Tests
test: $(TEST_EXEC)
	./$(TEST_EXEC)

# Clean Build Files
clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR)
