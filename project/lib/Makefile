# Target microcontroller
MCU = atmega324pa

# Project name
PROJECTNAME = wall_e

# Root directory
ROOT_DIR = $(CURDIR)

# Source and include directories
SRC_DIR = $(ROOT_DIR)/src
INC_DIR = $(ROOT_DIR)/include
BUILD_DIR = $(ROOT_DIR)/build
OBJ_DIR = $(BUILD_DIR)/obj
DEP_DIR = $(BUILD_DIR)/dep

# CPU frequency
CPU_FREQUENCY = 8000000UL

# Include directories (find all subdirectories in include directory)
INC = $(shell find $(INC_DIR) -type d | sed 's/^/-I /')

# Optimization level
OPTLEVEL = s

# Compiler and tools
CC = avr-gcc
REMOVE = rm -f
REMOVE_DIR = rm -rf 
AR = avr-ar

# Precompiled header files
PCH_HEADER = $(INC_DIR)/pch.h
PCH_OBJECT = $(PCH).gch

# Compiler flags
CFLAGS = -MMD -MP $(INC) -g -mmcu=$(MCU) -O$(OPTLEVEL) \
    -std=c++14 -fpack-struct -fshort-enums \
    -funsigned-bitfields -funsigned-char \
    -Wall -fno-threadsafe-statics -DF_CPU=$(CPU_FREQUENCY)

# C++ compiler flags
CXXFLAGS = -fno-exceptions

# Target library file
LIBRARY = $(ROOT_DIR)/$(PROJECTNAME).a

# Source files (find all .cpp files in src directory and subdirectories)
CPPFILES = $(shell find $(SRC_DIR) -type f -name '*.cpp')

# Object files in the obj directory and dependency files in the dep directory
OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(CPPFILES))
DEP_FILES = $(patsubst $(SRC_DIR)/%.cpp, $(DEP_DIR)/%.d, $(CPPFILES))

# Default target to build the library
$(LIBRARY): $(PCH_OBJECT) $(OBJ_FILES)
	$(AR) crs $@ $^

# Compile the PCH (create necessary directories)
$(PCH_OBJECT): $(PCH_HEADER)
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -x c++-header $(PCH_HEADER) -o $@

# Compile C++ source files to object files (use the precompiled header)
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	mkdir -p $(dir $@) $(DEP_DIR)/$(dir $*)
	$(CC) $(CFLAGS) $(CXXFLAGS) -include $(PCH_HEADER) -c $< -o $@ -MF $(DEP_DIR)/$*.d

# Include dependency files
-include $(DEP_FILES)

# Phony targets
.PHONY: clean debug

# Clean up build files and library
clean:
	$(REMOVE) $(LIBRARY)
	$(REMOVE_DIR) $(BUILD_DIR)

# Display variables for debugging
debug: CFLAGS += -DDEBUG
debug: $(LIBRARY)
	@echo "obj: " $(OBJ_FILES)
	@echo "src: " $(CPPFILES)
