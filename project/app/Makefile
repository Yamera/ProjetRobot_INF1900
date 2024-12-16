# Target microcontroller
MCU = atmega324pa

# Project name
PROJECTNAME = appWall_e

# Source directories
SRC_DIR = .
IMPLS_DIR = $(SRC_DIR)/impls
DEF_DIR = $(SRC_DIR)/defs
MODE_IMPLS_DIR = $(IMPLS_DIR)/mode
MODE_DEF_DIR = $(DEF_DIR)/mode

# Source files
PRJSRC = $(SRC_DIR)/main.cpp $(wildcard $(IMPLS_DIR)/*.cpp) $(wildcard $(MODE_IMPLS_DIR)/*.cpp)

# Additional includes
INC_DIR = ../lib/include
INC = -I"$(INC_DIR)" -I"$(DEF_DIR)" -I"$(MODE_DEF_DIR)"

# Library directory and target library
LIB_DIR = ../lib
LIBRARY = $(LIB_DIR)/wall_e.a
LIBS = $(wildcard $(LIBRARY))

# CPU frequency
CPU_FREQUENCY = 8000000UL

# Optimization level
OPTLEVEL = s

# Programmer ID
AVRDUDE_PROGRAMMERID = usbasp

# Compiler and tools
CC = avr-gcc
OBJCOPY = avr-objcopy
AVRDUDE = avrdude
REMOVE = rm -f
HEXFORMAT = ihex

# Compilation flags
CFLAGS = -I. -I/usr/include/simavr -MMD $(INC) -g -mmcu=$(MCU) -O$(OPTLEVEL) \
    -std=c++14 -fshort-enums \
    -funsigned-bitfields -funsigned-char \
    -Wall -fno-threadsafe-statics -DF_CPU=$(CPU_FREQUENCY) -fno-rtti

# C++ compiler flags
CXXFLAGS = -fno-exceptions

# Linker flags
LDFLAGS = -Wl,-Map,$(OBJ_DIR)/$(PROJECTNAME).map -mmcu=$(MCU)

# Default targets
OBJ_DIR = obj
TRG = $(OBJ_DIR)/$(PROJECTNAME).elf
HEXROMTRG = $(OBJ_DIR)/$(PROJECTNAME).hex
HEXTRG = $(HEXROMTRG) $(OBJ_DIR)/$(PROJECTNAME).ee.hex

# Object files
OBJDEPS = $(PRJSRC:%.cpp=$(OBJ_DIR)/%.o)

# Phony targets
.PHONY: all install clean

# Create obj directory and subdirectories if they don't exist
$(shell mkdir -p $(OBJ_DIR)/impls $(OBJ_DIR)/impls/mode)

# Build all targets
all: $(LIBRARY) $(TRG) $(HEXROMTRG)

# Check and build the library if needed
$(LIBRARY):
	$(MAKE) -C $(LIB_DIR)

# Link target
$(TRG): $(OBJDEPS)
	$(CC) $(LDFLAGS) -o $@ $(OBJDEPS) -lm $(LIBS)

# Compile C++ source files to object files, ensuring output directory structure
$(OBJ_DIR)/%.o: %.cpp
	$(CC) $(CFLAGS) $(CXXFLAGS) -c $< -o $@

-include $(OBJDEPS:.o=.d)

# Generate hex files
$(OBJ_DIR)/%.hex: $(OBJ_DIR)/%.elf
	$(OBJCOPY) -j .text -j .data -O $(HEXFORMAT) $< $@

# Install to the microcontroller
install: all $(HEXROMTRG)
	$(AVRDUDE) -c $(AVRDUDE_PROGRAMMERID) -p $(MCU) -P usb -e -U flash:w:$(HEXROMTRG)

# Clean build files
clean:
	$(REMOVE) $(TRG) $(OBJ_DIR)/$(PROJECTNAME).map $(OBJDEPS) $(HEXTRG)
	rm -rf $(OBJ_DIR)
	$(MAKE) -C $(LIB_DIR) clean
