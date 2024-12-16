# Target microcontroller
MCU=atmega324pa

# Project name
PROJECTNAME=robot_tests

# Source files
SRC_DIR=.
PRJSRC=$(SRC_DIR)/button_test.cpp

# Additional includes
INC_DIR=../lib/include
INC=-I"$(INC_DIR)"

# Library directory and target library
LIB_DIR=../lib
LIBRARY=$(LIB_DIR)/wall_e.a
LIBS=$(wildcard $(LIBRARY))

# CPU frequency
CPU_FREQUENCY=8000000UL

# Optimization level
OPTLEVEL=s

# Programmer ID
AVRDUDE_PROGRAMMERID=usbasp

# Compiler and tools
CC=avr-gcc
OBJCOPY=avr-objcopy
AVRDUDE=avrdude
REMOVE=rm -f
HEXFORMAT=ihex

# Compilation flags
CFLAGS=-I. -I/usr/include/simavr -MMD $(INC) -g -mmcu=$(MCU) -O$(OPTLEVEL) \
	-std=c++14 -fpack-struct -fshort-enums \
	-funsigned-bitfields -funsigned-char \
	-Wall -fno-threadsafe-statics -DF_CPU=$(CPU_FREQUENCY)

# C++ compiler flags
CXXFLAGS=-fno-exceptions

# Linker flags
LDFLAGS=-Wl,-Map,$(OBJ_DIR)/$(PROJECTNAME).map -mmcu=$(MCU)

# Default targets
OBJ_DIR=obj
TRG=$(OBJ_DIR)/$(PROJECTNAME).elf
HEXROMTRG=$(OBJ_DIR)/$(PROJECTNAME).hex
HEXTRG=$(HEXROMTRG) $(OBJ_DIR)/$(PROJECTNAME).ee.hex

# Object files
CFILES=$(filter %.c, $(PRJSRC))
CPPFILES=$(filter %.cpp, $(PRJSRC))
OBJDEPS=$(addprefix $(OBJ_DIR)/,$(CFILES:.c=.o) $(CPPFILES:.cpp=.o))

# Phony targets
.PHONY: all install clean

# Create obj directory if it doesn't exist
$(shell mkdir -p $(OBJ_DIR))

# Build all targets
all: $(LIBRARY) $(TRG) $(HEXROMTRG)

# Check and build the library if needed
$(LIBRARY):
	# @echo "Building library..."
	$(MAKE) -C $(LIB_DIR)

# Link target
$(TRG): $(OBJDEPS)
	# @echo "Linking..."
	$(CC) $(LDFLAGS) -o $(TRG) $(OBJDEPS) -lm $(LIBS)

# Compile C and C++ source files to object files
$(OBJ_DIR)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
$(OBJ_DIR)/%.o: %.cpp
	$(CC) $(CFLAGS) $(CXXFLAGS) -c $< -o $@

-include $(OBJ_DIR)/*.d

# Generate hex files
$(OBJ_DIR)/%.hex: $(OBJ_DIR)/%.elf
	$(OBJCOPY) -j .text -j .data -O $(HEXFORMAT) $< $@

# Install to the microcontroller
install: all $(HEXROMTRG)
	$(AVRDUDE) -c $(AVRDUDE_PROGRAMMERID) \
	-p $(MCU) -P usb -e -U flash:w:$(HEXROMTRG)

# Clean build files
clean:
	$(REMOVE) $(TRG) $(OBJ_DIR)/$(PROJECTNAME).map $(OBJDEPS) $(HEXTRG) $(OBJ_DIR)/*.d
	$(MAKE) -C $(LIB_DIR) clean
