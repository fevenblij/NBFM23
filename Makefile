###############################################################################
# Makefile for the project NBFM_simulator
###############################################################################

## General Flags
TARGET = NBFM_Simulator
CC = gcc

## Compile options common for all C compilation units.
CFLAGS = $(COMMON) -Warray-bounds  -DTESTING=yes

## Linker flags
LDFLAGS = $(COMMON)

## Objects that must be built in order to link
OBJECTS = $(TARGET) $(TARGET).o

## Objects explicitly added by the user
#LINKONLYOBJECTS = 

## Build
all: $(TARGET) 

##Link
#$(TARGET): $(OBJECTS)
#	 $(CC) -DESTING  $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS)
