# Makefile for Basler pylon sample program
.PHONY: all clean

# GIT REPO path
GIT_REPO := ../..

# SRC_DIR
DETECTOR_DIR := $(GIT_REPO)/prog_odroid/basler/x4_detector
SERIAL_DIR := $(GIT_REPO)/prog_odroid/serial_com

# The program to build	
NAME   := project_detector
LIB_CPP:= $(DETECTOR_DIR)/camera.cpp $(DETECTOR_DIR)/class_types.cpp $(DETECTOR_DIR)/custom_functions.cpp $(DETECTOR_DIR)/drawing.cpp $(DETECTOR_DIR)/img_processing.cpp $(SERIAL_DIR)/serial_lib.c $(SERIAL_DIR)/types_convert.cpp
SRC	   := $(LIB_CPP) project_detector.cpp
OBJ	   := $(SRC:.cpp=.o)

# Installation directories for pylon
PYLON_ROOT ?= /opt/pylon5

# Build tools and flags
LD         	:= $(CXX)
CPPFLAGS   	:= $(shell $(PYLON_ROOT)/bin/pylon-config --cflags) $(shell pkg-config opencv --cflags)
# for release
CXXFLAGS	:= -O2 -std=c++11
# for debugging
DBGCXXFLAGS	:= -O0 -g -std=c++11
LDFLAGS    	:= $(shell $(PYLON_ROOT)/bin/pylon-config --libs-rpath)
LDLIBS     	:= $(shell $(PYLON_ROOT)/bin/pylon-config --libs) $(shell pkg-config opencv --libs) -I$(DETECTOR_DIR) -I$(SERIAL_DIR)

# Rules for building
all: $(NAME)

debug: CXXFLAGS := $(DBGCXXFLAGS)
debug: $(NAME)

$(NAME): $(OBJ)
	$(LD) $(CPPFLAGS) $(LDFLAGS) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

%.o: %.cpp
	$(LD) $(CPPFLAGS) $(LDFLAGS) $(CXXFLAGS) -c -o $@ $< $(LDLIBS)

clean:
	$(RM) $(OBJ) $(NAME)
