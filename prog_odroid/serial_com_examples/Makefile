# Makefile for Basler pylon sample program
.PHONY: all clean

# The program to build
NAME   := serial_gum_test
LIB_CPP:= types_convert.cpp serial_lib.c
SRC	   := $(LIB_CPP) serial_gum_test.cpp
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
LDLIBS     	:= $(shell $(PYLON_ROOT)/bin/pylon-config --libs) $(shell pkg-config opencv --libs)

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
