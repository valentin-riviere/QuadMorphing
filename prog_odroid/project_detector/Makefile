# Makefile for Basler pylon sample program
.PHONY: all clean

# The program to build
NAME   := project_detector
LIB_CPP:= 
SRC	   := $(LIB_CPP) project_detector.cpp
OBJ	   := $(SRC:.cpp=.o)

# Installation directories for pylon
PYLON_ROOT ?= /opt/pylon5
GIT_REPO := ../..

# Build tools and flags
LD         	:= $(CXX)
CPPFLAGS   	:= $(shell $(PYLON_ROOT)/bin/pylon-config --cflags) $(shell pkg-config opencv --cflags)
# for release
CXXFLAGS	:= -O2 -std=c++11
# for debugging
DBGCXXFLAGS	:= -O0 -g -std=c++11
LDFLAGS    	:= $(shell $(PYLON_ROOT)/bin/pylon-config --libs-rpath)
LDLIBS     	:= $(shell $(PYLON_ROOT)/bin/pylon-config --libs) $(shell pkg-config opencv --libs) $(GIT_REPO)/prog_odroid/basler/x4_detector/*.cpp $(GIT_REPO)/prog_odroid/basler/x4_detector/*.cpp

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
