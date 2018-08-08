# Makefile for Basler pylon sample program
.PHONY: all clean

# The program to build
NAME   := ttc_estimation
SRC	   := string_tools.cpp ttc_estimation.cpp
OBJ	   := $(SRC:.cpp=.o)

# Installation directories for pylon
PYLON_ROOT ?= /opt/pylon5

# Build tools and flags
LD         := $(CXX)
CPPFLAGS   := $(shell $(PYLON_ROOT)/bin/pylon-config --cflags) $(shell pkg-config opencv --cflags)
CXXFLAGS   := -O2 -std=c++11
DBGCXXFLAGS	:= -O0 -g -std=c++11
LDFLAGS    := $(shell $(PYLON_ROOT)/bin/pylon-config --libs-rpath)
LDLIBS     := $(shell $(PYLON_ROOT)/bin/pylon-config --libs) $(shell pkg-config opencv --libs)

# Rules for building
all: $(NAME)

debug: CXXFLAGS := $(DBGCXXFLAGS)
debug: $(NAME)

$(NAME): $(OBJ)
	$(LD) $(CPPFLAGS) $(LDFLAGS) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

%.o: %.cpp
	$(LD) $(CPPFLAGS) $(LDFLAGS) $(CXXFLAGS) -c -o $@ $< $(LDLIBS)

clean:
	$(RM) $(OBJ) $(NAME).o $(NAME)
