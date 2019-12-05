	# Makefile for real time triangulation programm
.PHONY: all clean

# The program to build
NAME       := triangulate_rt

# detect OS
OS := $(shell uname)

# if OSX
ifeq ($(OS),Darwin) 
SYSTEM_LIBRARY_DIR := /Library
# PYLON_ROOT ?= $(SYSTEM_LIBRARY_DIR)/Frameworks
PYLON_ROOT     ?= /opt/pylon5
PYLON_INCLUDE  := $(PYLON_ROOT)/pylon
PYLON_LIB      := /Library/Frameworks/pylon.framework/Libraries
# PYLON_LDFLAGS :=  -Wl,--enable-new-dtags -Wl,-rpath,$(PYLON_LIB)
# PYLON_LDFLAGS  :=  -Wl,-rpath,$(PYLON_LIB)
PYLON_LDFLAGS  :=  -Wl,-rpath,/Library/Frameworks
PYLON_LIBS     := -lpylonbase -lpylonutility -lGenApi_gcc_v3_1_Basler_pylon_v5_1 -lGCBase_gcc_v3_1_Basler_pylon_v5_1
# PYLON_LDLIBS   := $(PYLON_LIB) -Wl,-E
PYLON_LDLIBS   := $(PYLON_LIB)
# /Library/Frameworks/pylon.framework/Libraries
# Build tools and flags

CXX+=-std=c++11
LD         := $(CXX)
CPPFLAGS   := -I $(PYLON_ROOT) -I $(PYLON_INCLUDE)/GenICam -I $(PYLON_ROOT)/GenICam -DUSE_GIGE -I /usr/local/include/opencv4
CXXFLAGS   := #e.g., CXXFLAGS=-g -O0 for debugging
LDFLAGS    := $(PYLON_LDFLAGS) -I /usr/local/include/opencv4
LDLIBS     := -L $(PYLON_LDLIBS) $(PYLON_LIBS)  -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lopencv_video -lopencv_imgproc -lopencv_tracking  -lopencv_features2d -lopencv_calib3d -pthread 
# LDLIBS     := -L /Library/Frameworks/pylon.framework -framework /Library/Frameworks/pylon.framework/pylon  -L /Library/Frameworks/pylon.framework/Libraries  -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -pthread 
# linux
else
# Installation directories for pylon
PYLON_ROOT ?= /opt/pylon5
# Build tools and flags
CXX+=-std=c++11
LD         := $(CXX)
CPPFLAGS   := $(shell $(PYLON_ROOT)/bin/pylon-config --cflags) -DUSE_GIGE -I /usr/local/include/opencv4
CXXFLAGS   := #e.g., CXXFLAGS=-g -O0 for debugging
LDFLAGS    := $(shell $(PYLON_ROOT)/bin/pylon-config --libs-rpath) -I /usr/local/include/opencv4
LDLIBS     := $(shell $(PYLON_ROOT)/bin/pylon-config --libs) -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lopencv_video  -lopencv_imgproc -lopencv_tracking -lopencv_features2d -lopencv_calib3d -pthread 
# LFLIBS = -lopencv_core -lopencv_calib3d -lopencv_imgproc -lopencv_tracking -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_highgui -lopencv_flann

endif
# Rules for building

# tbb
CPPFLAGS+=-I /usr/local/Cellar/tbb/2019_U3_1/include 
LDFLAGS+=-I /usr/local/Cellar/tbb/2019_U3_1/include 
LDLIBS+=-L /usr/local/Cellar/tbb/2019_U3_1/lib -L/usr/local/lib/ 


all: $(NAME)

$(NAME): $(NAME).o
	$(LD) $(LDFLAGS) -o $@ $^ $(LDLIBS)
# 	$(LD) $(LDFLAGS) -o $@ $^ $(LDLIBS) jetsonGPIO.c

$(NAME).o: $(NAME).cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

clean:
	$(RM) $(NAME).o $(NAME)

debug: 
	@echo  "$(OS) 		" ;


# 	-I/opt/pylon5/include
# -Wl,--enable-new-dtags -Wl,-rpath,/opt/pylon5/lib64
# -L/opt/pylon5/lib64 -Wl,-E -lpylonbase -lpylonutility -lGenApi_gcc_v3_1_Basler_pylon -lGCBase_gcc_v3_1_Basler_pylon

# g++ -std=c++11 -I/opt/pylon5/include  -DUSE_GIGE -I /usr/local/include/opencv4  -c -o triangulate_rt.o triangulate_rt.cpp
# g++ -std=c++11 -Wl,--enable-new-dtags -Wl,-rpath,/opt/pylon5/lib64 -I /usr/local/include/opencv4 -o triangulate_rt triangulate_rt.o -L/opt/pylon5/lib64 -Wl,-E -lpylonbase -lpylonutility -lGenApi_gcc_v3_1_Basler_pylon -lGCBase_gcc_v3_1_Basler_pylon -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -pthread 