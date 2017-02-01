# the compiler: gcc for C program, define as g++ for C++
CC = g++

# compiler flags:
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings
CFLAGS  = `pkg-config --cflags --libs opencv` -lboost_system -lboost_thread -lpthread -O3 -floop-nest-optimize -floop-parallelize-all -ftree-loop-distribution -std=c++11


# the build target executable:
TARGET1 = vision
TARGET2 = server

all: $(TARGET)

	$(TARGET): $(TARGET1).cpp
	$(CC) src/$(TARGET1).cpp -o bin/$(TARGET1) $(CFLAGS)

	$(TARGET): $(TARGET2).cpp
	$(CC) src/$(TARGET2).cpp -o bin/$(TARGET2) $(CFLAGS)
