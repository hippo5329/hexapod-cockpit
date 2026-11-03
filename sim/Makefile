# Host simulator: the firmware's gait, IK and servo libraries, compiled natively.
#   make -C sim          build
#   make -C sim run      build and run every scenario
LIB := ../firmware/lib
SRC := hexa_sim.cpp $(LIB)/hexa_kin/hexa_kin.cpp $(LIB)/hexa_gait/hexa_gait.cpp $(LIB)/hexa_servo/hexa_servo.cpp
CXXFLAGS ?= -O2 -g
CXXFLAGS += -std=c++17 -Wall -Wextra -I$(LIB)/hexa_kin -I$(LIB)/hexa_gait -I$(LIB)/hexa_servo

hexa_sim: $(SRC) $(wildcard $(LIB)/*/*.h)
	$(CXX) $(CXXFLAGS) -o $@ $(SRC) -lm

run: hexa_sim
	./hexa_sim

clean:
	rm -f hexa_sim

.PHONY: run clean
