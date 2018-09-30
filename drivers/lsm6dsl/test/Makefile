CXX:=g++

PROJ_ROOT_DIR=../../..

CPPUTEST_HOME=$(PROJ_ROOT_DIR)/third_party/CppUTest

# CUT = Code-Under-Test
CUT_SRC_DIR=..

INC_DIRS=-I.. \
-I$(CPPUTEST_HOME)/include

CPPFLAGS=$(INC_DIRS) -std=c++11
#CXXFLAGS += -include $(CPPUTEST_HOME)/include/CppUTest/MemoryLeakDetectorNewMacros.h
#CFLAGS += -include $(CPPUTEST_HOME)/include/CppUTest/MemoryLeakDetectorMallocMacros.h

LD_LIBRARIES = -L$(CPPUTEST_HOME)/cpputest_build/lib -lCppUTest -lCppUTestExt \

TESTS=lsm6dsl_driver_unit_test

all:$(TESTS)

run:
	./lsm6dsl_driver_unit_test -c

clean:
	rm -rf *.o lsm6dsl_driver_unit_test

lsm6dsl.o: $(CUT_SRC_DIR)/lsm6dsl.c
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) -c $(CUT_SRC_DIR)/lsm6dsl.c

lsm6dsl_driver_test.o: lsm6dsl_driver_test.cc
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) -c lsm6dsl_driver_test.cc

lsm6dsl_driver_unit_test: lsm6dsl.o lsm6dsl_driver_test.o
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) $^ -o $@ $(LD_LIBRARIES)
