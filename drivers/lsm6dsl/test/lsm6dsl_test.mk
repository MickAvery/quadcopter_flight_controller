CUT_DIR=$(LSM6DSL_TEST_DIR)/..

i2c.o: $(LSM6DSL_TEST_DIR)/mocks/i2c.c
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) -c $(LSM6DSL_TEST_DIR)/mocks/i2c.c

chsys.o: $(LSM6DSL_TEST_DIR)/mocks/chsys.c
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) -c $(LSM6DSL_TEST_DIR)/mocks/chsys.c

lsm6dsl.o: $(CUT_DIR)/lsm6dsl.c
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) -c $(CUT_DIR)/lsm6dsl.c

lsm6dsl_driver_test.o: $(LSM6DSL_TEST_DIR)/lsm6dsl_driver_test.cc
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) -c $(LSM6DSL_TEST_DIR)/lsm6dsl_driver_test.cc

lsm6dsl_unit_test: i2c.o chsys.o lsm6dsl.o lsm6dsl_driver_test.o
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) $^ -o $(TEST_DIR)/$@ $(LD_LIBRARIES)