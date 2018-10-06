LSM6DSL_CUT_DIR=$(LSM6DSL_TEST_DIR)/..

LSM6DSLOBJ=$(addprefix $(TESTOBJDIR)/,i2c.o chsys.o lsm6dsl.o lsm6dsl_driver_test.o)

$(TESTOBJDIR)/i2c.o: $(LSM6DSL_TEST_DIR)/mocks/i2c.c
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) -o $@ -c $(LSM6DSL_TEST_DIR)/mocks/i2c.c

$(TESTOBJDIR)/chsys.o: $(LSM6DSL_TEST_DIR)/mocks/chsys.c
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) -o $@ -c $(LSM6DSL_TEST_DIR)/mocks/chsys.c

$(TESTOBJDIR)/lsm6dsl.o: $(LSM6DSL_CUT_DIR)/lsm6dsl.c
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) -o $@ -c $(LSM6DSL_CUT_DIR)/lsm6dsl.c

$(TESTOBJDIR)/lsm6dsl_driver_test.o: $(LSM6DSL_TEST_DIR)/lsm6dsl_driver_test.cc
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) -o $@ -c $(LSM6DSL_TEST_DIR)/lsm6dsl_driver_test.cc

lsm6dsl_unit_test: $(LSM6DSLOBJ)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) $^ -o $(TEST_DIR)/$@ $(LD_LIBRARIES)