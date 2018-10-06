IIS2MDC_CUT_DIR=$(IIS2MDC_TEST_DIR)/..

IIS2MDCOBJ=$(addprefix $(TESTOBJDIR)/,i2c_iis2mdc.o chsys_iis2mdc.o iis2mdc.o iis2mdc_driver_test.o)

$(TESTOBJDIR)/i2c_iis2mdc.o: $(IIS2MDC_TEST_DIR)/mocks/i2c.c
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) -o $@ -c $(IIS2MDC_TEST_DIR)/mocks/i2c.c

$(TESTOBJDIR)/chsys_iis2mdc.o: $(IIS2MDC_TEST_DIR)/mocks/chsys.c
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) -o $@ -c $(IIS2MDC_TEST_DIR)/mocks/chsys.c

$(TESTOBJDIR)/iis2mdc.o: $(IIS2MDC_CUT_DIR)/iis2mdc.c
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) -o $@ -c $(IIS2MDC_CUT_DIR)/iis2mdc.c

$(TESTOBJDIR)/iis2mdc_driver_test.o: $(IIS2MDC_TEST_DIR)/iis2mdc_driver_test.cc
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) -o $@ -c $(IIS2MDC_TEST_DIR)/iis2mdc_driver_test.cc

iis2mdc_unit_test: $(IIS2MDCOBJ)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CFLAGS) $^ -o $(TEST_DIR)/$@ $(LD_LIBRARIES)