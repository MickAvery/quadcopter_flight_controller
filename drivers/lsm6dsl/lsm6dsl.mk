LSM6DSL_PATH = $(DRIVERPATH)/lsm6dsl

LSM6DSL_TEST_DIR = $(LSM6DSL_PATH)/test

include $(LSM6DSL_TEST_DIR)/lsm6dsl_test.mk 

LSM6DSLSRC = $(LSM6DSL_PATH)/lsm6dsl.c

LSM6DSLINC = $(LSM6DSL_PATH)