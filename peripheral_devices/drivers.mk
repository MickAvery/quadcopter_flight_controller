DRIVERPATH = $(ROOT)/peripheral_devices

include $(DRIVERPATH)/lsm6dsl/lsm6dsl.mk

DRIVERSRC = $(LSM6DSLSRC)

DRIVERINC = $(LSM6DSLINC)