/**
 * \file lsm6dsl_driver_test.cc
 *
 * \author Mav Cuyugan
 * \brief  LSM6DSL driver unit test
 **/

#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include "lsm6dsl.h"

I2CDriver i2c;

TEST_GROUP(LSM6DSLStartTestGroup)
{
  lsm6dsl_handle_t* lsm6dsl = &LSM6DSL_HANDLE;
  const lsm6dsl_config_t cfg = {
    &i2c
  };

  void setup()
  {
  }

  void teardown()
  {
    mock().checkExpectations();
    mock().clear();
  }
};

TEST(LSM6DSLStartTestGroup, lsm6dslStartSuccess)
{
  mock().expectOneCall("i2cAcquireBus");
  mock().expectOneCall("i2cReleaseBus");
  (void)lsm6dslStart(lsm6dsl, &cfg);
}

int main(int argc, char** argv)
{
  return RUN_ALL_TESTS(argc, argv);
}
