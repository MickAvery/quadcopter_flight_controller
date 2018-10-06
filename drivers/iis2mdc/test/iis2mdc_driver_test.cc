/**
 * \file lsm6dsl_driver_test.cc
 *
 * \author Mav Cuyugan
 * \brief  LSM6DSL driver unit test
 **/

#include <stdint.h>
#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include "iis2mdc.h"

TEST_GROUP(IIS2MDCStartTestGroup)
{
  void setup()
  {

  }

  void teardown()
  {
    mock().checkExpectations();
    mock().clear();
  }
};

TEST(IIS2MDCStartTestGroup, startFxnTest)
{
  LONGS_EQUAL(1, 2);
}

int main(int argc, char** argv)
{
  return RUN_ALL_TESTS(argc, argv);
}