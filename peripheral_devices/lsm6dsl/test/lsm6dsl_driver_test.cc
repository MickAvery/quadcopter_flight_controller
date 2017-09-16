/**
 * \file lsm6dsl_driver_test.cc
 *
 * \author Mav Cuyugan
 * \brief  LSM6DSL driver unit test
 **/

#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport_c.h"
#include "lsm6dsl.h"

TEST_GROUP(LSM6DSLTestGroupSimple)
{
  LSM6DSLHandle hndl;
  LSM6DSLConfig cfg;

  void setup()
  {
    lsm6dslObjectInit(&hndl);
  }

  void teardown()
  {
    mock_c()->checkExpectations();
    mock_c()->clear();
  }
};

TEST(LSM6DSLTestGroupSimple, lsm6dslObjectInit)
{
  LONGS_EQUAL(LSM6DSL_STATE_STOP, hndl.state);
  POINTERS_EQUAL(NULL, hndl.cfg);
}

TEST(LSM6DSLTestGroupSimple, lsm6dslStartSuccess)
{

}

TEST(LSM6DSLTestGroupSimple, lsm6dslStartNullArgs)
{

}

int main(int argc, char** argv)
{
  return RUN_ALL_TESTS(argc, argv);
}
