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
#include "lsm6dsl.h"

I2CDriver i2c;

static void expect_i2c_read(
  uint8_t* rxbuf,
  size_t rxbytes,
  i2caddr_t addr,
  msg_t ret)
{
  mock().expectOneCall("i2cMasterReceiveTimeout")
        .withParameter("addr", addr)
        .withOutputParameterReturning("rxbuf", rxbuf, rxbytes)
        .withParameter("rxbytes", rxbytes)
        .andReturnValue(ret);
}

static void expect_i2c_write(
  uint8_t* txbuf, size_t txbytes,
  uint8_t* rxbuf, size_t rxbytes,
  i2caddr_t addr,
  msg_t ret)
{
  mock().expectOneCall("i2cMasterTransmitTimeout")
        .withParameter("addr", addr)
        .withParameter("txbuf", txbuf, txbytes)
        .withParameter("txbytes", txbytes)
        .withOutputParameterReturning("rxbuf", rxbuf, rxbytes)
        .withParameter("rxbytes", rxbytes)
        .andReturnValue(ret);
}

static void expect_startup_sequence(
  const lsm6dsl_config_t* cfg,
  uint8_t* membuf,
  size_t membytes)
{
  membuf[0] = 0x00U;
  membuf[1] = membuf[0] | (cfg->odr << 4) | (cfg->accel_fs << 2);
  membuf[2] = membuf[0] | (cfg->odr << 4) | (cfg->gyro_fs << 2);

  mock().expectOneCall("i2cAcquireBus");
  expect_i2c_read(&membuf[0], 1U, 0x0010U, MSG_OK);
  expect_i2c_read(&membuf[0], 1U, 0x0011U, MSG_OK);

  expect_i2c_write(&membuf[1], 1U, NULL, 0U, 0x0010U, MSG_OK);
  expect_i2c_write(&membuf[2], 1U, NULL, 0U, 0x0011U, MSG_OK);
  mock().expectOneCall("i2cReleaseBus");
}

TEST_GROUP(LSM6DSLStartTestGroup)
{
  lsm6dsl_handle_t* lsm6dsl = &LSM6DSL_HANDLE;
  const lsm6dsl_config_t cfg = {
    &i2c,
    LSM6DSL_12_5_Hz,
    LSM6DSL_ACCEL_4G,
    LSM6DSL_GYRO_1000DPS
  };

  void setup()
  {
    lsm6dsl->state = LSM6DSL_STATE_STOP; /* TODO: maybe better to make objectInit() fxn */
  }

  void teardown()
  {
    mock().checkExpectations();
    mock().clear();
  }
};

TEST(LSM6DSLStartTestGroup, lsm6dslStartSuccess)
{
  uint8_t membuf[16] = {0};

  expect_startup_sequence(&cfg, membuf, 16);

  LONGS_EQUAL(LSM6DSL_OK, lsm6dslStart(lsm6dsl, &cfg));
  LONGS_EQUAL(LSM6DSL_STATE_RUNNING, lsm6dsl->state);
}

TEST_GROUP(LSM6DSLReadTestGroup)
{
  lsm6dsl_handle_t* lsm6dsl = &LSM6DSL_HANDLE;
  const lsm6dsl_config_t cfg = {
    &i2c,
    LSM6DSL_12_5_Hz,
    LSM6DSL_ACCEL_4G,
    LSM6DSL_GYRO_1000DPS
  };

  void setup()
  {
    mock().disable();
    lsm6dsl->state = LSM6DSL_STATE_STOP; /* TODO: maybe better to make objectInit() fxn */
    (void)lsm6dslStart(lsm6dsl, &cfg);
    mock().enable();
  }

  void teardown()
  {
    mock().checkExpectations();
    mock().clear();
  }
};

TEST(LSM6DSLReadTestGroup, lsm6dslTestValues)
{
  float accel_values[] = {350.0f, 1.0f, -350.0f};
  uint16_t accel_raw[] = {0x6916U, 0x0940U, 0x97E9U};

  float gyro_values[] = {100.0f, 200.0f, -100.0f};
  float gyro_raw[] = {0xA42CU, 0x4959U, 0x5CD3U};
}

int main(int argc, char** argv)
{
  return RUN_ALL_TESTS(argc, argv);
}
