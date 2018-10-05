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

static I2CDriver i2c;
static i2caddr_t lsm6dsl_addr = 0b01101010;

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
  membuf[0] = 0x10U;
  membuf[1] = 0x00U;

  membuf[2] = 0x11U;
  membuf[3] = 0x00U;

  membuf[4] = 0x10U;
  membuf[5] = membuf[0] | (cfg->odr << 4) | (cfg->accel_fs << 2);

  membuf[6] = 0x11U;
  membuf[7] = membuf[0] | (cfg->odr << 4) | (cfg->gyro_fs << 2);

  mock().expectOneCall("i2cAcquireBus");
  expect_i2c_write(&membuf[0], 1U, &membuf[1], 1, lsm6dsl_addr, MSG_OK);
  expect_i2c_write(&membuf[2], 1U, &membuf[3], 1, lsm6dsl_addr, MSG_OK);

  expect_i2c_write(&membuf[4], 2U, NULL, 0U, lsm6dsl_addr, MSG_OK);
  expect_i2c_write(&membuf[6], 2U, NULL, 0U, lsm6dsl_addr, MSG_OK);
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
    LSM6DSL_ACCEL_2G,
    LSM6DSL_GYRO_250DPS
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
  uint16_t raw_sensor_bytes[] =
  {
    0x2CA4, 0x5949, 0xD35C, /* raw gyroscope bytes */
    0x1669, 0x4009, 0xE997  /* raw accel bytes */
  };

  float gyro_values[] = {100.0f, 200.0f, -100.0f};
  float accel_values[] = {350.0f, 1000.0f, -350.0f};

  uint8_t status_reg_addr= 0x1EU;
  uint8_t status_reg = 0x03U;

  uint8_t read_addr = 0x22U;

  lsm6dsl_sensor_readings_t readings;

  mock().expectOneCall("i2cAcquireBus");

  expect_i2c_write(&status_reg_addr, 1, &status_reg, 1, lsm6dsl_addr, MSG_OK);
  expect_i2c_write(
    &read_addr, 1,
    (uint8_t*)raw_sensor_bytes, sizeof(raw_sensor_bytes),
    lsm6dsl_addr, MSG_OK);

  mock().expectOneCall("i2cReleaseBus");

  LONGS_EQUAL(LSM6DSL_OK, lsm6dslRead(lsm6dsl, &readings));

  DOUBLES_EQUAL(gyro_values[0], readings.gyro_x / 1000.0, 0.1f);
  DOUBLES_EQUAL(gyro_values[1], readings.gyro_y / 1000.0, 0.1f);
  DOUBLES_EQUAL(gyro_values[2], readings.gyro_z / 1000.0, 0.1f);

  DOUBLES_EQUAL(accel_values[0], readings.acc_x, 0.1f);
  DOUBLES_EQUAL(accel_values[1], readings.acc_y, 0.1f);
  DOUBLES_EQUAL(accel_values[2], readings.acc_z, 0.1f);
}

TEST(LSM6DSLReadTestGroup, lsm6dslReadingsNotReady)
{
  uint8_t status_addr = 0x1EU;
  uint8_t status_reg = 0x00U;
  lsm6dsl_sensor_readings_t readings;

  mock().expectOneCall("i2cAcquireBus");

  expect_i2c_write(&status_addr, 1, &status_reg, 1, lsm6dsl_addr, MSG_OK);

  mock().expectOneCall("i2cReleaseBus");

  LONGS_EQUAL(LSM6DSL_DATA_NOT_AVAILABLE, lsm6dslRead(lsm6dsl, &readings));
}

int main(int argc, char** argv)
{
  return RUN_ALL_TESTS(argc, argv);
}
