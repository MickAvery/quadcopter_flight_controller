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
#include "lsm6dsl_reg.h"

static I2CDriver i2c;

static void expect_i2c_read(
  uint8_t* txbuf,
  uint8_t* rxbuf,
  size_t rxbytes,
  i2caddr_t addr,
  msg_t ret)
{
  mock().expectOneCall("i2cMasterReceiveTimeout")
        .withParameter("addr", addr)
        .withParameter("txbuf", txbuf, 1)
        .withParameter("txbytes", 1)
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
  membuf[0] = CTRL1_XL_ADDR;
  membuf[1] = 0x00U;

  membuf[2] = CTRL2_G_ADDR;
  membuf[3] = 0x00U;

  membuf[4] = CTRL1_XL_ADDR;
  membuf[5] = membuf[1] | (cfg->accel_odr << 4) | (cfg->accel_fs << 2);

  membuf[6] = CTRL2_G_ADDR;
  membuf[7] = membuf[3] | (cfg->gyro_odr << 4) | (cfg->gyro_fs << 2);

  /* read from and write to CTRL4_C register */
  membuf[8]  = CTRL4_C_ADDR;
  membuf[9]  = 0x00U;
  membuf[10] = CTRL4_C_ADDR;
  membuf[11] = (membuf[9] & (~LPF1_SEL_G)) | (cfg->gyro_lpf_en << 1);

  /* read from and write to CTRL6_C register */
  membuf[12]  = CTRL6_C_ADDR;
  membuf[13]  = 0x00U;
  membuf[14]  = CTRL6_C_ADDR;
  membuf[15]  = (membuf[13] & (~FTYPE)) | (cfg->gyro_lpf_bw);

  mock().expectOneCall("i2cAcquireBus");
  expect_i2c_write(&membuf[0], 1U, &membuf[1], 1, LSM6DSL_I2C_SLAVEADDR, MSG_OK);
  expect_i2c_write(&membuf[2], 1U, &membuf[3], 1, LSM6DSL_I2C_SLAVEADDR, MSG_OK);
  expect_i2c_write(&membuf[8], 1U, &membuf[9], 1, LSM6DSL_I2C_SLAVEADDR, MSG_OK);
  expect_i2c_write(&membuf[12], 1U, &membuf[9], 1, LSM6DSL_I2C_SLAVEADDR, MSG_OK);

  expect_i2c_write(&membuf[4], 2U, NULL, 0U, LSM6DSL_I2C_SLAVEADDR, MSG_OK);
  expect_i2c_write(&membuf[6], 2U, NULL, 0U, LSM6DSL_I2C_SLAVEADDR, MSG_OK);
  expect_i2c_write(&membuf[10], 2U, NULL, 0U, LSM6DSL_I2C_SLAVEADDR, MSG_OK);

  if(cfg->gyro_lpf_en)
    expect_i2c_write(&membuf[14], 2U, NULL, 0U, LSM6DSL_I2C_SLAVEADDR, MSG_OK);

  mock().expectOneCall("i2cReleaseBus");
}

TEST_GROUP(LSM6DSLStartTestGroup)
{
  lsm6dsl_handle_t lsm6dsl;
  const lsm6dsl_config_t cfg = {
    .i2c_drv   = &i2c,
    .accel_odr = LSM6DSL_ACCEL_12_5_Hz,
    .gyro_odr  = LSM6DSL_GYRO_12_5_Hz,
    .accel_fs  = LSM6DSL_ACCEL_4G,
    .gyro_fs   = LSM6DSL_GYRO_1000DPS,
    .gyro_lpf_en = true,
    .gyro_lpf_bw = LSM6DSL_GYRO_LPF_BW_D
  };

  void setup()
  {
    lsm6dslObjectInit(&lsm6dsl);
    LONGS_EQUAL(LSM6DSL_STATE_STOP, lsm6dsl.state);
    POINTERS_EQUAL(NULL, lsm6dsl.cfg);
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

  LONGS_EQUAL(LSM6DSL_OK, lsm6dslStart(&lsm6dsl, &cfg));
  LONGS_EQUAL(LSM6DSL_STATE_RUNNING, lsm6dsl.state);
  POINTERS_EQUAL(&cfg, lsm6dsl.cfg)
}

TEST_GROUP(LSM6DSLReadTestGroup)
{
  lsm6dsl_handle_t lsm6dsl;
  const lsm6dsl_config_t cfg = {
    .i2c_drv   = &i2c,
    .accel_odr = LSM6DSL_ACCEL_12_5_Hz,
    .gyro_odr  = LSM6DSL_GYRO_12_5_Hz,
    .accel_fs  = LSM6DSL_ACCEL_2G,
    .gyro_fs   = LSM6DSL_GYRO_250DPS,
    .gyro_lpf_en = true,
    .gyro_lpf_bw = LSM6DSL_GYRO_LPF_BW_D
  };

  void setup()
  {
    mock().disable();
    lsm6dslObjectInit(&lsm6dsl);
    (void)lsm6dslStart(&lsm6dsl, &cfg);
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

  expect_i2c_write(&status_reg_addr, 1, &status_reg, 1, LSM6DSL_I2C_SLAVEADDR, MSG_OK);
  expect_i2c_write(
    &read_addr, 1,
    (uint8_t*)raw_sensor_bytes, sizeof(raw_sensor_bytes),
    LSM6DSL_I2C_SLAVEADDR, MSG_OK);

  mock().expectOneCall("i2cReleaseBus");

  LONGS_EQUAL(LSM6DSL_OK, lsm6dslRead(&lsm6dsl, &readings));

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

  expect_i2c_write(&status_addr, 1, &status_reg, 1, LSM6DSL_I2C_SLAVEADDR, MSG_OK);

  mock().expectOneCall("i2cReleaseBus");

  LONGS_EQUAL(LSM6DSL_DATA_NOT_AVAILABLE, lsm6dslRead(&lsm6dsl, &readings));
}

TEST_GROUP(LSM6DSLAccelOffsetTestGroup)
{
  lsm6dsl_handle_t lsm6dsl;
  const lsm6dsl_config_t cfg = {
    .i2c_drv   = &i2c,
    .accel_odr = LSM6DSL_ACCEL_12_5_Hz,
    .gyro_odr  = LSM6DSL_GYRO_12_5_Hz,
    .accel_fs  = LSM6DSL_ACCEL_2G,
    .gyro_fs   = LSM6DSL_GYRO_250DPS,
    .gyro_lpf_en = true,
    .gyro_lpf_bw = LSM6DSL_GYRO_LPF_BW_D
  };

  void setup()
  {
    mock().disable();
    lsm6dslObjectInit(&lsm6dsl);
    (void)lsm6dslStart(&lsm6dsl, &cfg);
    mock().enable();
  }

  void teardown()
  {
    mock().checkExpectations();
    mock().clear();
  }
};

TEST(LSM6DSLAccelOffsetTestGroup, lsm6dslSetAccelOffsetsTest)
{
  mock().expectOneCall("i2cAcquireBus");

  int8_t offsets[3U] = { -4, 0, 10 };

  int8_t x_off[2] = { X_OFS_USR_ADDR, -offsets[0] }; // sensor internally adds contents of X_OFF register to X-axis readings
  int8_t y_off[2] = { Y_OFS_USR_ADDR, -offsets[1] }; // sensor internally adds contents of Y_OFF register to Y-axis readings
  int8_t z_off[2] = { Z_OFS_USR_ADDR,  offsets[2] }; // sensor internally subtracts contents of Z_OFF register to Z-axis readings

  expect_i2c_write( x_off, 2, NULL, 0, LSM6DSL_I2C_SLAVEADDR, MSG_OK );
  expect_i2c_write( y_off, 2, NULL, 0, LSM6DSL_I2C_SLAVEADDR, MSG_OK );
  expect_i2c_write( z_off, 2, NULL, 0, LSM6DSL_I2C_SLAVEADDR, MSG_OK );

  mock().expectOneCall("i2cReleaseBus");

  LONGS_EQUAL(LSM6DSL_OK, lsm6dslSetAccelOffset(&lsm6dsl, offsets));
}

TEST_GROUP(LSM6DSLPassThroughTestGroup)
{
  lsm6dsl_handle_t lsm6dsl;
  const lsm6dsl_config_t cfg = {
    .i2c_drv   = &i2c,
    .accel_odr = LSM6DSL_ACCEL_12_5_Hz,
    .gyro_odr  = LSM6DSL_GYRO_12_5_Hz,
    .accel_fs  = LSM6DSL_ACCEL_2G,
    .gyro_fs   = LSM6DSL_GYRO_250DPS,
    .gyro_lpf_en = true,
    .gyro_lpf_bw = LSM6DSL_GYRO_LPF_BW_D
  };

  void setup()
  {
    mock().disable();
    lsm6dslObjectInit(&lsm6dsl);
    (void)lsm6dslStart(&lsm6dsl, &cfg);
    mock().enable();
  }

  void teardown()
  {
    mock().checkExpectations();
    mock().clear();
  }
};

TEST(LSM6DSLPassThroughTestGroup, lsm6dslPassThroughSuccessTest)
{
  uint8_t membuf[16] = {0};

  mock().expectOneCall("i2cAcquireBus");

  /* read MASTER_CONFIG and set START_CONFIG bit */
  membuf[0] = 0x1AU; /* MASTER_CONFIG address */
  membuf[1] = 0x00U; /* MASTER_CONFIG default value */
  expect_i2c_write(&membuf[0], 1, &membuf[1], 1, LSM6DSL_I2C_SLAVEADDR, MSG_OK);

  membuf[2] = 0x1AU; /* MASTER_CONFIG address */
  membuf[3] = membuf[1] | (1 << 4); /* setting START_CONFIG */
  expect_i2c_write(&membuf[2], 2, NULL, 0, LSM6DSL_I2C_SLAVEADDR, MSG_OK);

  /* reset MASTER_ON, START_CONFIG, and PULL_UP_EN
   * set PASS_THROUGH_MODE
   */
  membuf[4] = 0x1AU;
  membuf[5] = membuf[3] & ~(1 | (1 << 4) | (1 << 3));
  membuf[5] |= (1 << 2);

  expect_i2c_write(&membuf[4], 2, NULL, 0, LSM6DSL_I2C_SLAVEADDR, MSG_OK);

  mock().expectOneCall("i2cReleaseBus");

  LONGS_EQUAL(LSM6DSL_OK, lsm6dslPassthroughEnable(&lsm6dsl));
  LONGS_EQUAL(LSM6DSL_STATE_PASSTHROUGH, lsm6dsl.state);
}

int main(int argc, char** argv)
{
  return RUN_ALL_TESTS(argc, argv);
}
