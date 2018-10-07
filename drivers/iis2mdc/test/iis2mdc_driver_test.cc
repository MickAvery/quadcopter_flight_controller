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

static I2CDriver i2c;
static i2caddr_t iis2mdc_addr = 0b00011110;
static uint8_t cfg_a_addr = 0x60U;

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
  const iis2mdc_config_t* cfg,
  uint8_t* membuf,
  size_t membyte)
{
  membuf[0] = cfg_a_addr;
  membuf[1] = 0b00000011; /* default register value */

  uint8_t rx = membuf[1] & 0; /* clear bits[1:0] for continuous mode */
  rx |= 

  membuf[2] = cfg_a_addr;
  membuf[3] = cfg->odr << 2;

  mock().expectOneCall("i2cAcquireBus");
  expect_i2c_write(&membuf[0], 1, &membuf[1], 1, iis2mdc_addr, MSG_OK);
  expect_i2c_write(&membuf[2], 2, NULL, 0, iis2mdc_addr, MSG_OK);
  mock().expectOneCall("i2cReleaseBus");
}

TEST_GROUP(IIS2MDCStartTestGroup)
{
  iis2mdc_handle_t iis2mdc;
  iis2mdc_config_t cfg =
  {
    &i2c,
    IIS2MDC_ODR_50_Hz
  };

  void setup()
  {
    iis2mdcObjectInit(&iis2mdc);
    LONGS_EQUAL(IIS2MDC_STOP, iis2mdc.state);
    POINTERS_EQUAL(NULL, iis2mdc.cfg);
  }

  void teardown()
  {
    mock().checkExpectations();
    mock().clear();
  }
};

TEST(IIS2MDCStartTestGroup, startFxnTest)
{
  uint8_t membuf[16] = {0U};

  expect_startup_sequence(&cfg, membuf, sizeof(membuf));

  LONGS_EQUAL(IIS2MDC_STATUS_OK, iis2mdcStart(&iis2mdc, &cfg));
  LONGS_EQUAL(IIS2MDC_RUNNING, iis2mdc.state);
  POINTERS_EQUAL(&cfg, iis2mdc.cfg);
}

TEST_GROUP(IIS2MDCReadTestGroup)
{
  iis2mdc_handle_t iis2mdc;
  iis2mdc_config_t cfg =
  {
    &i2c,
    IIS2MDC_ODR_100_Hz
  };

  void setup()
  {
    mock().disable();
    iis2mdcObjectInit(&iis2mdc);
    (void)iis2mdcStart(&iis2mdc, &cfg);
    mock().enable();
  }

  void teardown()
  {
    mock().checkExpectations();
    mock().clear();
  }
};

TEST(IIS2MDCReadTestGroup, ReadSuccessful)
{
  iis2mdc_sensor_readings_t vals;

  mock().expectOneCall("i2cAcquireBus");

  /* expect to read status register */
  uint8_t a[] = {0x67U /* status reg addr */, (1 << 3) /* zxyda flag set */};
  expect_i2c_write(&a[0], 1, &a[1], 1, iis2mdc_addr, MSG_OK);

  /* expect to read output registers */
  uint8_t b = 0x68U;
  uint8_t raw[] = { /* LSB:MSB */
    0x21U, 0x00U,   /* x */
    0x1DU, 0xFFU,   /* y */
    0xCBU, 0xFEU    /* z */
  };

  expect_i2c_write(&b, 1, raw, sizeof(raw), iis2mdc_addr, MSG_OK);

  mock().expectOneCall("i2cReleaseBus");

  LONGS_EQUAL(IIS2MDC_STATUS_OK, iis2mdcRead(&iis2mdc, &vals));
  DOUBLES_EQUAL(49.5f,   vals.mag_x, 0.1f);
  DOUBLES_EQUAL(-340.5f, vals.mag_y, 0.1f);
  DOUBLES_EQUAL(-463.5f, vals.mag_z, 0.1f);
}


int main(int argc, char** argv)
{
  return RUN_ALL_TESTS(argc, argv);
}