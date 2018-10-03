/**
 * \file   i2c.c
 * \author Mav Cuyugan
 * \brief  i2c.c mocks
 */

#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include "hal.h"

msg_t i2cMasterTransmitTimeout(I2CDriver *i2cp,
                               i2caddr_t addr,
                               const uint8_t *txbuf, size_t txbytes,
                               uint8_t *rxbuf, size_t rxbytes,
                               systime_t timeout)
{
  (void)i2cp;
  (void)timeout;

  mock().actualCall("i2cMasterTransmitTimeout")
        .withParameter("addr", addr)
        .withParameter("txbuf", txbuf, txbytes).withParameter("txbytes", txbytes)
        .withOutputParameter("rxbuf", rxbuf).withParameter("rxbytes", rxbytes);
  return mock().returnIntValueOrDefault(MSG_OK);
}

msg_t i2cMasterReceiveTimeout(I2CDriver *i2cp,
                              i2caddr_t addr,
                              uint8_t *rxbuf, size_t rxbytes,
                              systime_t timeout)
{
  (void)i2cp;
  (void)timeout;

  mock().actualCall("i2cMasterReceiveTimeout")
        .withParameter("addr", addr)
        .withOutputParameter("rxbuf", rxbuf).withParameter("rxbytes", rxbytes);
  return mock().returnIntValueOrDefault(MSG_OK);
}

void i2cAcquireBus(I2CDriver *i2cp)
{
  (void)i2cp;
  mock().actualCall("i2cAcquireBus");
}

void i2cReleaseBus(I2CDriver *i2cp)
{
  (void)i2cp;
  mock().actualCall("i2cReleaseBus");
}
