/**
 * \file   i2c.c
 * \author Mav Cuyugan
 * \brief  i2c.c mocks
 */

#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport_c.h"
#include "hal.h"

msg_t i2cMasterTransmitTimeout(I2CDriver *i2cp,
                               i2caddr_t addr,
                               const uint8_t *txbuf, size_t txbytes,
                               uint8_t *rxbuf, size_t rxbytes,
                               systime_t timeout)
{

}

msg_t i2cMasterReceiveTimeout(I2CDriver *i2cp,
                              i2caddr_t addr,
                              uint8_t *rxbuf, size_t rxbytes,
                              systime_t timeout)
{

}

void i2cAcquireBus(I2CDriver *i2cp)
{
  mock_c()->actualCall("i2cAcquireBus");
}

void i2cReleaseBus(I2CDriver *i2cp)
{
  mock_c()->actualCall("i2cReleaseBus");
}
