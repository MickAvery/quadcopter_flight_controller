/**
 * \file   main.c
 * \author Mav Cuyugan
 * \brief  Main app point of entry
 **/

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#include "lsm6dsl.h"
#include "iis2mdc.h"
#include "pinconf.h"

static lsm6dsl_sensor_readings_t readings;
static iis2mdc_sensor_readings_t mag_readings;

#define SHELL_WORKING_AREA_SIZE THD_WORKING_AREA_SIZE(2048)

static void csv(BaseSequentialStream* chp, int argc, char* argv[])
{
  (void)argc;
  (void)argv;

  while(true) {
    if(((SerialDriver*)chp)->vmt->gett(chp, 100) == 3) {
      break;
    }

    chprintf(
      chp,
      "%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\t%0.1f\n",
      readings.gyro_x, readings.gyro_y, readings.gyro_z,
      readings.acc_x, readings.acc_y, readings.acc_z,
      mag_readings.mag_x, mag_readings.mag_z, mag_readings.mag_z);
    chThdSleepMilliseconds(3);
  }
}

static const ShellCommand shellcmds[] =
{
  {"csv", csv},
  {NULL, NULL}
};

static const ShellConfig shellcfg =
{
  (BaseSequentialStream*)&SD4,
  shellcmds
};

static const I2CConfig i2ccfg =
{
  OPMODE_I2C,
  400000,
  FAST_DUTY_CYCLE_2
};

static const lsm6dsl_config_t lsm6dsl_cfg =
{
  &I2CD2,
  LSM6DSL_104_Hz,
  LSM6DSL_ACCEL_2G,
  LSM6DSL_GYRO_250DPS
};

static const iis2mdc_config_t iis2mdc_cfg =
{
  &I2CD2,
  IIS2MDC_ODR_100_Hz
};

/*************************************************
 * Threads
 *************************************************/

static THD_WORKING_AREA(imuReadThreadWorkingArea, 1024);

static THD_FUNCTION(imuReadThread, arg)
{
  (void)arg;
  lsm6dsl_handle_t lsm6dsl;
  iis2mdc_handle_t iis2mdc;

  lsm6dslObjectInit(&lsm6dsl);
  iis2mdcObjectInit(&iis2mdc);

  if(lsm6dslStart(&lsm6dsl, &lsm6dsl_cfg) != LSM6DSL_OK) {
    /* failed to start driver */
  } else if(iis2mdcStart(&iis2mdc, &iis2mdc_cfg) != IIS2MDC_STATUS_OK) {
    /* failed to start driver */
  } else {

    while(true) {

      (void)lsm6dslRead(&lsm6dsl, &readings);
      (void)iis2mdcRead(&iis2mdc, &mag_readings);

      chThdSleepMilliseconds(10);

    }

  }

  chSysHalt("Failed to start IMUs");
}

/*************************************************
 * main
 *************************************************/

int main(void) {

  /* initialize system */
  halInit();
  chSysInit();

  /* start serial */
  palSetPadMode(UART_TX_PORT, UART_TX_PADNUM, PAL_MODE_ALTERNATE(UART_PIN_ALTMODE));
  palSetPadMode(UART_RX_PORT, UART_RX_PADNUM, PAL_MODE_ALTERNATE(UART_PIN_ALTMODE));
  sdStart(&SD4, NULL);

  shellInit();

  /* start I2C */
  palSetPadMode(I2C_SCL_PORT, I2C_SCL_PADNUM, PAL_MODE_ALTERNATE(I2C_PIN_ALTMODE));
  palSetPadMode(I2C_SDA_PORT, I2C_SDA_PADNUM, PAL_MODE_ALTERNATE(I2C_PIN_ALTMODE));
  i2cStart(&I2CD2, &i2ccfg);

  /* create threads */
  chThdCreateStatic(
    imuReadThreadWorkingArea,
    sizeof(imuReadThreadWorkingArea),
    NORMALPRIO,
    imuReadThread,
    NULL);

  while (1) {
    thread_t* shelltp = chThdCreateFromHeap(
      NULL,
      SHELL_WORKING_AREA_SIZE,
      "shell",
      NORMALPRIO,
      shellThread,
      (void*)&shellcfg);

    chThdWait(shelltp);
    chThdSleepMilliseconds(500);
  }

  return 0;
}