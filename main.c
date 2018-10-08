/**
 * \file   main.c
 * \author Mav Cuyugan
 * \brief  Main app point of entry
 **/

#include <math.h>

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#include "lsm6dsl.h"
#include "iis2mdc.h"
#include "pinconf.h"

static lsm6dsl_sensor_readings_t readings;
static iis2mdc_sensor_readings_t mag_readings;
static float euler_angles[3] = {0.0f};

static lsm6dsl_handle_t lsm6dsl;
static iis2mdc_handle_t iis2mdc;

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
      "%4.1f\t%4.1f\t%4.1f\t"
      "%2.1f\t%2.1f\t%2.1f\t"
      "%3.1f\t%3.1f\t%3.1f\t"
      "%4.1f\t%4.1f\t%4.1f\n",
      readings.gyro_x / 1000.0f, readings.gyro_y / 1000.0f, readings.gyro_z / 1000.0f,
      readings.acc_x / 1000.0f, readings.acc_y / 1000.0f, readings.acc_z / 1000.0f,
      mag_readings.mag_x / 1000.0f, mag_readings.mag_y / 1000.0f, mag_readings.mag_z / 1000.0f,
      euler_angles[0], euler_angles[1], euler_angles[2]);

    chThdSleepMilliseconds(3);
  }
}

static void mag_calibrate(BaseSequentialStream* chp, int argc, char* argv[])
{
  (void)argc;
  (void)argv;

  chprintf(
    chp,
    "Magnetometer calibration sequence\n"
    "Rotate device around\n");

  float acc_x_max = 0.0f;
  float acc_x_min = 0.0f;
  float acc_y_max = 0.0f;
  float acc_y_min = 0.0f;
  float acc_z_max = 0.0f;
  float acc_z_min = 0.0f;

  float acc_x_diff = 0.0f;
  float acc_y_diff = 0.0f;
  float acc_z_diff = 0.0f;

  float mag_x_max = 0.0f;
  float mag_x_min = 0.0f;
  float mag_y_max = 0.0f;
  float mag_y_min = 0.0f;
  float mag_z_max = 0.0f;
  float mag_z_min = 0.0f;

  bool x_done = false;
  bool y_done = false;
  bool z_done = false;

  while(
    (acc_x_diff < 2000.0f) ||
    (acc_y_diff < 2000.0f) ||
    (acc_z_diff < 2000.0f)) {

    if(((SerialDriver*)chp)->vmt->gett(chp, 100) == 3) {
      return;
    }

    if(readings.acc_x > acc_x_max) acc_x_max = readings.acc_x;
    if(readings.acc_x < acc_x_min) acc_x_min = readings.acc_x;
    if(readings.acc_y > acc_y_max) acc_y_max = readings.acc_y;
    if(readings.acc_y < acc_y_min) acc_y_min = readings.acc_y;
    if(readings.acc_z > acc_z_max) acc_z_max = readings.acc_z;
    if(readings.acc_z < acc_z_min) acc_z_min = readings.acc_z;

    acc_x_diff = acc_x_max - acc_x_min;
    acc_y_diff = acc_y_max - acc_y_min;
    acc_z_diff = acc_z_max - acc_z_min;

    if(mag_readings.mag_x > mag_x_max) mag_x_max = mag_readings.mag_x;
    if(mag_readings.mag_x < mag_x_min) mag_x_min = mag_readings.mag_x;
    if(mag_readings.mag_y > mag_y_max) mag_y_max = mag_readings.mag_y;
    if(mag_readings.mag_y < mag_y_min) mag_y_min = mag_readings.mag_y;
    if(mag_readings.mag_z > mag_z_max) mag_z_max = mag_readings.mag_z;
    if(mag_readings.mag_z < mag_z_min) mag_z_min = mag_readings.mag_z;

    if((x_done == false) && (acc_x_diff >= 2000.0f)) {
      chprintf(chp, "x-axis done\n");
      x_done = true;
    }

    if((y_done == false) && (acc_y_diff >= 2000.0f)) {
      chprintf(chp, "y-axis done\n");
      y_done = true;
    }

    if((z_done == false) && (acc_z_diff >= 2000.0f)) {
      chprintf(chp, "z-axis done\n");
      z_done = true;
    }

  }

  chprintf(
    chp,
    "mag_x_max = %3.2f\tmag_x_min = %3.2f\n"
    "mag_y_max = %3.2f\tmag_y_min = %3.2f\n"
    "mag_z_max = %3.2f\tmag_z_min = %3.2f\n"
    "mag_x_offset = %3.2f\n"
    "mag_y_offset = %3.2f\n"
    "mag_z_offset = %3.2f\n",
    mag_x_max, mag_x_min,
    mag_y_max, mag_y_min,
    mag_z_max, mag_z_min,
    (mag_x_max + mag_x_min) / 2.0f,
    (mag_y_max + mag_y_min) / 2.0f,
    (mag_z_max + mag_z_min) / 2.0f);

  (void)iis2mdcCalibrate(
    &iis2mdc,
    (mag_x_max + mag_x_min) / 2.0f,
    (mag_y_max + mag_y_min) / 2.0f,
    (mag_z_max + mag_z_min) / 2.0f);
}

static const ShellCommand shellcmds[] =
{
  {"csv", csv},
  {"mag_calibrate", mag_calibrate},
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

      /* euler angle measurements */
      /* TODO: make library for this */
      static bool first_reading = true;

      if(first_reading) {
        euler_angles[0] = atan2f(readings.acc_x, readings.acc_z) * 180.0f / M_PI; /* roll */
        euler_angles[1] = atan2f(readings.acc_y, readings.acc_z) * 180.0f / M_PI; /* pitch */
      } else {
        euler_angles[0] += readings.gyro_y * 10.0f; /* roll */
        euler_angles[1] += readings.gyro_x * 10.0f; /* pitch */

        float roll_acc = atan2f(readings.acc_x, readings.acc_z) * 180.0f / M_PI;
        float pitch_acc = atan2f(readings.acc_y, readings.acc_z) * 180.0f / M_PI;

        euler_angles[0] = (euler_angles[0] * 0.98f) + (roll_acc * 0.02f);
        euler_angles[1] = (euler_angles[1] * 0.98f) + (pitch_acc * 0.02f);
      }

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