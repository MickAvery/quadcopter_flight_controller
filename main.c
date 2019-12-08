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

#include "imu_engine.h"
#include "radio_tx_rx.h"
#include "motor_driver.h"
#include "main_controller.h"
#include "pinconf.h"

#define SHELL_WORKING_AREA_SIZE THD_WORKING_AREA_SIZE(2048)

static void tsv(BaseSequentialStream* chp, int argc, char* argv[])
{
  (void)argc;
  (void)argv;

  while(true) {
    if(((SerialDriver*)chp)->vmt->gett(chp, 100) == 3) {
      break;
    }

    float accel[IMU_DATA_AXES] = {0.0f};
    float gyro[IMU_DATA_AXES] = {0.0f};
    float mag[IMU_DATA_AXES] = {0.0f};
    float euler[IMU_DATA_AXES] = {0.0f};

    size_t x = IMU_DATA_X;
    size_t y = IMU_DATA_Y;
    size_t z = IMU_DATA_Z;

    imuEngineGetData(&IMU_ENGINE, accel, IMU_ENGINE_ACCEL);
    imuEngineGetData(&IMU_ENGINE, gyro, IMU_ENGINE_GYRO);
    imuEngineGetData(&IMU_ENGINE, mag, IMU_ENGINE_MAG);
    imuEngineGetData(&IMU_ENGINE, euler, IMU_ENGINE_EULER);

    chprintf(
      chp,
      "%4.1f\t%4.1f\t%4.1f\t"
      "%2.1f\t%2.1f\t%2.1f\t"
      "%3.1f\t%3.1f\t%3.1f\t"
      "%4.1f\t%4.1f\t%4.1f\n",
      gyro[x] / 1000.0f, gyro[y] / 1000.0f, gyro[z] / 1000.0f,
      accel[x] / 1000.0f, accel[y] / 1000.0f, accel[z] / 1000.0f,
      mag[x] / 1000.0f, mag[y] / 1000.0f, mag[z] / 1000.0f,
      euler[x], euler[y], euler[z]);

    chThdSleepMilliseconds(3);
  }
}

static void ppm_printout(BaseSequentialStream* chp, int argc, char* argv[])
{
  (void)argc;
  (void)argv;

  while(true) {

    /* return if escape key ^C is pressed */
    if(((SerialDriver*)chp)->vmt->gett(chp, 100) == 3) {
      return;
    }

    uint32_t channels[RADIO_TXRX_CHANNELS] = {0U};

    radioTxRxReadInputs(&RADIO_TXRX, channels);

    for(size_t i = 0U ; i < RADIO_TXRX_CHANNELS ; i++) {
      chprintf(chp, "%5u", channels[i]);

      if(i + 1 < RADIO_TXRX_CHANNELS) {
        chprintf(chp, "\t");
      } else {
        chprintf(chp, "\n");
      }
    }
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

  float acc_max[IMU_DATA_AXES] = {0.0f};
  float acc_min[IMU_DATA_AXES] = {0.0f};

  float mag_max[IMU_DATA_AXES] = {0.0f}; /* road warrior! */
  float mag_min[IMU_DATA_AXES] = {0.0f};
  float mag_offset[IMU_DATA_AXES] = {0.0f};

  float acc_x_diff = 0.0f;
  float acc_y_diff = 0.0f;
  float acc_z_diff = 0.0f;

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

    float acc[IMU_DATA_AXES] = {0.0f};
    float mag[IMU_DATA_AXES] = {0.0f};

    imuEngineGetData(&IMU_ENGINE, acc, IMU_ENGINE_ACCEL);
    imuEngineGetData(&IMU_ENGINE, mag, IMU_ENGINE_MAG);

    /* determine max and min from data so far */
    for(size_t i = 0U ; i < IMU_DATA_AXES ; i++) {
      acc_max[i] = fmax(acc_max[i], acc[i]);
      acc_min[i] = fmin(acc_min[i], acc[i]);

      mag_max[i] = fmax(mag_max[i], mag[i]);
      mag_min[i] = fmin(mag_min[i], mag[i]);
    }

    /* get delta between acc max and min */
    acc_x_diff = acc_max[IMU_DATA_X] - acc_min[IMU_DATA_X];
    acc_y_diff = acc_max[IMU_DATA_Y] - acc_min[IMU_DATA_Y];
    acc_z_diff = acc_max[IMU_DATA_Z] - acc_min[IMU_DATA_Z];

    /**
     * completion is determined if accel data has reached the extremes
     * on each axis (assuming that the extreme is 1G on each end, 1G - (-1G) = 2G)
     */
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

  /* calculate mag offset from determined max and min values */
  for(size_t i = 0U ; i < IMU_DATA_AXES ; i++) {
    mag_offset[i] = (mag_max[i] + mag_min[i]) / 2.0f;
  }

  chprintf(
    chp,
    "mag_x_max = %3.2f\tmag_x_min = %3.2f\n"
    "mag_y_max = %3.2f\tmag_y_min = %3.2f\n"
    "mag_z_max = %3.2f\tmag_z_min = %3.2f\n"
    "mag_x_offset = %3.2f\n"
    "mag_y_offset = %3.2f\n"
    "mag_z_offset = %3.2f\n",
    mag_max[IMU_DATA_X], mag_min[IMU_DATA_X],
    mag_max[IMU_DATA_Y], mag_min[IMU_DATA_Y],
    mag_max[IMU_DATA_Z], mag_min[IMU_DATA_Z],
    mag_offset[IMU_DATA_X],
    mag_offset[IMU_DATA_Y],
    mag_offset[IMU_DATA_Z]);

  if(imuEngineMagCalibrate(&IMU_ENGINE, mag_offset) != IMU_ENGINE_OK) {
    chprintf(chp, "failed to calibrate magnetometer\n");
  } else {
    chprintf(chp, "successfully calibrated magnetometer\n");
  }
}

static const ShellCommand shellcmds[] =
{
  {"tsv", tsv},
  {"ppm", ppm_printout},
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

  /* start IMU Engine */
  imuEngineInit(&IMU_ENGINE);
  imuEngineStart(&IMU_ENGINE);

  /* start Radio Transceiver Input Capture */
  radioTxRxInit(&RADIO_TXRX);
  radioTxRxStart(&RADIO_TXRX);

  /* start Motor Driver */
  motorDriverInit(&MOTOR_DRIVER);
  motorDriverStart(&MOTOR_DRIVER);

  /* start Main Controller */
  mainControllerInit(&MAIN_CTRL);
  mainControllerStart(&MAIN_CTRL);

  /* loop for shell thread */
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