/**
 * \file   main.c
 * \author Mav Cuyugan
 * \brief  Main app point of entry
 **/

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#include "pinconf.h"

#define SHELL_WORKING_AREA_SIZE THD_WORKING_AREA_SIZE(2048)

static void csv(BaseSequentialStream* chp, int argc, char* argv[])
{
  (void)argc;
  (void)argv;
  chprintf(chp, "%s\r\n", "Test");
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

int main(void) {

  /* initialize system */
  halInit();
  chSysInit();

  /* start serial */
  palSetPadMode(UART_TX_PORT, UART_TX_PADNUM, PAL_MODE_ALTERNATE(UART_PIN_ALTMODE));
  palSetPadMode(UART_RX_PORT, UART_RX_PADNUM, PAL_MODE_ALTERNATE(UART_PIN_ALTMODE));
  sdStart(&SD4, NULL);

  shellInit();

  while (1) {
    thread_t* shelltp = chThdCreateFromHeap(
      NULL,
      SHELL_WORKING_AREA_SIZE,
      "shell",
      NORMALPRIO,
      shellThread,
      (void*)&shellcfg);

    chThdWait(shelltp);
    chThdSleepMilliseconds(1000);
  }

  return 0;
}