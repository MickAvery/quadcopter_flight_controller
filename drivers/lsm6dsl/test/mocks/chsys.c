/**
 * \file   chsys.c
 * \author Mav Cuyugan
 * \brief  chsys.c mocks
 */

#include "ch.h"

void chSysHalt(const char* reason) {
  (void)reason;

  while(true) { /* infinite loop */ }
}