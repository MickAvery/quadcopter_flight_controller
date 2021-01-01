/**
 * \file   logger.h
 * \author Mav Cuyugan (mav.cuyugan@gmail.com)
 * \brief  System-wide logging module
 */

#ifndef LOGGER_H
#define LOGGER_H

#include "hal.h"
#include "chprintf.h"

#define LOG_DEBUG(...) do{ chprintf((BaseSequentialStream*)&SD4, __VA_ARGS__); } while(0);

#endif /* LOGGER_H */