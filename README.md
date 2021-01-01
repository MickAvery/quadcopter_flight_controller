# M.A.V. (multirotor aerial vehicle)
> Quadcopter Flight Controller

[![Build Status](https://travis-ci.org/MickAvery/quadcopter_flight_controller.svg)](https://travis-ci.org/MickAvery/quadcopter_flight_controller)

## Introduction
As a student, what's the best way to get choosy firmware companies to notice you when you're not a nerd with straight-A's? Why, make a custom quadcopter flight controller, of course!

## Wiki
[I suggest visiting the wiki that goes more in detail on the components of the quadctoper, as well as how they are tested.](https://github.com/MickAvery/quadcopter_flight_controller/wiki)

## Components
### Hardware
MCU - [STM32F407 with an ARM Cortex M4 chip](https://www.st.com/en/microcontrollers-microprocessors/stm32f407-417.html)

IMU - [LSM6DSL 6-DOF gyroscope and accelerometer](https://www.st.com/en/mems-and-sensors/lsm6dsl.html/)

RC receiver - Flysky FS-iA6B 2.4GHz 6channel with PPM

RC - Flysky FS-i6

### Software
Multithreaded flight control firmware developed using [ChibiOS RTOS](https://www.chibios.org/dokuwiki/doku.php).

## Required tools
1. [OpenOCD](http://openocd.org/)

2. [GNU ARM cross-compiler and debugger](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)(I'm using 7.2.1)

3. Autoreconf (for building CppUTest)

`apt install automake autoconf libtool`

### Installing third-party libraries

Clone all submodules with `git submodule update --init --recursive`, then run `make install`

## Build
### VS Code (recommended, if only for convenience)

Instructions coming soon.

### No-IDE building

Compile with `make`.

Next, have **two** terminal windows open, and in each one:
1. Run `openocd -f stm32f4.cfg` on one window
2. Run `arm-none-eabi-gdb build/ch.elf`. Here you'll get a prompt much like running regular GDB. Then run the following commands:
    * `tar ext :3333`
    * `mon reset halt`
    * `load`
    * `run`

### Using ChibiStudio for Windows

ChibiOS provides a free Eclipse-based IDE for Windows computers that you can [download here](http://www.chibios.org/dokuwiki/doku.php?id=chibios:product:chibistudio:start). Download and unzip the file into `C:\`. Right-click on **Project Explorer**, go to **Import > C/C++ > Existing Code as Makefile Project**, then point to the codebase location. Build the project, then flash.

For more information on setting up ChibiStudio, visit this [link](https://www.playembedded.org/blog/developing-stm32-chibistudio/).

Instructions incomplete for now, I'll add to this later on. For now, at least you know how to compile the code and load it into the board.
