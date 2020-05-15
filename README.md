# Quadcopter Flight Controller
> Name to be determined

[![Build Status](https://travis-ci.org/MickAvery/quadcopter_flight_controller.svg)](https://travis-ci.org/MickAvery/quadcopter_flight_controller)

## Introduction
As a student, what's the best way to get choosy firmware companies to notice you when you're not a nerd with straight-A's? Why, make a custom quadcopter flight controller, of course!

## Wiki
[I highly suggest visiting the wiki that goes more in detail on the components of the quadctoper, as well as how they are tested.](https://github.com/MickAvery/quadcopter_flight_controller/wiki)

## Components
### Hardware
The main flight controller is an **STM32F407 with an ARM Cortex M4 chip**.
The other components of the flight controller are subject to change, so I won't put them here for now.
### Software
The embedded control software is developed using ChibiOS open-source RTOS.

## Build
### Dependencies
1. OpenOCD

`apt/brew install openocd`

2. ARM cross-compiler and debugger

`apt/brew install gcc-arm-none-eabi && apt/brew install gdb-arm-none-eabi`

3. Autoreconf (for building CppUTest)

`apt/brew install automake autoconf libtool`

### After cloning

Clone all submodules with `git submodule update --init --recursive`

### Building CppUTest
```
$ cd <proj_root_dir>/third_party/CppUTest/cpputest_build
$ autoreconf .. -i
$ ../configure
$ make
```

### Building and flashing project
I asume you have to following packages:
1. OpenOCD on-chip debugger
2. ARM cross-compiler and debug tools (I'll make this more specific later on)

Asuming you have the packages above, just run `make` to compile. You'll notice a `build/` folder is made that houses the binary files.
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
