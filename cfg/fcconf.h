/**
 * \file   fcconf.h
 * \author Mav Cuyugan (mav.cuyugan@gmail.com)
 * \brief  System-wide flight controller configurations
 */

#ifndef FCCONF_H
#define FCCONF_H

/********************************************************************
 * 
 * IMU Engine Configurations
 * 
 ********************************************************************/

/* IMU Engine sampling rate in Hz */
#define IMU_ENGINE_SAMPLING_RATE 3300

/* Accelerometer configurations */
#define ACCELEROMETER_ODR        LSM6DSL_ACCEL_208_Hz
#define ACCELEROMETER_FULLSCALE  LSM6DSL_ACCEL_8G

/* Gyroscope configurations */
#define GYROSCOPE_ODR            LSM6DSL_GYRO_104_Hz
#define GYROSCOPE_FULLSCALE      LSM6DSL_GYRO_500DPS
#define GYROSCOPE_LPF_EN         false
#define GYROSCOPE_LPF_BW         LSM6DSL_GYRO_LPF_BW_D

/* Magnetometer configurations */
#define MAGNETOMETER_ENABLE      FALSE
#define MAGNETOMETER_ODR         IIS2MDC_ODR_100_Hz

/********************************************************************
 * 
 * PID Configurations
 * 
 ********************************************************************/

/**
 * Minimum throttle value for PID loops to start.
 * It's a percentage value scaled by x100,
 * i.e. 200 = 2%, 5000 = 50%, 10000 = 100%
 */
#define THROTTLE_MIN  200

/**
 * PID frequency for all loops in Hz
 */
#define PID_FREQUENCY 1000

/**
 * Constrain PID sum limit when driving motors
 */
#define PID_SUM_LIMIT 500

/**
 * Max allowable body tilt angle about roll and pitch in degrees
 */
#define BODY_TILT_MAX 45

/**
 * Roll PID configurations
 */
#define PID_ROLL_KP        53
#define PID_ROLL_KI        45
#define PID_ROLL_KD        52
#define PID_ROLL_KF        0
#define PID_ROLL_ITERM_MAX 400

/**
 * Yaw PID configurations
 */
#define PID_PITCH_KP        53
#define PID_PITCH_KI        45
#define PID_PITCH_KD        52
#define PID_PITCH_KF        0
#define PID_PITCH_ITERM_MAX 400

/**
 * Yaw PID configurations
 */
#define PID_YAW_KP        64
#define PID_YAW_KI        0
#define PID_YAW_KD        18
#define PID_YAW_KF        0
#define PID_YAW_ITERM_MAX 400

/**
 * RC rates configurations
 */
#define RC_EXPO             0
#define RC_RATE             80
#define SUPER_RATE          65
#define RC_RATE_INCREMENTAL 1454

#endif /* FCCONF_H */