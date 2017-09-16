/**
 * \file lsm6dsl_reg.h
 *
 * \author Mav Cuyugan
 * \brief  Register addresses and descriptions for LSM6DSL 6-axis IMU
 **/

#ifndef _LSM6DSL_REG_H
#define _LSM6DSL_REG_H

/**
 * /brief Enable embedded functions register (Read/Write)
 **/
#define FUNC_CFG_ACCESS_ADDR 0x01U /*<! FUNC_CFG_ACCESS register address */
#define FUNC_CFG_EN          0x80U /*<! Enable access to embedded functions configuration
                                        registers bank A and B (default : 0) */
#define FUNC_CFG_B           0x20U /*<! Enable access to embedded functions configuration
                                        register bank B (default : 0) */

/**
 * \brief Sensor synchronization timeframe register (Read/Write)
 **/
#define SENSOR_SYNC_TIME_FRAME_ADDR 0x04U /*<! SENSOR_SYNC_TIME_FRAME register address */
#define SENSOR_SYNC_TIME_FRAME_TPH  0x0FU /*<! TPH_[3:0] bits - sensor synchronization
                                               timeframe with step of 500ms and full range of
                                               5s (default : 4b'0000)*/
#define SENSOR_SYNC_TIME_FRAME_TPH3 0x08U /*<! TPH_[3] */
#define SENSOR_SYNC_TIME_FRAME_TPH2 0x04U /*<! TPH_[2] */
#define SENSOR_SYNC_TIME_FRAME_TPH1 0x02U /*<! TPH_[1] */
#define SENSOR_SYNC_TIME_FRAME_TPH0 0x01U /*<! TPH_[0] */

/**
 * \brief Sensor synchronization resolution ratio (Read/Write)
 *
 * RR_[1:0] :
 * 00 - SensorSync, Res_Ratio = 2-11
 * 01 - SensorSync, Res_Ratio = 2-12
 * 10 - SensorSync, Res_Ratio = 2-13
 * 11 - SensorSync, Res_Ratio = 2-14
 **/
#define SENSOR_SYNC_RES_RATIO_ADDR 0x05U /*<! SENSOR_SYNC_RES_RATIO register address */
#define SENSOR_SYNC_RES_RATIO_RR   0X03U /*<! RR_[1:0] bits - resolution ratio of error
                                              code for sensor synchronization */
#define SENSOR_SYNC_RES_RATIO_RR1  0x02U /*<! RR_[1] */
#define SENSOR_SYNC_RES_RATIO_RR0  0x01U /*<! RR_[0] */

/**
 * \brief FIFO control register #1 (Read/Write)
 *
 * Minimum resolution for FIFO is 1 LSB = 2 bytes (1 word) in FIFO
 **/
#define FIFO_CTRL1_ADDR 0x06U /*<! FIFO_CTRL1 register address */
#define FIFO_CTRL1_FTH  0xFFU /*<! FTH_[7:0] - FIFO threshold level setting
                                   (default : 8'b00000000) */
#define FIFO_CTRL1_FTH7 0x80U /*<! FTH_[7] */
#define FIFO_CTRL1_FTH6 0x40U /*<! FTH_[6] */
#define FIFO_CTRL1_FTH5 0x20U /*<! FTH_[5] */
#define FIFO_CTRL1_FTH4 0x10U /*<! FTH_[4] */
#define FIFO_CTRL1_FTH3 0x08U /*<! FTH_[3] */
#define FIFO_CTRL1_FTH2 0x04U /*<! FTH_[2] */
#define FIFO_CTRL1_FTH1 0x02U /*<! FTH_[1] */
#define FIFO_CTRL1_FTH0 0x01U /*<! FTH_[0] */

/**
 * \brief FIFO control register #2 (Read/Write)
 *
 * TIMER_PEDO_FIFO_EN :
 * 0 - enable write on FIFO based on XL/Gyro data-ready
 * 1 - enable write on FIFO at every step detected by step counter
 **/
#define FIFO_CTRL2_ADDR      0x07U /*<! FIFO_CTRL2 register address */
#define FIFO_CTRL2_FTH       0x07U /*<! FTH_[10:8] - FIFO threshold level setting (default : 3'b000) */
#define FIFO_CTRL2_FTH10     0x04U /*<! FTH_[10] */
#define FIFO_CTRL2_FTH9      0x02U /*<! FTH_[9] */
#define FIFO_CTRL2_FTH8      0x01U /*<! FTH_[8] */
#define TIMER_PEDO_FIFO_EN   0x80U /*<! Enable pedometer step counter and timestamp as 4th FIFO
                                        data set (default : 0) */
#define TIMER_PEDO_FIFO_DRDY 0x40U /*<! FIFO write mode (default : 0) */
#define FIFO_TEMP_EN         0x08U /*<! Enable temperature data storage in FIFO (default : 0) */

#define FIFO_THRESHOLD_MASK                      0x07FFU                      /*<! limit FIFO threshold to 10-bits long */
#define FIFO_CTRL1_THRESHOLD_GET_BITS(threshold) (threshold & 0x00FFU)        /*<! first 8 bits of threshold, goes to FIFO_CTRL1 */
#define FIFO_CTRL2_THRESHOLD_GET_BITS(threshold) ((threshold & 0x0700U) >> 8) /*<! last 3 bits of threshold, goes to FIFO_CTRL2 */
#define FIFO_ENABLE_TIMESTAMP()                  TIMER_PEDO_FIFO_EN

/**
 * \brief FIFO control register #3 (Read/Write)
 *
 * DEC_FIFO_GYRO[2:0] / DEC_FIFO_XL[2:0] -
 * 000 - Gyro/Accelerometer sensor not in FIFO
 * 001 - No decimation
 * 010 - Decimation with factor 2
 * 011 - Decimation with factor 3
 * 100 - Decimation with factor 4
 * 101 - Decimation with factor 8
 * 110 - Decimation with factor 16
 * 111 - Decimation with factor 32
 **/
#define FIFO_CTRL3_ADDR 0x08U /*<! FIFO_CTRL3 register address */
#define DEC_FIFO_GYRO   0x38U /*<! [2:0] - Gyro FIFO (first data set) decimation setting
                                   (default : 3'b000) */
#define DEC_FIFO_GYRO2  0x20U /*<! DEC_FIFO_GYRO[2] */
#define DEC_FIFO_GYRO1  0x10U /*<! DEC_FIFO_GYRO[1] */
#define DEC_FIFO_GYRO0  0x08U /*<! DEC_GYRO_FIFO[0] */
#define DEC_FIFO_XL     0x07U /*<! [2:0] - Accelerometer FIFO (second data set) decimation setting
                                   (default : 3'b000) */
#define DEC_FIFO_XL2    0x04U /*<! DEC_FIFO_XL[2] */
#define DEC_FIFO_XL1    0x02U /*<! DEC_FIFO_XL[1] */
#define DEC_FIFO_XL0    0x01U /*<! DEC_FIFO_XL[0] */

#define FIFO_GYRO_NO_DECIMATION 0x08U
#define FIFO_XL_NO_DECIMATION   0x01U

/**
 * \brief FIFO control register #4 (Read/Write)
 *
 * STOP_ON_FTH :
 * 0 - FIFO depth is unlimited
 * 1 - FIFO depth is limited to threshold level
 *
 * ONLY_HIGH_DATA :
 * 0 - disable MSByte only memorization in FIFO for XL and Gyro
 * 1 - enable MSByte only memorization in FIFO for XL and Gyro
 *
 * DEC_DSx_FIFO[2:0] :
 * 000 - X-th data set not in FIFO
 * 001 - No decimation
 * 010 - Decimation with factor 2
 * 011 - Decimation with factor 3
 * 100 - Decimation with factor 4
 * 101 - Decimation with factor 8
 * 110 - Decimation with factor 16
 * 111 - Decimation with factor 32
 **/
#define FIFO_CTRL4_ADDR 0x09U /*<! FIFO_CTRL4 register address */
#define STOP_ON_FTH     0x80U /*<! Enable FIFO threshold level use (default : 0) */
#define ONLY_HIGH_DATA  0x40U /*<! 8-bit data storage in FIFO (default : 0) */
#define DEC_DS4_FIFO    0x38U /*<! [2:0] - Fourth FIFO data set decimation level (default : 3'b000) */
#define DEC_DS4_FIFO2   0x20U /*<! DEC_DS4_FIFO[2] */
#define DEC_DS4_FIFO1   0x10U /*<! DEC_DS4_FIFO[1] */
#define DEC_DS4_FIFO0   0x08U /*<! DEC_DS4_FIFO[0] */
#define DEC_DS3_FIFO    0x07U /*<! [2:0] - Third FIFO data set decimation level (default 3'b000) */
#define DEC_DS3_FIFO2   0x04U /*<! DEC_DS3_FIFO[2] */
#define DEC_DS3_FIFO1   0x02U /*<! DEC_DS3_FIFO[1] */
#define DEC_DS3_FIFO0   0x01U /*<! DEC_DS3_FIFO[0] */

#define FIFO_THIRD_SET_NO_DECIMATION  0x01U
#define FIFO_FOURTH_SET_NO_DECIMATION 0x08U

/**
 * \brief FIFO control register #5 (Read/Write)
 *
 * ODR_FIFO[3:0] :
 * 0000 - FIFO disabled
 * 0001 - FIFO ODR is set to 12.5 Hz
 * 0010 - FIFO ODR is set to 26 Hz
 * 0011 - FIFO ODR is set to 52 Hz
 * 0100 - FIFO ODR is set to 104 Hz
 * 0101 - FIFO ODR is set to 208 Hz
 * 0110 - FIFO ODR is set to 416 Hz
 * 0111 - FIFO ODR is set to 833 Hz
 * 1000 - FIFO ODR is set to 1.66 kHz
 * 1001 - FIFO ODR is set to 3.33 kHz
 * 1010 - FIFO ODR is set to 6.66 kHz
 *
 * FIFO_MODE[2:0] :
 * 000 - Bypass mode, FIFO disabled
 * 001 - FIFO mode, stops collecting data when FIFO is full
 * 010 - RESERVED
 * 011 - Continuous mode until trigger is deasserted, then FIFO mode
 * 100 - Bypass mode until trigger is deasserted, then Continuous mode
 * 101 - RESERVED
 * 110 - Continuous mode. If FIFO is full, new samples overwrite older ones
 * 111 - RESERVED
 **/
#define FIFO_CTRL5_ADDR 0x0AU /*<! FIFO_CTRL5 register address */
#define ODR_FIFO        0x78U /*<! ODR_FIFO[3:0] - FIFO ODR selection (default : 4'b0000) */
#define ODR_FIFO3       0x40U /*<! ODR_FIFO[3] */
#define ODR_FIFO2       0x20U /*<! ODR_FIFO[2] */
#define ODR_FIFO1       0x10U /*<! ODR_FIFO[1] */
#define ODR_FIFO0       0x08U /*<! ODR_FIFO[0] */
#define FIFO_MODE       0x07U /*<! FIFO_MODE[2:0] - FIFO mode selection (default : 3'b000) */
#define FIFO_MODE2      0x04U /*<! FIFO_MODE[2] */
#define FIFO_MODE1      0x02U /*<! FIFO_MODE[1] */
#define FIFO_MODE0      0x01U /*<! FIFO_MODE[0] */

#define FIFO_SET_ODR(odr)   ((odr << 3) & ODR_FIFO)
#define FIFO_SET_MODE(mode) (mode & FIFO_MODE)

/**
 * \brief DataReady configuration register (Read/Write)
 *
 * DRDY_PULSED :
 * 0 - DataReady latched mode, returns to 0 only after output data has been read
 * 1 - DataReady pulses mode, pulses are 75 microseconds long
 **/
#define DRDY_PULSE_CFG_G_ADDR 0x0BU /*<! DRDY_PULSE_CFG_G register address */ 
#define DRDY_PULSED           0x80U /*<! Enable pulsed DataReady mode (default : 0) */
#define INT2_WRIST_TILT       0x01U /*<! Enable wrist tilt interrupt on INT2 pad (default : 0) */

#define ENABLE_DRDY_PULSED()  DRDY_PULSED
#define DISABLE_DRDY_PULSED() 0U

/**
 * \brief INT1 pad control register (Read/Write)
 *
 * Each bit in this register enables a signal to be carried through INT1. The pad's output will supply the OR
 * combination of the selected signals.
 **/
#define INT1_CTRL_ADDR     0x0DU /*<! INT1_CTRL register address */
#define INT1_STEP_DETECTOR 0x80U /*<! Pedometer step recognition interrupt enable on INT1 pad (default : 0) */
#define INT1_SIGN_MOT      0x40U /*<! Significant motion interrupt enable on INT1 pad (default : 0) */
#define INT1_FULL_FLAG     0x20U /*<! FIFO full flag interrupt enable on INT1 pad (default : 0) */
#define INT1_FIFO_OVR      0x10U /*<! FIFO overrun interrupt on INT1 pad (default : 0) */
#define INT1_FTH           0x08U /*<! FIFO threshold interrupt on INT1 pad (default : 0) */
#define INT1_BOOT          0x04U /*<! Boot status available on INT1 pad (default : 0) */
#define INT1_DRDY_G        0x02U /*<! Gyroscope data ready on INT1 pad (default : 0) */
#define INT1_DRDY_XL       0x01U /*<! Accelerometer data ready on INT1 pad (default : 0) */

#define ENABLE_DRDY_G_INT1()         INT1_DRDY_G
#define ENABLE_DRDY_XL_INT1()        INT1_DRDY_XL
#define ENABLE_FIFO_THRESHOLD_INT1() INT1_FTH
#define ENABLE_FIFO_THRESHOLD_INT1() INT1_FTH

/**
 * \brief INT2 pad control register (Read/Write)
 *
 * Each bit in this register enables a signal to be carried through INT2. The pad's output will
 * supply the OR combination of the selected signals
 **/
#define INT2_CTRL_ADDR     0x0EU /*<! INT2_CTRL register address */
#define INT2_STEP_DELTA    0x80U /*<! Pedometer step recognition interrupt interrupt on delta time enable on INT2 pad (default : 0) */
#define INT2_STEP_COUNT_OV 0x40U /*<! Step count overflow interrupt enable on INT2 pad (default : 0) */
#define INT2_FULL_FLAG     0x20U /*<! FIFO full flag interrupt enable on INT2 pad (default : 0) */
#define INT2_FIFO_OVR      0x10U /*<! FIFO overrun interrupt on INT2 pad (default : 0) */
#define INT2_FTH           0x08U /*<! FIFO threshold interrupt on INT2 pad (default : 0) */
#define INT2_DRDY_TEMP     0x04U /*<! Temperature data ready interrupt on INT2 pad (default : 0) */
#define INT2_DRDY_G        0x02U /*<! Gyroscope data ready interrupt on INT2 pad (default : 0) */
#define INT2_DRDY_XL       0x01U /*<! Accelerometer data ready interrupt on INT2 pad (default : 0) */

/**
 * \brief WHO_AM_I register (Read)
 *
 * Value of this register is fixed at 0x6AU
 **/
#define LSM6DSL_WHO_AM_I_ADDR 0x0FU /*<! WHO_AM_I register address */
#define LSM6DSL_WHO_AM_I      0x6AU /*<! WHO_AM_I register value */

/**
 * \brief Linear acceleration sensor control register #1 (Read/Write)
 *
 * FS_XL[1:0] :
 * 00 - +/- 2g
 * 01 - +/- 16g
 * 10 - +/- 4g
 * 11 - +/- 8g
 *
 * BW0_XL :
 * 0 - BW @ 1.5kHz
 * 1 - BW @ 400Hz
 **/
#define CTRL1_XL_ADDR 0x10U /*<! CTRL1_XL register address */
#define ODR_XL        0xF0U /*<! ODR_XL[3:0] - Output data rate and power mode selection (default : 4'b0000) */
#define ODR_XL3       0x80U /*<! ODR_XL[3] */
#define ODR_XL2       0x40U /*<! ODR_XL[2] */
#define ODR_XL1       0x20U /*<! ODR_XL[1] */
#define ODR_XL0       0x10U /*<! ODR_XL[0] */
#define FS_XL         0x0CU /*<! FS_XL[1:0] - Accelerometer fullscale selection (default : 2'b00) */
#define FS_XL1        0x08U /*<! FS_XL[1] */
#define FS_XL0        0x04U /*<! FS_XL[0] */
#define LPF1_BW_SEL   0x02U /*<! Accelerometer digital LPF (LPF1) bandwidth selection. For bandwidth selection refer to CTRL8_XL (0x17U) */
#define BW0_XL        0x01U /*<! Accelerometer analog chain bandwidth selection (only for accelerometer ODR >= 1.67kHz) */

#define ODR_XL_SHIFT(bits) ((bits << 4) & ODR_XL)
#define FS_XL_SHIFT(bits)  ((bits << 2) & FS_XL)

/**
 * \brief Angular rate sensor control register # 2 (Read/Write)
 *
 * FS_G[1:0] :
 * 00 - 245dps
 * 01 - 500dps
 * 10 - 1000dps
 * 11 - 2000dps
 **/
#define CTRL2_G_ADDR 0x11U /*<! CTRL2_G register address */
#define ODR_G        0xF0U /*<! ODR_G[1:0] - Gyroscope output data rate selection (default : 4'b0000) */
#define ODR_G3       0x80U /*<! ODR_G[3] */
#define ODR_G2       0x40U /*<! ODR_G[2] */
#define ODR_G1       0x20U /*<! ODR_G[1] */
#define ODR_G0       0x10U /*<! ODR_G[0] */
#define FS_G         0x0CU /*<! FS_G[1:0] - Gyroscope full scale selection (default : 2'b00) */
#define FS_G1        0x08U /*<! FS_G[1] */
#define FS_G0        0x04U /*<! FS_G[0] */
#define FS_125       0x02U /*<! Gyroscope full scale at 125 dps (default : 0) */

#define ODR_G_SHIFT(bits) ((bits << 4) & ODR_G)
#define FS_G_SHIFT(bits)  ((bits << 2) & FS_G)
#define FS_G_125DPS()     FS_125

/**
 * \brief Control register # 3 (Read/Write)
 *
 * BOOT :
 * 0 - normal mode
 * 1 - reboot memory content
 *
 * BDU :
 * 0 - continuous update
 * 1 - output registers not updated until MSB and LSB have been read
 *
 * H_LACTIVE :
 * 0 - interrupt output pads active high
 * 1 - interrupt output pads active low
 *
 * PP_OD :
 * 0 - push-pull mode
 * 1 - open-drain mode
 *
 * SIM :
 * 0 - 4-wire interface
 * 1 - 3-wire interface
 *
 * BLE :
 * 0 - data LSB @ lower address
 * 1 - data MSB @ lower address
 *
 * SW_RESET :
 * 0 - normal mode
 * 1 - reset device
 **/
#define CTRL3_C_ADDR 0x12U /*<! CTRL3_C register address */
#define BOOT         0x80U /*<! Reboot memory content (default : 0) */
#define BDU          0x40U /*<! Block data update (default : 0) */
#define H_LACTIVE    0x20U /*<! Interrupt activation level (default : 0) */
#define PP_OD        0x10U /*<! Push-pull/open-drain selection on INT1 and INT2 pads (default : 0) */
#define SIM          0x08U /*<! SPI serial interface mode selection (default : 0) */
#define IF_INC       0x04U /*<! Register address automatically incremented during multiple byte access with a serial interface (default : 1) */
#define BLE          0x02U /*<! Big/Little endian data selection (default : 0) */
#define SW_RESET     0x01U /*<! Software reset (default : 0) */

/**
 * \brief Control register # 4 (Read/Write)
 *
 * INT2_ON_INT1 :
 * 0 - interrupt signals divided between INT1 and INT2
 * 1 - all interrupt signals in logic or on INT1 pad
 *
 * DRDY_MASK :
 * 0 - DA timer disabled
 * 1 - DA timer enabled
 **/
#define CTRL4_C_ADDR  0x13U /*<! CTRL4_C register address */
#define DEN_XL_EN     0x80U /*<! Extend DEN functionality to accelerometer sensor (default : 0) */
#define SLEEP         0x40U /*<! Gyroscope sleep mode enable (default : 0) */
#define DEN_DRDY_INT1 0x20U /*<! DEN DRDY signal on INT1 pad (default : 0) */
#define INT2_ON_INT1  0x10U /*<! All interrupt signals available on INT1 pad enable (default : 0) */
#define DRDY_MASK     0x08U /*<! Configuration 1 data available enable bit (default : 0) */
#define I2C_DISABLE   0x04U /*<! Disable I2C interface (default : 0) */
#define LPF1_SEL_G    0x02U /*<! Enable gyroscope digital LPF1, bandwidth can be selected through
                                 FTYPE[1:0] on CTRL6_C (0x15U) (default : 0) */

/**
 * \brief Control register # 5 (Read/Write)
 *
 * ROUNDING[2:0] :
 * 000 - no rounding
 * 001 - accelerometer only
 * 010 - gyroscope only
 * 011 - gyroscope + accelerometer
 * 100 - registers from SENSORHUB1_REG (0x2EU) to SENSORHUB6_REG (0x33U) only
 * 101 - accelerometer + registers from SENSORHUB1_REG (0x2EU) to SENSORHUB6_REG (0x33U)
 * 110 - gyroscope + accelerometer + registers from SENSORHUB1_REG (0x2EU) to SENSORHUB12_REG (0x39U)
 * 111 - gyroscope + accelerometer + registers from SENSORHUB1_REG (0x2EU) to SENSORHUB6_REG (0x33U)
 *
 * ST_G[1:0] :
 * 00 - normal mode
 * 01 - positive sign self-test
 * 10 - NOT ALLOWED
 * 11 - negative sign self-test
 *
 * ST_XL[1:0] :
 * 00 - normal mode
 * 01 - positive sign self-test
 * 10 - negative sign self-test
 * 11 - NOT ALLOWED
 **/
#define CTRL5_C_ADDR 0x14U /*<! CTRL5_C register address */
#define ROUNDING     0xE0U /*<! ROUNDING[2:0] - Circular burst-mode (rounding) read from output registers (default : 3'b000) */
#define ROUNDING2    0x80U /*<! ROUNDING[] */
#define ROUNDING1    0x40U /*<! ROUNDING[] */
#define ROUNDING0    0x20U /*<! ROUNDING[] */
#define DEN_LH       0x10U /*<! DEN active level configuration (default : 0) */
#define ST_G         0x0CU /*<! ST_G[1:0] - Angular rate sensor self-test enable (default : 2'b00) */
#define ST_G1        0x08U /*<! ST_G[1] */
#define ST_G0        0x04U /*<! ST_G[0] */
#define ST_XL        0x03U /*<! ST_XL[1:0] - Linear acceleration sensor self-test enable (default : 2'b00) */
#define ST_XL1       0x02U /*<! ST_XL[1] */
#define ST_XL0       0x01U /*<! ST_XL[0] */

/**
 * \brief Angular rate sensor control register # 6 (Read/Write)
 *
 * { TRIG_EN, LVL_EN, LVL2_EN } :
 * 100 - edge-sensitive trigger mode is selected
 * 010 - level-sensitive trigger mode is selected
 * 011 - level-sensitive latched mode is selected
 * 110 - level-sensitive FIFO enable mode is selected
 *
 * USR_OFF_W :
 * 0 - 2^(-10) g/LSB
 * 1 - 2^(-6) g/LSB
 **/
#define CTRL6_C_ADDR 0x15U /*<! CTRL6_C register address */
#define TRIG_EN      0x80U /*<! DEN data edge-sensitive trigger enable */
#define LVL_EN       0x40U /*<! DEN data level-sensitive trigger enable */
#define LVL2_EN      0x20U /*<! DEN level-sensitive latched enable */
#define XL_HM_MODE   0x10U /*<! High-performance operating mode disable for accelerometer (default : 0) */
#define USR_OFF_W    0x08U /*<! Weight of XL user offset bits of register X_OFS_USR (0x73U), Y_OFS_USR (0x74U), Z_OFS_USR (0x75U) */
#define FTYPE        0x03U /*<! FTYPE[1:0] - Gyroscope LPF1 bandwidth selection */

/**
 * \brief Linear acceleration sensor control register # 8 (Read/Write)
 *
 * INPUT_COMPOSITE :
 * 0 - ODR/2 low pass filtered sent to composite filter
 * 1 - ODR/4 low pass filtered sent to composite filter
 **/
#define CTRL8_XL_ADDR   0x17U /*<! CTRL8_XL register address */
#define LPF2_XL_EN      0x80U /*<! Accelerometer low-pass filter LPF2 selection */
#define HPCF_XL         0x60U /*<! Accelerometer LPF2 and high-pass filter configuration and cutoff setting */
#define HPCF_XL1        0x40U /*<! HPCF_XL[1] */
#define HPCF_XL0        0x40U /*<! HPCF_XL[0] */
#define HP_REF_MODE     0x10U /*<! Enable HP filter reference mode (default : 0) */
#define INPUT_COMPOSITE 0x08U /*<! Composite filter input selection (default : 0) */
#define HP_SLOPE_XL_EN  0x04U /*<! Accelerometer slope filter / high-pass filter selection */
#define LOW_PASS_ON_6D  0x01U /*<! LPF2 on 6D function selection */

/**
 * \brief Linear acceleration sensor control register # 9 (Read/Write)
 *
 * DEN_XL_G :
 * 0 - DEN info stamped in the gyroscope axis selected by bits [7:5]
 * 1 - DEN info stamped in the accelerometer axis selected by bits [7:5]
 **/
#define CTRL9_XL_ADDR 0x18U /*<! CTRL9_XL register address */
#define DEN_X         0x80U /*<! DEN value stored in LSB of X-axis (default : 0) */
#define DEN_Y         0x40U /*<! DEN value stored in LSB of Y-axis (default : 0) */
#define DEN_Z         0x20U /*<! DEN value stored in LSB of Z-axis (default : 0) */
#define DEN_XL_G      0x10U /*<! DEN stamping sensor selection (default : 0) */
#define SOFT_EN       0x04U /*<! Enable soft-ironing correction algorithm for magnetometer (default : 0) */

/**
 * \brief Control register # 10 (Read/Write)
 **/
#define CTRL10_C_ADDR  0x19U /*<! CTRL10_C register address */
#define WRIST_TILT_EN  0x80U /*<! Enable wrist tilt algorithm (default : 0) */
#define TIMER_EN       0x20U /*<! Enable timestamp count. The count is saved in TIMESTAMP0_REG(0x40U) - TIMESTAMP2_REG(0x42U) (default : 0) */
#define PEDO_EN        0x10U /*<! Enable pedometer algorithm (default : 0) */
#define TILT_EN        0x08U /*<! Enable tilt calculation */
#define FUNC_EN        0x04U /*<! Enable embedded functionalities (pedometer, tilt, significant motion detection, sensorhub, and ironing) (default : 0) */
#define PEDO_RTS_STEP  0x02U /*<! Reset pedometer step counter (default : 0) */
#define SIGN_MOTION_EN 0x01U /*<! Enable significant motion detection function (default : 0) */

/**
 * \brief Master configuration register (Read/Write)
 *
 * DATA_VALID_SEL_FIFO :
 * 0 - data-valid signal used to write data in FIFO is the XL/Gyro data-ready or step detection
 * 1 - data-valid signal used to write data in FIFO is the sensor hub data-ready
 *
 * START_CONFIG :
 * 0 - Sensor hub signal is the XL/Gyro data-ready
 * 1 - Sensor hub signal external from INT2 pad
 **/
#define MASTER_CONFIG_ADDR  0x1AU /*<! MASTER_CONFIG register address */
#define DRDY_ON_INT1        0x80U /*<! Manage the Master DRDY signal on INT1 pad (default : 0) */
#define DATA_VALID_SEL_FIFO 0x40U /*<! Selection of FIFO data-valid signal (default : 0) */
#define START_CONFIG        0x10U /*<! Sensor Hub trigger signal selection (default : 0) */
#define PULL_UP_EN          0x08U /*<! Auxiliary I2C pull-up (default : 0) */
#define PASS_THROUGH_MODE   0x04U /*<! I2C interface pass-through (default : 0) */
#define IRON_EN             0x02U /*<! Enable hard-iron correction algorithm for magnetometer (default : 0) */
#define MASTER_ON           0x01U /*<! Sensor hub I2C master enable (default : 0) */

/**
 * \brief Wake up internal source register (Read)
 *
 * All registers : 
 * 0 - event not detected
 * 1 - detected
 **/
#define WAKE_UP_SRC_ADDR 0x1BU /*<! WAKE_UP_SRC register address */
#define FF_IA            0x20U /*<! Free-fall event detection status (default : 0) */
#define SLEEP_STATE_IA   0x10U /*<! Sleep event status (default : 0) */
#define WU_IA            0x08U /*<! Wakeup event detection status (default : 0) */
#define X_WU             0x04U /*<! Wakeup event detection status on X-axis (default : 0) */
#define Y_WU             0x02U /*<! Wakeup event detection status on Y-axis (default : 0)*/
#define Z_WU             0x01U /*<! Wakeup event detection status on Z-axis (default : 0)*/

/**
 * \brief Tap source register (Read)
 *
 * All registers :
 * 0 - event not detected
 * 1 - detected
 **/
#define TAP_SRC_ADDR 0x1CU /*<! TAP_SRC reigster address */
#define TAP_IA       0x40U /*<! Tap event detection (default : 0) */
#define SINGLE_TAP   0x20U /*<! Single-tap event status (default : 0) */
#define DOUBLE_TAP   0x10U /*<! Double-tap event status (default : 0) */
#define TAP_SIGN     0x08U /*<! Sign of acceleration detected by tap event (default: 0) */
#define X_TAP        0x04U /*<! Tap event detection status on X-axis (default : 0) */
#define Y_TAP        0x02U /*<! Tap event detection status on Y-axis (default : 0) */
#define Z_TAP        0x01U /*<! Tap event detection status on Z-axis (default : 0) */

/**
 * \brief Portrait, landscape, face-up, and face-down source register (Read)
 **/
#define D6D_SRC_ADDR 0x1DU /*<! D6D_SRC register address */
#define DEN_DRDY     0x80U /*<! DEN data-ready signal. It is set high when data output is related to the data coming from a DEN active condition (default : 0) */
#define D6D_IA       0x40U /*<! Interrupt active for change position portrait, landscape, face-up, face-down (default : 0) */
#define ZH           0x20U /*<! Z-axis high event (over threshold) (default : 0) */
#define ZL           0x10U /*<! X-axis low event (under threshold) (default : 0) */
#define YH           0x08U /*<! Y-axis high event (over threshold) (default : 0) */
#define YL           0x04U /*<! Y-axis low event (under threshold) (default : 0) */
#define XH           0x02U /*<! X-axis high event (over threshold) (default : 0) */
#define XL           0x01U /*<! X-axis low event (under threshold) (default : 0) */

/**
 * \brief The STATUS_REG register is read by the I2C/SPI interface (Read)
 **/
#define STATUS_REG_ADDR 0x1EU /*<! STATUS_REG register address */
#define TDA             0x04U /*<! Temperature new data available (default : 0) */
#define GDA             0x02U /*<! Gyroscope new data available (default : 0) */
#define XLDA            0x01U /*<! Accelerometer new data available (default : 0) */

/**
 * \brief Temperature data output registers
 *
 * Temperature value is expressed as two's compliment sign extended on the MSB
 **/
#define OUT_TEMP_L_ADDR 0x20U /*<! OUT_TEMP_L register address, contains TEMP[7:0] */
#define OUT_TEMP_H_ADDR 0x21U /*<! OUT_TEMP_H register address, contains TEMP[15:8] */

/**
 * \brief Angular rate output registers (Read)
 *
 * Pitch - X-axis
 * Roll  - Y-axis
 * Yaw   - Z-axis
 *
 * Value is expressed as 16-bit in two's compliment.
 *
 * Data are according to the full-scale and ODR settings in CTRL2_G (0x11U) of the gyro
 * user interface
 **/
#define OUTX_L_G_ADDR 0x22U /*<! OUTX_L_G register address, contains D[7:0] */
#define OUTX_H_G_ADDR 0x23U /*<! OUTX_H_G register address, contains D[15:8] */
#define OUTY_L_G_ADDR 0x24U /*<! OUTY_L_G register address, contains D[7:0] */
#define OUTY_H_G_ADDR 0x25U /*<! OUTY_H_G register address, contains D[15:8] */
#define OUTZ_L_G_ADDR 0x26U /*<! OUTZ_L_G register address, contains D[7:0] */
#define OUTZ_H_G_ADDR 0x27U /*<! OUTZ_H_G register address, contains D[15:8] */

/**
 * \brief Linear acceeration output registers (Read)
 *
 * Value is expressed as 16-bit word in two's compliment.
 **/
#define OUTX_L_XL_ADDR 0x28U /*<! OUTX_L_XL register address, contains D[7:0] */
#define OUTX_H_XL_ADDR 0x29U /*<! OUTX_H_XL register address, contains D[15:8] */
#define OUTY_L_XL_ADDR 0x2AU /*<! OUTY_L_XL register address, contains D[7:0] */
#define OUTY_H_XL_ADDR 0x2BU /*<! OUTY_H_XL register address, contains D[15:8] */
#define OUTZ_L_XL_ADDR 0x2CU /*<! OUTZ_L_XL register address, contains D[7:0] */
#define OUTZ_H_XL_ADDR 0x2DU /*<! OUTZ_H_XL register address, contains D[15:8] */

/**
 * First byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB1_REG_ADDR 0x2EU /*<! SENSORHUB1_REG register address, contains SHub_1[7:0] */

/**
 * Second byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB2_REG_ADDR 0x2FU /*<! SENSORHUB2_REG register address. contains SHub_2[7:0] */

/**
 * Third byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB3_REG_ADDR 0x30U /*<! SENSORHUB3_REG register address. contains SHub_3[7:0] */

/**
 * Fourth byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB4_REG_ADDR 0x31U /*<! SENSORHUB4_REG register address. contains SHub_4[7:0] */

/**
 * Fifth byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB5_REG_ADDR 0x32U /*<! SENSORHUB5_REG register address. contains SHub_5[7:0] */

/**
 * Sixth byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB6_REG_ADDR 0x33U /*<! SENSORHUB6_REG register address. contains SHub_6[7:0] */

/**
 * Seventh byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB7_REG_ADDR 0x34U /*<! SENSORHUB7_REG register address. contains SHub_7[7:0] */

/**
 * Eighth byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB8_REG_ADDR 0x35U /*<! SENSORHUB8_REG register address. contains SHub_8[7:0] */

/**
 * Ninth byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB9_REG_ADDR 0x36U /*<! SENSORHUB9_REG register address. contains SHub_9[7:0] */

/**
 * Tenth byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB10_REG_ADDR 0x37U /*<! SENSORHUB10_REG register address. contains SHub_10[7:0] */

/**
 * Eleventh byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB11_REG_ADDR 0x38U /*<! SENSORHUB11_REG register address. contains SHub_11[7:0] */

/**
 * Twelfth byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB12_REG_ADDR 0x39U /*<! SENSORHUB12_REG register address. contains SHub_12[7:0] */

/**
 * \brief FIFO status control register # 1 (Read)
 *
 * For proper reading of the register, it is recommended to set the BDU bit in CTRL3_C (0x12U) to 1
 **/
#define FIFO_STATUS1_ADDR 0x3AU /*<! FIFO_STATUS1 register address */
#define DIFF_FIFO_7_0     0xFFU /*<! Number of unread words (16-bit axes) stored in FIFO, contains DIFF_FIFO[7:0] */
#define DIFF_FIFO7        0x80U /*<! DIFF_FIFO[7] */
#define DIFF_FIFO6        0x40U /*<! DIFF_FIFO[6] */
#define DIFF_FIFO5        0x20U /*<! DIFF_FIFO[5] */
#define DIFF_FIFO4        0x10U /*<! DIFF_FIFO[4] */
#define DIFF_FIFO3        0x08U /*<! DIFF_FIFO[3] */
#define DIFF_FIFO2        0x04U /*<! DIFF_FIFO[2] */
#define DIFF_FIFO1        0x02U /*<! DIFF_FIFO[1] */
#define DIFF_FIFO0        0x01U /*<! DIFF_FIFO[0] */

/**
 * \brief FIFO status control register # 2 (Read)
 *
 * For proper reading of the register, it is recommended to set the BDU bit in CTRL3_C (0x12U) to 1.
 *
 * WATER_M :
 * 0 - FIFO filling is lower than watermark level
 * 1 - FIFO filling >= watermark level
 *
 * OVER_RUN :
 * 0 - FIFO not completely filled
 * 1 - FIFO completely filled
 *
 * FIFO_FULL_SMART :
 * 0 - FIFO is not full
 * 1 - FIFO will be full in next ODR
 *
 * FIFO_EMPTY :
 * 0 - FIFO contains data
 * 1 - FIFO is empty
 **/
#define FIFO_STATUS2_ADDR 0x3BU /*<! FIFO_STATUS2 register address */
#define WATER_M           0x80U /*<! FIFO watermark status. Watermark is set through bits FTH[7:0] in FIFO_CTRL1 (0x06U) (default : 0) */
#define OVER_RUN          0x40U /*<! FIFO overrun status (default : 0) */
#define FIFO_FULL_SMART   0x20U /*<! Smart FIFO full status (default : 0) */
#define FIFO_EMPTY        0x10U /*<! FIFO empty bit (default : 0) */
#define DIFF_FIFO_10_8    0x07U /*<! Number of unread words (16-bit axes) stored in FIFO, contains DIFF_FIFO[10:8] */
#define DIFF_FIFO10       0x04U /*<! DIFF_FIFO[10] */
#define DIFF_FIFO9        0x02U /*<! DIFF_FIFO[9] */
#define DIFF_FIFO8        0x01U /*<! DIFF_FIFO[8] */

/**
 * \brief FIFO status control register # 3 (Read)
 *
 * For proper reading of the register, it is recommended to set the BDU bit in CTRL3_C (0x12U) to 1.
 **/
#define FIFO_STATUS3_ADDR 0x3CU /*<! FIFO_STATUS3 register address */
#define FIFO_PATTERN_7_0  0xFFU /*<! Word of recursive pattern read at next reading, contains FIFO_PATTERN[7:0] */
#define FIFO_PATTERN7     0x80U /*<! FIFO_PATTERN[7] */
#define FIFO_PATTERN6     0x40U /*<! FIFO_PATTERN[6] */
#define FIFO_PATTERN5     0x20U /*<! FIFO_PATTERN[5] */
#define FIFO_PATTERN4     0x10U /*<! FIFO_PATTERN[4] */
#define FIFO_PATTERN3     0x08U /*<! FIFO_PATTERN[3] */
#define FIFO_PATTERN2     0x04U /*<! FIFO_PATTERN[2] */
#define FIFO_PATTERN1     0x02U /*<! FIFO_PATTERN[1] */
#define FIFO_PATTERN0     0x01U /*<! FIFO_PATTERN[0] */

/**
 * \brief FIFO status control register # 4 (Read)
 *
 * For proper reading of the register, it is recommended to set the BDU bit in CTRL3_C (0x12U) to 1.
 **/
#define FIFO_STATUS4_ADDR 0x3DU /*<! FIFO_STATUS4 register address */
#define FIFO_PATTERN_9_8  0x03U /*<! Word of recursive pattern read at next reading, contains FIFO_PATTERN [9:8] */
#define FIFO_PATTERN9     0x02U /*<! FIFO_PATTERN[9] */
#define FIFO_PATTERN8     0x01U /*<! FIFO_PATTERN[8] */

/**
 * \brief FIFO data output register, first byte (Read)
 *
 * For proper reading of the register, it is recommended to set the BDU bit in CTRL3_C (0x12U) to 1.
 **/
#define FIFO_DATA_OUT_L_ADDR 0x3EU /*<! FIFO_DATA_OUT_L register address */
#define DATA_OUT_FIFO_L      0xFFU /*<! FIFO data output (first byte) */

/**
 * \brief FIFO data output register, last byte (Read)
 *
 * For proper reading of the register, it is recommended to set the BDU bit in CTRL3_C (0x12U) to 1.
 **/
#define FIFO_DATA_OUT_H_ADDR 0x3FU /*<! FIFO_DATA_OUT_H register address */
#define DATA_OUT_FIFO_H      0xFFU /*<! FIFO data output (last byte) */

/**
 * \brief Timestamp output registers (Read)
 *
 * Timestamp is expressed as a 24-bit word, with resolution defined in WAKE_UP_DUR (0x5CU)
 **/
#define TIMESTAMP0_REG_ADDR 0x40U /*<! TIMESTAMP0_REG register address */
#define TIMESTAMP0          0xFFU /*<! Timestamp first byte data output (LSB) */
#define TIMESTAMP1_REG_ADDR 0x41U /*<! TIMESTAMP1_REG register address */
#define TIMESTAMP1          0xFFU /*<! Timestamp second byte data output */
#define TIMESTAMP2_REG_ADDR 0x42U /*<! TIMESTAMP2_REG register address */
#define TIMESTAMP2          0xFFU /*<! Timestamp third byte data output (MSB) */

/**
 * \brief Step timestamp output registers, contains timestamp of last step detected (Read)
 **/
#define STEP_TIMESTAMP_L_ADDR 0x49U /*<! STEP_TIMESTAMP_L register address, contains LSB of step timestamp */
#define STEP_TIMESTAMP_L      0xFFU /*<! LSB of last step detected */
#define STEP_TIMESTAMP_H_ADDR 0x4AU /*<! STEP_TIMESTAMP_X register address, contains MSB of step timestamp */
#define STEP_TIMESTAMP_L      0xFFU /*<! MSB of last step detected */

/**
 * \brief Step counter output registers (Read)
 **/
#define STEP_COUNTER_L_ADDR 0x4BU /*<! STEP_COUNTER_L register address, contains LSB of step counter */
#define STEP_COUNTER_L      0xFFU /*<! LSB of step counter */
#define STEP_COUNTER_H_ADDR 0x4CU /*<! STEP_COUNTER_H register address, contains MSB of step counter */
#define STEP_COUNTER_H      0xFFU /*<! MSB of step counter */

/**
 * Thirteenth byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB13_REG_ADDR 0x4DU /*<! SENSORHUB13_REG register address, contains SHub_13[7:0] */

/**
 * Fourteenth byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB14_REG_ADDR 0x4EU /*<! SENSORHUB14_REG register address, contains SHub_14[7:0] */

/**
 * Fifteenth byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB15_REG_ADDR 0x4FU /*<! SENSORHUB15_REG register address, contains SHub_15[7:0] */

/**
 * Sixteenth byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB16_REG_ADDR 0x50U /*<! SENSORHUB16_REG register address, contains SHub_16[7:0] */

/**
 * Seventeenth byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB17_REG_ADDR 0x51U /*<! SENSORHUB17_REG register address, contains SHub_17[7:0] */

/**
 * Eigtheenth byte associated to external registers. The content of the register is consistent
 * with the SLAVEx_CONFIG number of read operation configurations (for external sensors
 * from x = 0 to x = 3).
 **/
#define SENSORHUB18_REG_ADDR 0x52U /*<! SENSORHUB18_REG register address, contains SHub_18[7:0] */

/**
 * \brief Significant motion, tilt, step detector, hard/soft-iron and sensor hub interrupt source register (Read)
 *
 * SENSORHUB_END_OP :
 * 0 - sensor hub communication not concluded
 * 1 - sensor hub communication concluded
 **/
#define FUNC_SRC1_ADDR      0x53U /*<! FUNC_SRC1 register address */
#define STEP_COUNT_DELTA_IA 0x80U /*<! Pedometer step recognition on delta time status (default : 0) */
#define SIGN_MOTION_IA      0x40U /*<! Significant motion event detection status (default : 0) */
#define TILT_IA             0x20U /*<! Tilt event detection status (default : 0) */
#define STEP_DETECTED       0x10U /*<! Step detector event detection status (default : 0) */
#define STEP_OVERFLOW       0x08U /*<! Step counter overflow status (default : 0) */
#define HI_FAIL             0x04U /*<! Fail in hard/soft-ironing algorithm */
#define SI_END_OP           0x02U /*<! Hard/soft-iron calculation status (default : 0) */
#define SENSORHUB_END_OP    0x01U /*<! Sensor hub communication status (default : 0) */

/**
 * \brief Wrist tilt interrupt source register (Read)
 **/
#define FUNC_SRC2_ADDR 0x54U /*<! FUNC_SRC2 register address */
#define SLAVE3_NACK    0xXXU /*<! Set to 1 if NACK occurs in slave 3 communication (default : 0) */
#define SLAVE2_NACK    0xXXU /*<! Set to 1 if NACK occurs in slave 2 communication (default : 0  */
#define SLAVE1_NACK    0xXXU /*<! Set to 1 if NACK occurs in slave 1 communication (default : 0  */
#define SLAVE0_NACK    0xXXU /*<! Set to 1 if NACK occurs in slave 0 communication (default : 0  */
#define WRIST_TILT_IA  0x01U /*<! Wrist tilt event detection status (default : 0) */

/**
 * \brief Wrist tilt interrupt source register (Read)
 **/
/* TODO: yet to define this because I'm too lazy */

/**
 * \brief Enables interrupt and inactivity functions, configuration of filtering and tap recognition 
 *        functions (Read/Write)
 **/
/* TODO: yet to define this because I'm too lazy */

/**
 * \brief Portrait/landscape position and tap function threshold register (Read/Write)
 **/
/* TODO: yet to define this because I'm too lazy */

/**
 * \brief Tap recognition function setting register
 **/
/* TODO: yet to define this because I'm too lazy */

/**
 * \brief Tap recognition function setting register (Read/Write)
 **/
/* TODO: yet to define this because I'm too lazy */

/**
 * \brief Free-fall, wakeup, timestamp and sleep mode functions duration setting register
 **/
#define WAKE_UP_DUR_ADDR 0x5CU /*<! WAKE_UP_DUR register address */
#define FF_DUR5          0x80U /*<! Free-fall duration event (default : 0) */
#define WAKE_DUR         0x60U /*<! Wake up duration event (default : 0) */
#define TIMER_HR         0x10U /*<! Timestamp register resolution setting (default : 0) */
#define SLEEP_DUR        0x0FU /*<! Duration to go in sleep mode (default : 4'b0000) */

#define SET_TIMER_RESOLUTION(reso) ((reso << 4) & TIMER_HR)

/**
 * \brief Free-fall function duration setting register
 **/
/* TODO: yet to define this because I'm too lazy */

/**
 * \brief Functions routing on INT1 register (Read/Write)
 **/
#define MD1_CFG_ADDR     0x5EU /*<! MD1_CFG register address */
#define INT1_INACT_STATE 0x80U /*<! Routing on INT1 of inactivity mode (default : 0) */
#define INT1_SINGLE_TAP  0x40U /*<! Single-tap recognition routing on INT1 (default : 0) */
#define INT1_WU          0x20U /*<! Routing of wakeup event on INT1 (default : 0) */
#define INT1_FF          0x10U /*<! Routing of free-fall event on INT1 (default : 0) */
#define INT1_DOUBLE_TAP  0x08U /*<! Routing of tap event on INT1 (default : 0) */
#define INT1_6D          0x04U /*<! Routing of 6D event on INT1 (default : 0) */
#define INT1_TILT        0x02U /*<! Routing of tilt event on INT1 (default : 0) */
#define INT1_TIMER       0x01U /*<! Routing of end counter event of timer on INT1 (default : 0) */

/**
 * \brief Functions routing on INT2 register (Read/Write)
 **/
/* TODO: yet to define this because I'm too lazy */

/**
 * \brief Master command code used for stamping for sensor sync (Read/Write)
 *
 * \note THIS IS NOT AN INPUT REGISTER
 **/
#define MASTER_CMD_CODE_ADDR 0x60U /*<! MASTER_CMD_CODE register address */
#define MASTER_CMD_CODE      0xFFU /*<! Master command code used for stamping for sensor sync (default : 0) */

/**
 * \brief Error code used for sensor synchronization (Read/Write)
 *
 * \note THIS IS NOT AN INPUT REGISTER
 **/
#define SENS_SYNC_SPI_ERROR_CODE_ADDR 0x61U /*<! SENS_SYNC_SPI_ERROR_CODE register address */
#define SENS_SYNC_SPI_ERROR_CODE      0xFFU /*<! Error code used for sensor synchronization (default : 0) */

/**
 * \brief External magnetometer raw data registers (Read)
 **/
#define OUT_MAG_RAW_X_L_ADDR 0x66U /*<! OUT_MAG_RAW_X_L register address, contains LSB of X-axis */
#define OUT_MAG_RAW_X_H_ADDR 0x67U /*<! OUT_MAG_RAW_X_H register address, contains MSB of X-axis */
#define OUT_MAG_RAW_Y_L_ADDR 0x68U /*<! OUT_MAG_RAW_Y_L register address, contains LSB of Y-axis */
#define OUT_MAG_RAW_Y_H_ADDR 0x69U /*<! OUT_MAG_RAW_Y_H register address, contains MSB of Y-axis */
#define OUT_MAG_RAW_Z_L_ADDR 0x6AU /*<! OUT_MAG_RAW_Z_L register address, contains LSB of Z-axis */
#define OUT_MAG_RAW_Z_H_ADDR 0x6BU /*<! OUT_MAG_RAW_Z_H register address, contains MSB of Z-axis */

/**
 * \brief Accelerometer axes user offset correction registers (Read/Write)
 *
 * The offset value set in the [x, y, z]_OFS_USR offset register is internally
 * added to the acceleration value measured on the [x, y, z]-axis
 **/
#define X_OFS_USR_ADDR 0x73U /*<! X_OFS_USR register address */
#define X_OFS_USR      0xFFU /*<! Accelerometer X-axis user offset correction expressed in two’s complement, weight depends on CTRL6_C(4) bit.
                                  The value must be in the range [-127 127] */
#define Y_OFS_USR_ADDR 0x74U /*<! Y_OFS_USR register address */
#define Y_OFS_USR      0xFFU /*<! Accelerometer Y-axis user offset correction expressed in two’s complement, weight depends on CTRL6_C(4) bit.
                                  The value must be in the range [-127 127] */
#define Z_OFS_USR_ADDR 0x75U /*<! Z_OFS_USR register address */
#define Z_OFS_USR      0xFFU /*<! Accelerometer Z-axis user offset correction expressed in two’s complement, weight depends on CTRL6_C(4) bit.
                                  The value must be in the range [-127 127] */

/*************************************************
 * Embedded registers - Bank A
 *
 * NOTE: All modifications of the content of the
 * embedded functions registers have to be performed 
 * with the device in power-down mode.
 *************************************************/

/**
 * \brief I2C slave address of Sensor1 register (Read/Write)
 *
 * RW_0 :
 * 0 - WRITE
 * 1 - READ
 **/
#define SLV0_ADD_ADDR 0x02U /*<! SLV0_ADD register address */
#define SLAVE0_ADD    0xFEU /*<! I2C slave address of Sensor1 that can be read by sensor hub (default : 7'b0000000) */
#define RW_0          0x01U /*<! Read/Write operation on Sensor1 (default : 0) */

/**
 * \brief Address of register on Sensor1 register (Read/Write)
 **/
#define SLV0_SUBADD_ADDR 0x03U /*<! SLV0_SUBADD register address */
#define SLAVE0_REG       0xFFU /*<! Address of register on Sensor1 that has to be read/write
                                    according to the rw_0 bit value in SLV0_ADD (0x02U) (default : 8'b00000000) */

/**
 * \brief Sensor1 configuration and sensor hub settings register (Read/Write)
 **/
#define SLAVE0_CONFIG_ADDR 0x04U /*<! SLAVE0_CONFIG register address */
#define SLAVE0_RATE        0xC0U /*<! Decimation of read operation on Sensor1 starting from the sensor hub trigger (default : 2'b00) */
#define AUX_SENS_ON        0x30U /*<! Number of external sensors to be read by sensor hub (default : 2'b00) */
#define SRC_MODE           0x08U /*<! Source mode conditioned read (default : 0) */
#define SLAVE0_NUMOP       0x07U /*<! Number of read operations on Sensor1 */

#define NUM_EXTERNAL_SENSORS_MASK(num)  ((num << 4) & AUX_SENS_ON)
#define NUM_REGISTERS_TO_READ_MASK(num) (num & SLAVE0_NUMOP)

/**
 * \brief I2C slave address of Sensor2 register (Read/Write)
 *
 * RW_1 :
 * 0 - WRITE
 * 1 - READ
 **/
#define SLV1_ADD_ADDR 0x05U /*<! SLV1_ADD register address */
#define SLAVE1_ADD    0xFDU /*<! I2C slave address of Sensor2 that can be read by sensor hub (default : 7'b0000000) */
#define RW_1          0x01U /*<! Read/Write operation on Sensor2 (default : 0) */

/**
 * \brief Address of register on Sensor2 register (Read/Write)
 **/
#define SLV1_SUBADD_ADDR 0x06U /*<! SLV1_SUBADD register address */
#define SLAVE1_REG       0xFFU /*<! Address of register on Sensor2 that has to be read/write
                                    according to the rw_1 bit value in SLV1_ADD (0x02U) (default : 8'b00000000) */

/**
 * \brief Sensor2 configuration and sensor hub settings register (Read/Write)
 *
 * WRITE_ONCE :
 * 0 - write operation for each sensor hub cycle
 * 1 - write operation only for the first sensor hub cycle
 **/
#define SLAVE1_CONFIG_ADDR 0x07U /*<! SLAVE1_CONFIG register address */
#define SLAVE1_RATE        0xC0U /*<! Decimation of read operation on Sensor2 starting from the sensor hub trigger (default : 2'b00) */
#define WRITE_ONCE         0x20U /*<! Slave 0 write operation is performed only at the first sensor hub cycle (default : 0) */
#define SLAVE1_NUMOP       0x07U /*<! Number of read operations on Sensor2 */

/**
 * \brief I2C slave address of Sensor3 register (Read/Write)
 *
 * RW_2 :
 * 0 - WRITE
 * 1 - READ
 **/
#define SLV2_ADD_ADDR 0x08U /*<! SLV2_ADD register address */
#define SLAVE2_ADD    0xFDU /*<! I2C slave address of Sensor3 that can be read by sensor hub (default : 7'b0000000) */
#define RW_2          0x01U /*<! Read/Write operation on Sensor3 (default : 0) */

/**
 * \brief Address of register on Sensor3 register (Read/Write)
 **/
#define SLV2_SUBADD_ADDR 0x09U /*<! SLV2_SUBADD register address */
#define SLAVE2_REG       0xFFU /*<! Address of register on Sensor3 that has to be read/write
                                    according to the rw_2 bit value in SLV2_ADD (0x02U) (default : 8'b00000000) */

/**
 * \brief Sensor3 configuration and sensor hub settings register (Read/Write)
 */
#define SLAVE2_CONFIG_ADDR 0x0AU /*<! SLAVE2_CONFIG register address */
#define SLAVE2_RATE        0xC0U /*<! Decimation of read operation on Sensor3 starting from the sensor hub trigger (default : 2'b00) */
#define SLAVE2_NUMOP       0x07U /*<! Number of read operations on Sensor3 */

/**
 * \brief I2C slave address of Sensor4 register (Read/Write)
 *
 * RW_3 :
 * 0 - WRITE
 * 1 - READ
 **/
#define SLV3_ADD_ADDR 0x0BU /*<! SLV3_ADD register address */
#define SLAVE3_ADD    0xFDU /*<! I2C slave address of Sensor4 that can be read by sensor hub (default : 7'b0000000) */
#define RW_3          0x01U /*<! Read/Write operation on Sensor4 (default : 0) */

/**
 * \brief Address of register on Sensor4 register (Read/Write)
 **/
#define SLV3_SUBADD_ADDR 0x0CU /*<! SLV3_SUBADD register address */
#define SLAVE3_REG       0xFFU /*<! Address of register on Sensor4 that has to be read/write
                                    according to the rw_3 bit value in SLV3_ADD (0x02U) (default : 8'b00000000) */

/**
 * \brief Sensor4 configuration and sensor hub settings register (Read/Write)
 */
#define SLAVE3_CONFIG_ADDR 0x0DU /*<! SLAVE3_CONFIG register address */
#define SLAVE3_RATE        0xC0U /*<! Decimation of read operation on Sensor4 starting from the sensor hub trigger (default : 2'b00) */
#define SLAVE3_NUMOP       0x07U /*<! Number of read operations on Sensor4 */

/**
 * \brief Data to be written into the slave device register (Read/Write)
 **/
#define DATAWRITE_SRC_MODE_SUB_SLV0_ADDR 0x0EU /*<! DATAWRITE_SRC_MODE_SUB_SLV0 register address */
#define SLAVE_DATAW                      0xFFU /*<! Data to be written into the slave device according to the rw_0 bit in
                                                    SLV0_ADD (0x02U) register or address to be read in source mode */

/* TODO: too lazy to define rest of Bank A registers */
#define CONFIG_PEDO_THS_MIN
#define SM_THS
#define PEDO_DEB_REG
#define STEP_COUNT_DELTA
#define MAG_SI_XX
#define MAG_SI_XY
#define MAG_SI_XZ
#define MAG_SI_YX
#define MAG_SI_YY
#define MAG_SI_YZ
#define MAG_SI_ZX
#define MAG_SI_ZY
#define MAG_SI_ZZ
#define MAG_OFFX_L
#define MAG_OFFX_H
#define MAG_OFFY_L
#define MAG_OFFY_H
#define MAG_OFFZ_L
#define MAG_OFFZ_H

/*************************************************
 * Embedded registers - Bank B
 *
 * NOTE: All modifications of the content of the
 * embedded functions registers have to be performed 
 * with the device in power-down mode.
 *************************************************/

/* TODO: too lazy to define Bank B register definitions */
#define A_WRIST_TILT_LAT_ADDR
#define A_WRIST_TILT_THS_ADDR
#define A_WRIST_TILT_MASK_ADDR

#endif /* _LSM6DSL_REG_H */
