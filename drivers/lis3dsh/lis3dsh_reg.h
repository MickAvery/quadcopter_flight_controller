/**
 * \brief  lis3dsh_reg.h
 * \author Mav Cuyugan
 * \brief  Register definitions for LIS3DSH accelerometer
 **/

#ifndef LIS3DSH_REG_H
#define LIS3DSH_REG_H

/**
 * 8-bit temperature output register. Value is expressed as two's compliment.
 * Resolution is 1 LSB/deg and 0x00U corresponds to 25 degrees Celsius
 * (READ)
 **/
#define OUT_T_ADDR 0x0CU /*<! OUT_T register address */

/**
 * \brief Read-only information register # 2, its value is fixed at 0x21U
 * (READ)
 **/
#define INFO1_ADDR 0x0DU /*<! INFO1 register address */

/**
 * \brief Read-only information register # 2, its value is fixxed at 0x00U
 * (READ)
 **/
#define INFO2_ADDR 0x0EU /*<! INFO2 register address */

/**
 * \brief WHO_AM_I register
 * (READ)
 **/
#define LIS3DSH_WHO_AM_I_ADDR 0x0FU /*<! WHO_AM_I register address */
#define LIS3DSH_WHO_AM_I      0x3FU /*<! WHO_AM_I value */

/**
 * Offset compensation registers, values are expressed in two's complement.
 *
 * OUTPUT(axis) = Measurement(axis) - (Offset(axis) * 32)
 *
 * (READ)
 **/
#define OFF_X_ADDR 0x10U /*<! OFF_X register address */
#define OFF_Y_ADDR 0x11U /*<! OFF_Y register address */
#define OFF_Z_ADDR 0x12U /*<! OFF_Z register address */

/**
 * \brief Control register # 4
 *
 * ODR[3:0] :
 * 0000 - power-down
 * 0001 - 3.125 Hz
 * 0010 - 6.25 Hz
 * 0011 - 12.5 Hz
 * 0100 - 25 Hz
 * 0101 - 50 Hz
 * 0110 - 100 Hz
 * 0111 - 400 Hz
 * 1000 - 800 Hz
 * 1001 - 1600 Hz
 *
 * BDU :
 * 0 - continuous update
 * 1 - output registers not updated until MSB and LSB have been read
 *
 * (READ/WRITE)
 **/
#define CTRL_REG4_ADDR 0x20U /*<! CTRL_REG4 register address */
#define ODR            0xF0U /*<! Data rate selection (default : 4'b0000) */
#define ODR3           0x80U /*<! ODR[3] */
#define ODR2           0x40U /*<! ODR[2] */
#define ODR1           0x20U /*<! ODR[1] */
#define ODR0           0x10U /*<! ODR[0] */
#define BDU            0x08U /*<! Block data update (default : 0) */
#define ZEN            0x04U /*<! Z-axis enable (default : 1) */
#define YEN            0x02U /*<! Y-axis enable (default : 1) */
#define XEN            0x01U /*<! X-axis enable (default : 1) */

#define LIS3DSH_ODR_SHIFT(odr) ((odr << 4) & ODR)

/**
 * \brief Control register # 3
 *
 * IEA :
 * 0 - Interrupt signal active LOW
 * 1 - Interrupt signal active HIGH
 *
 * IEL :
 * 0 - Interrupt latched
 * 1 - Interrupt pulsed
 *
 * STRT :
 * Resets whole internal logic circuitry when set to 1, and automatically return to 0
 *
 * (READ/WRITE)
 **/
#define CTRL_REG3_ADDR 0x23U /*<! CTRL_REG3 register address */
#define DR_EN          0x80U /*<! Data-ready interrupt on INT1 enable (default : 0) */
#define IEA            0x40U /*<! Interrupt signal polarity (default : 0) */
#define IEL            0x20U /*<! Interrupt latched (default : 0) */
#define INT2_EN        0x10U /*<! INT2 signal enable (default : 0) */
#define INT1_EN        0x08U /*<! INT1 signal enable (default : 0) */
#define VFILT          0x04U /*<! Vector filter enable (default : 0) */
#define STRT           0x01U /*<! Soft reset */

/**
 * \brief Control register # 5
 *
 * FSCALE[1:0] :
 * 000 : +/-2g
 * 001 : +/-4g 
 * 010 : +/-6g
 * 011 : +/-8g
 * 100 : +/-16g
 *
 * (READ/WRITE)
 **/
#define CTRL_REG5_ADDR 0x24U /*<! CTRL_REG5 register address */
#define BW             0xC0U /*<! Anti-aliasing filter bandwidth (default : 2'b00) */
#define BW1            0x80U /*<! BW[1] */
#define BW0            0x40U /*<! BW[0] */
#define FSCALE         0x38U /*<! Full-scale selection (default : 3'b000) */
#define FSCALE2        0x20U /*<! FSCALE[2] */
#define FSCALE1        0x10U /*<! FSCALE[1] */
#define FSCALE0        0x08U /*<! FSCALE[0] */
#define ST             0x06U /*<! Self-test enable (default : 2'b00) */
#define ST1            0x04U /*<! ST[1] */
#define ST0            0x02U /*<! ST[0] */
#define SIM            0x01U /*<! SPI serial internal interface mode selection (default : 0) */

#define LIS3DSH_FS_SHIFT(fs) ((fs << 3) & FSCALE)

/**
 * \brief Control register # 6
 * (READ/WRITE)
 **/
#define CTRL_REG6_ADDR 0x25U /*<! CTRL_REG6 register address */
#define BOOT           0x80U /*<! Force reboot */
#define FIFO_EN        0x40U /*<! FIFO enable (default : 0) */
#define WTM_EN         0x20U /*<! Stop on watermark (default : 0) */
#define ADD_INC        0x10U /*<! Register address automatically increased during a multiple byte access with a serial interface (default : 0) */
#define P1_EMPTY       0x08U /*<! Enable FIFO empty indication on INT1 pin (default : 0) */
#define P1_WTM         0x04U /*<! FIFO watermark interrupt on INT1 pin (default : 0) */
#define P1_OVERRUN     0x02U /*<! FIFO overrun interrupt on INT1 pin (default : 0) */
#define P2_REBOOT      0x01U /*<! Boot interrupt on INT2 pin (default : 0) */

/**
 * \brief Status register
 * (READ)
 **/
#define STATUS_ADDR 0x27U /*<! STATUS register address */
#define ZYXOR       0x80U /*<! X-, Y-, and Z-axis data overrrun */
#define ZOR         0x40U /*<! Z-axis overrun */
#define YOR         0x20U /*<! Y-axis overrun */
#define XOR         0x10U /*<! X-axis overrun */
#define ZYXDA       0x08U /*<! X-, Y-, and Z-axis new data available */
#define ZDA         0x04U /*<! Z-axis new data available */
#define YDA         0x02U /*<! Y-axis new data available */
#define XDA         0x01U /*<! X-axis new data available */

/**
 * \brief Output registers
 **/
#define OUT_X_L 0x28U /*<! X-axis LSB */
#define OUT_X_H 0x29U /*<! X-axis MSB */
#define OUT_Y_L 0x2AU /*<! Y-axis LSB */
#define OUT_Y_H 0x2BU /*<! Y-axis MSB */
#define OUT_Z_L 0x2CU /*<! Z-axis LSB */
#define OUT_Z_H 0x2DU /*<! Z-axis MSB */

#endif /* LIS3DSH_REG_H */
