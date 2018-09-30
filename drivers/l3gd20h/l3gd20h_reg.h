/**
 * \file    l3gd20h_reg.h
 * \author  Mav Cuyugan
 * \brief   Register address and bit definitions for L3GD20H ST MEMS gyroscope.
 *
 * \addtogroup L3GD20H_Registers_Definition
 * \{
 */

#ifndef L3GD20H_REG_H
#define L3GD20H_REG_H

/* Address for WHO_AM_I register */
#define WHO_AM_I_REG_ADDR     0x0FU 

/* Bit definition and address for CTRL1 register */
#define CTRL1_ADDR 0x20U /*<! CTRL1 register address */
#define DR         0xC0U /*<! DR[1:0] bits - output data rate selection */
#define DR1        0x80U /*<! DR[1] */
#define DR0        0x40U /*<! DR[0] */
#define BW         0x30U /*<! BW[1:0] bits - bandwidth selection */
#define BW1        0x20U /*<! BW[1] */
#define BW0        0x10U /*<! BW[0] */
#define PD         0x08U /*<! Power mode ( 1 = Normal Mode ; 0 = Power Down ) */
#define ZEN        0x04U /*<! Z axis enable */
#define YEN        0x02U /*<! Y axis enable */
#define XEN        0x01U /*<! X axis enable */

#define ODR_SHIFT(odr) ((odr << 6) & DR)

/* Bit definition and address for CTRL2 register */
#define CTRL2_REG_ADDR        0x21U /*<! CTRL2 register address */
#define CTRL2_EXTRen          0x80U /*<! Edge sensitive trigger enable */
#define CTRL2_LVLen           0x40U /*<! Level sensitive trigger enable */

#define CTRL2_HPM             0x30U /*<! HPM[1:0] bits - high pass filter mode selection */
#define CTRL2_HPM1            0x20U /*<! Bit 1 */
#define CTRL2_HPM0            0x10U /*<! Bit 0 */

#define CTRL2_HPCF            0x0FU /*<! HPCF[3:0] bits - high pass filter cut off frequency selection */
#define CTRL2_HPCF3           0x08U /*<! Bit 3 */
#define CTRL2_HPCF2           0x04U /*<! Bit 2 */
#define CTRL2_HPCF1           0x02U /*<! Bit 1 */
#define CTRL2_HPCF0           0x01U /*<! Bit 0 */

/* Bit definition and address for CTRL3 register */
#define CTRL3_REG_ADDR        0x22U /*<! CTRL3 register address */
#define CTRL3_INT1_IG         0x80U /*<! Interrupt enable on INT1 pin */
#define CTRL3_INT1_Boot       0x40U /*<! Boot status available on INT1 pin ( 1 = enable ; 0 = disable ) */
#define CTRL3_H_Lactive       0x20U /*<! Interrupt active configuration on INT ( 1 = low ; 0 = high ) */
#define CTRL3_PP_OD           0x10U /*<! Push-pull / Open drain ( 1 = open drain ; 0 = push-pull ) */
#define CTRL3_INT2_DRDY       0x08U /*<! Date ready on DRDY/INT2 pin ( 1 = enable ; 0 = disable ) */
#define CTRL3_INT2_FTH        0x04U /*<! FIFO threshold interrupt on DRDY/INT2 pin (1 = enable ; 0 = disable) */
#define CTRL3_INT2_ORun       0x02U /*<! FIFO overrun interrupt on DRDY/INT2 pin ( 1 = enable ; 0 = disable ) */
#define CTRL3_INT2_Empty      0x01U /*<! FIFO empty interrupt on DRDY/INT2 pin ( 1 = enable ; 0 = disable ) */

/* Bit definition and address for CTRL4 register */
#define CTRL4_ADDR 0x23U /*<! CTRL4 register address */
#define BDU        0x80U /*<! Block data update (1 = output regs not updated until MSB and LSB reading ; 0 = continuous update) */
#define BLE        0x40U /*<! Big/Little endian data selection (1 = Big endian ; 0 = Little endian) */
#define FS         0x30U /*<! FS[1:0] bits - full scale selection */
#define FS1        0x20U /*<! Bit 1 */
#define FS0        0x10U /*<! Bit 0 */
#define IMPen      0x08U /*<! Level-sensitive latch enabled */
#define ST         0x06U /*<! ST[1:0] bits - self-test enable */
#define ST2        0x04U /*<! Bit 1 */
#define ST1        0x02U /*<! Bit 0 */
#define SIM        0x01U /*<! SPI Serial Interface Mode selection (1 = 3-wire interface ; 0 = 4-wire interface) */

#define FS_SHIFT(fs) ((fs << 4) & FS)

/* Bit definition and address for CTRL5 register */
#define CTRL5_REG_ADDR        0x24U /*<! CTRL5 register address */
#define CTRL5_BOOT            0x80U /*<! Reboot memory content (1 = reboot memory content ; 0 = normal mode) */
#define CTRL5_FIFO_EN         0x40U /*<! FIFO enable */
#define CTRL5_StopOnFTH       0x20U /*<! Sensing chain FIFO stop values memorization at FIFO threshold 
                                         1 = FIFO depth is limited to FIFO Threshold which is defined in FIFO_CTRL
                                         0 = FIFO depth is not limited */
#define CTRL5_HPen            0x10U /*<! High-pass filter enable */

#define CTRL5_IG_Sel          0x0CU /*<! IG_Sel[1:0] bits - INT Generator selection configuration */
#define CTRL5_IG_Sel1         0x08U /*<! Bit 1 */
#define CTRL5_IG_Sel0         0x04U /*<! Bit 0 */

#define CTRL5_Out_Sel         0x03U /*<! Out_Sel[1:0] bits - Out selection configuration */
#define CTRL5_Out_Sel1        0x02U /*<! Bit 1 */
#define CTRL5_Out_Sel0        0x01U /*<! Bit 0 */

/* REFERENCE register */
#define REFERENCE_REG_ADDR    0x25U /*<! REFERENCE register address */

/* OUT_TEMP register */
#define OUT_TEMP_REG_ADDR     0x26U /*<! OUT_TEMP register address */

/* STATUS register */
#define STATUS_REG_ADDR       0x27U /*<! STATUS register address */
#define STATUS_REG_X_READY    0X01U /*<! If bit is set, data is available for X axis */
#define STATUS_REG_Y_READY    0x02U /*<! If bit is set, data is available for Y axis */
#define STATUS_REG_Z_READY    0x04U /*<! If bti is set, data is available for Z axis */
#define STATUS_REG_DATA_READY 0x08U /*<! If bit is set, data is available for X Y Z axes */
#define STATUS_REG_X_OVERRUN  0X10U /*<! If bit is set, X axis data has been overwritten */
#define STATUS_REG_Y_OVERRUN  0X20U /*<! If bit is set, Y axis data has been overwritten */
#define STATUS_REG_Z_OVERRUN  0x40U /*<! If bit is set, Z axis data has been overwritten */
#define STATUS_REG_OVERRUN    0x80U /*<! If bit is set, all axes data have been overwritten */

/* OUT_X_L register */
#define OUT_X_L_REG_ADDR      0x28U /*<! OUT_X_L register address */

/* OUT_X_H register */
#define OUT_X_H_REG_ADDR      0x29U /*<! OUT_X_H register address */

/* OUT_Y_L register */
#define OUT_Y_L_REG_ADDR      0x2AU /*<! OUT_Y_L register address */

/* OUT_Y_H register */
#define OUT_Y_H_REG_ADDR      0x2BU /*<! OUT_Y_H register address */

/* OUT_Z_L register */
#define OUT_Z_L_REG_ADDR      0x2CU /*<! OUT_Z_L register address */

/* OUT_Z_H register */
#define OUT_Z_H_REG_ADDR      0x2DU /*<! OUT_Z_H register address */

/* Bit definition and address for FIFO_CTRL register */
#define FIFO_CTRL_REG_ADDR    0x2EU /*<! FIFO_CTRL register address */

#define FIFO_CTRL_FM          0xE0U /*<! FM[2:0] bits - FIFO mode selection */
#define FIFO_CTRL_FM2         0x80U /*<! Bit 2 */
#define FIFO_CTRL_FM1         0x40U /*<! Bit 1 */
#define FIFO_CTRL_FM0         0x20U /*<! Bit 0 */

#define FIFO_CTRL_FTH         0x1FU /*<! FTH[4:0] bits - FIFO threshold setting */
#define FIFO_CTRL_FTH4        0x10U /*<! Bit 4 */
#define FIFO_CTRL_FTH3        0x08U /*<! Bit 3 */
#define FIFO_CTRL_FTH2        0x04U /*<! Bit 2 */
#define FIFO_CTRL_FTH1        0x02U /*<! Bit 1 */
#define FIFO_CTRL_FTH0        0x01U /*<! Bit 0 */

/* FIFO_SRC register */
#define FIFO_SRC_REG_ADDR     0x2FU /*<! FIFO_SRC register address */

/* Bit definition and address for IG_CFG register */
#define IG_CFG_REG_ADDR       0x30U /*<! IG_CFG register address */
#define IG_CFG_AND_OR         0x80U /*<! AND/OR combo of interrupt events (1 = AND combo ; 0 = OR combo) */
#define IG_CFG_LIR            0x40U /*<! Latch interrupt request (1 = IR latched ; 0 = IR not latched) */
#define IG_CFG_ZHIE           0x20U /*<! Enable interrupt generation on Z high event */
#define IG_CFG_ZLIE           0x10U /*<! Enable interrupt generation on Z low event */
#define IG_CFG_YHIE           0x08U /*<! Enable interrupt generation on Y high event */
#define IG_CFG_YLIE           0x04U /*<! Enable interrupt generation on Y low event */
#define IG_CFG_XHIE           0x02U /*<! Enable interrupt generation on X high event */
#define IG_CFG_XLIE           0x01U /*<! Enable interrupt generation on X low event */

/* IG_SRC register */
#define IG_SRC_REG_ADDR       0x31U /*<! IG_SRC register address */

/* Bit definition and address for IG_THS_XH register */
#define IG_THS_XH_REG_ADDR 0x32U /*<! IG_THS_XH register address */
#define IG_THS_XH_DCRM     0x80U /*<! Interrupt generation counter mode selection */

#define IG_THS_XH_THSXH    0x7FU /*<! THSX[14:8] bits - interrupt threshold on X axis */
#define IG_THS_XH_THSX14   0x40U /*<! Bit 14 */
#define IG_THS_XH_THSX13   0x20U /*<! Bit 13 */
#define IG_THS_XH_THSX12   0x10U /*<! Bit 12 */
#define IG_THS_XH_THSX11   0x08U /*<! Bit 11 */
#define IG_THS_XH_THSX10   0x04U /*<! Bit 10 */
#define IG_THS_XH_THSX9    0x02U /*<! Bit 9 */
#define IG_THS_XH_THSX8    0x01U /*<! Bit 8 */

/* Bit definition and address for IG_THS_XL register */
#define IG_THS_XL_REG_ADDR 0x33U /*<! IG_THS_XL register address */

#define IG_THS_XL_THSXL    0xFFU /*<! THSX[7:0] bits - interrupt threshold on X axis */
#define IG_THS_XL_THSX7    0x80U /*<! Bits 7 */
#define IG_THS_XL_THSX6    0x40U /*<! Bits 6 */
#define IG_THS_XL_THSX5    0x20U /*<! Bits 5 */
#define IG_THS_XL_THSX4    0x10U /*<! Bits 4 */
#define IG_THS_XL_THSX3    0x08U /*<! Bits 3 */
#define IG_THS_XL_THSX2    0x04U /*<! Bits 2 */
#define IG_THS_XL_THSX1    0x02U /*<! Bits 1 */
#define IG_THS_XL_THSX0    0x01U /*<! Bits 0 */

/* Bit definition and address for IG_THS_YH register */
#define IG_THS_YH_REG_ADDR 0x34U /*<! IG_THS_YH register address */
/* 8th bit not used */
#define IG_THS_YH_THSYH    0x7FU /*<! THSY[14:8] bits - interrupt theshold on Y axis */
#define IG_THS_YH_THSY14   0x40U /*<! Bit 14 */
#define IG_THS_YH_THSY13   0x20U /*<! Bit 13 */
#define IG_THS_YH_THSY12   0x10U /*<! Bit 12 */
#define IG_THS_YH_THSY11   0x08U /*<! Bit 11 */
#define IG_THS_YH_THSY10   0x04U /*<! Bit 10 */
#define IG_THS_YH_THSY9    0x02U /*<! Bit 9 */
#define IG_THS_YH_THSY8    0x01U /*<! Bit 8 */

/* Bit definition and address for IG_THS_YL register */
#define IG_THS_YL_REG_ADDR 0x35U /*<! IG_THS_YL register address */

#define IG_THS_YL_THSYL    0xFFU /*<! THSY[7:0] bits - interrupt threshold on Y axis */
#define IG_THS_YL_THSY7    0x80U /*<! Bit 7 */
#define IG_THS_YL_THSY6    0x40U /*<! Bit 6 */
#define IG_THS_YL_THSY5    0x20U /*<! Bit 5 */
#define IG_THS_YL_THSY4    0x10U /*<! Bit 4 */
#define IG_THS_YL_THSY3    0x08U /*<! Bit 3 */
#define IG_THS_YL_THSY2    0x04U /*<! Bit 2 */
#define IG_THS_YL_THSY1    0x02U /*<! Bit 1 */
#define IG_THS_YL_THSY0    0x01U /*<! Bit 0 */

/* Bit definition and address for IG_THS_ZH register */
#define IG_THS_ZH_REG_ADDR 0x36U /*<! IG_THS_ZH register address */
/* 8th bit not used */
#define IG_THS_ZH_THSZH    0x7FU /*<! THSZ[14:8] bits - interrupt threshold on Z axis */
#define IG_THS_ZH_THSZ14   0x40U /*<! Bit 14 */
#define IG_THS_ZH_THSZ13   0x20U /*<! Bit 13 */
#define IG_THS_ZH_THSZ12   0x10U /*<! Bit 12 */
#define IG_THS_ZH_THSZ11   0x08U /*<! Bit 11 */
#define IG_THS_ZH_THSZ10   0x04U /*<! Bit 10 */
#define IG_THS_ZH_THSZ9    0x02U /*<! Bit 9 */
#define IG_THS_ZH_THSZ8    0x01U /*<! Bit 8 */

/* Bit definition and address for IG_THS_ZL register */
#define IG_THS_ZL_REG_ADDR 0x37U

#define IG_THS_ZL_THSZL    0xFFU /*<! THSZ[7:0] bits - interrupt threshold on Z axis */
#define IG_THS_ZL_THSZ7    0x80U /*<! Bit 7 */
#define IG_THS_ZL_THSZ6    0x40U /*<! Bit 6 */
#define IG_THS_ZL_THSZ5    0x20U /*<! Bit 5 */
#define IG_THS_ZL_THSZ4    0x10U /*<! Bit 4 */
#define IG_THS_ZL_THSZ3    0x08U /*<! Bit 3 */
#define IG_THS_ZL_THSZ2    0x04U /*<! Bit 2 */
#define IG_THS_ZL_THSZ1    0x02U /*<! Bit 1 */
#define IG_THS_ZL_THSZ0    0x01U /*<! Bit 0 */

/* Bit definition and address for IG_DURATION register */
#define IG_DURATION_REG_ADDR  0x38U /*<! IG_DURATION register address */
#define IG_DURATION_WAIT      0x80U /*<! WAIT enable
                                         1 = if signal crosses the selected threshold,
                                             the interrupt falls after a number of samples
                                             equal to the duration counter register value
                                         0 = the interrupt falls immediately if signal crosses
                                             the selected threshold */

#define IG_DURATION_D         0x7FU /*<! D[6:0] bits - duration value */
#define IG_DURATION_D6        0x40U /*<! Bit 6 */
#define IG_DURATION_D5        0x20U /*<! Bit 5 */
#define IG_DURATION_D4        0x10U /*<! Bit 4 */
#define IG_DURATION_D3        0x08U /*<! Bit 3 */
#define IG_DURATION_D2        0x04U /*<! Bit 2 */
#define IG_DURATION_D1        0x02U /*<! Bit 1 */
#define IG_DURATION_D0        0x01U /*<! Bit 0 */

/* Bit definition and address for LOW_ODR register */
#define LOW_ODR_ADDR  0x39U /*<! LOW_ODR register address */
#define DRDY_HL       0x20U /*<! DRDY/INT2 pin active level (1 = active low ; 0 = active high) */
#define I2C_DIR       0x08U /*<! 1 = SPI only ; 0 = both I2C and SPI enabled */
#define SW_RES        0x04U /*<! Software reset (1 = reset device ; 0 = normal mode ) */
#define LOW_ODR       0x01U /*<! Low speed ODR enable */

#endif /* L3GD20H_REG_H */
/**
 * \}
 */
