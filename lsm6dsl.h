/*
 * lsm6dsl.h
 *
 *  Created on: 28 Apr 2020
 *      Author: silvere
 */

#ifndef LSM6DSL_H_
#define LSM6DSL_H_

#define SCL GPIO10
#define SDA GPIO11


#define LSM6DSL_ADDR 0x6A
#define WHO_I_AM 0x0F
#define STATUS_REG 0x1E
#define OUT_TEMP_L 0x20
#define OUT_TEMP_H 0x21

#define OUTX_L_GY 0x22
#define OUTX_H_GY 0x23
#define OUTY_L_GY 0x24
#define OUTY_H_GY 0x25
#define OUTZ_L_GY 0x26
#define OUTZ_H_GY 0x27

#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D

#define TIMESTAMP1_REG 0x41
#define TIMESTAMP2_REG 0x42

#define INT1_CTRL 0x0D
#define INT1_DRDY_GY 1
#define INT1_DRDY_XL 0
#define INT1_ACC_ON (1 << INT1_DRDY_XL)
#define INT1_GY_ON (1 << INT1_DRDY_GY)

#define CTRL1_XL 0x10
#define ODR_XL3 7
#define	ODR_XL2 6
#define ODR_XL1 5
#define ODR_XL0 4
#define FS_XL1 3
#define FS_XL0 2

#define FS_XL_2G 0.061
#define FS_XL_4G 0.122
#define FS_XL_8G 0.244
#define FS_XL_16G 0.488

#define CTRL2_GY 0x11
#define ODR_GY3 7
#define ODR_GY2 6
#define ODR_GY1 5
#define ODR_GY0 4
#define FS_GY1 3
#define FS_GY0 2

#define FS_G_250DPS 8.75
#define FS_G_500DPS 17.5
#define FS_G_1000DPS 35.00
#define FS_G_2000DPS 70.00


#define CTRL3_C 0x12
#define BDU 6
#define IF_NC 2
#define BDU_ON (1 << BDU)
#define IF_NC_ON (1 << IF_NC)

#define CTRL5_C 0x14
#define ROUNDING2 7
#define ROUNDING1 6
#define ROUNDING0 5


/*
 *  Read status register and compare with following macros
 *  if output data are available
*/
#define GET_TDA 0x04
#define GET_GYDA 0x02
#define GET_XLDA 0x01


void i2c2_setup(void);
void uart_setup(void);
void clock_setup(void);
int _write(int file, char *ptr, int len);

void who_i_am(uint8_t dev, uint8_t reg);
int16_t temperature_read(void);
float *gy_grades_read(void);
float *acc_axis_read(void);
void lsm6dsl_enable(void);

#endif /* LSM6DSL_H_ */
