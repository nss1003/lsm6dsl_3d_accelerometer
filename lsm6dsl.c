/*
 * lsm6dsl.c
 *
 *  Created on: 28 Apr 2020
 *      Author: silvere
 */

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>

#include "lsm6dsl.h"
#include "systick.h"


void clock_setup(void)
{
	/* FIXME - this should eventually become a clock struct helper setup */
		rcc_osc_on(RCC_HSI16);

		flash_prefetch_enable();
		flash_set_ws(4);
		flash_dcache_enable();
		flash_icache_enable();
		/* 16MHz / 4 = > 4 * 40 = 160MHz VCO => 80MHz main pll  */
		rcc_set_main_pll(RCC_PLLCFGR_PLLSRC_HSI16, 4, 40,
				0, 0, RCC_PLLCFGR_PLLR_DIV2);
		rcc_osc_on(RCC_PLL);

		rcc_periph_clock_enable(RCC_GPIOB);
		rcc_periph_clock_enable(RCC_GPIOC);
		rcc_periph_clock_enable(RCC_USART3);
		rcc_periph_clock_enable(RCC_I2C2);


		rcc_set_sysclk_source(RCC_CFGR_SW_PLL); /* careful with the param here! */
		rcc_wait_for_sysclk_status(RCC_PLL);

		/* FIXME - eventually handled internally */
		rcc_ahb_frequency = 80e6;
		rcc_apb1_frequency = 80e6;
		rcc_apb2_frequency = 80e6;
}

void uart_setup(void)
{
	/* Setup GPIO pins for USART3 transmit. */
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4|GPIO5);

	/* Setup UART4 TX and RX pin as alternate function. */
	gpio_set_af(GPIOC, GPIO_AF7, GPIO5);
	gpio_set_af(GPIOC, GPIO_AF7, GPIO4);

	//USART3 setup for printf commands
	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

	/* Finally enable USART3. */
	usart_enable(USART3);

}

void i2c2_setup(void)
{
	/* Setup SDA and SLC for I2C communication*/
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, SCL);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, SDA);

	/* Setup SDA and SCL pin as alternate function. */
	gpio_set_af(GPIOB, GPIO_AF4, SCL);
	gpio_set_af(GPIOB, GPIO_AF4, SDA);

	i2c_peripheral_disable(I2C2);
	i2c_enable_analog_filter(I2C2);

	i2c_set_speed(I2C2,i2c_speed_sm_100k, 8);
	i2c_enable_stretching(I2C2);

	i2c_set_7bit_addr_mode(I2C2);
	i2c_peripheral_enable(I2C2);

}

/**
 * Use USART3 as a console.
 * This is a syscall for newlib
 * @param file
 * @param ptr
 * @param len
 * @return
 */

int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART3, '\r');
			}
			usart_send_blocking(USART3, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

void lsm6dsl_enable(void)
{
	msleep(20); // Boot time norally 15ms
	uint8_t cmd[2];

	// Enable BDU and IF_NC
	cmd[0] = CTRL3_C;
	i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
	cmd[1] |= (BDU_ON | IF_NC_ON);
	i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 2, NULL, 0);

	// Set accelerometer ODR = 416Hz, FS_XL = ±4g (Output / FS_XL_4G)
	cmd[0] = CTRL1_XL;
	i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
	cmd[1] |= (1 << ODR_XL2 | 1 << ODR_XL1 | 1 << FS_XL1);
	i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 2, NULL, 0);

	// Set gyroscope ODR = 416Hz, FS_G = 1000dps (Output / FS_G_1000DPS)
	cmd[0] = CTRL2_GY;
	i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
	cmd[1] |= (1 << ODR_GY2 | 1 << ODR_GY1 | 1 << FS_GY1);
	i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 2, NULL, 0);

	// Enable accelerometer and gyroscope data ready on INT1
	cmd[0] = INT1_CTRL;
	i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
	cmd[1] |= (INT1_GY_ON | INT1_ACC_ON);
	i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 2, NULL, 0);

	// Enable rounding function
	cmd[0] = CTRL5_C;
	i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
	cmd[1] |= (1 << ROUNDING1 | 1 << ROUNDING0);
	i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 2, NULL, 0);
}

float *acc_axis_read(void)
{
	uint8_t outx_l_xl;
	uint8_t outx_h_xl;
	uint8_t outy_l_xl;
	uint8_t outy_h_xl;
	uint8_t outz_l_xl;
	uint8_t outz_h_xl;

	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t status_cmd[2];
	uint8_t cmd[2];

	/*
	* Alternatively to keyword static in this case
	* use malloc to dynamic allocate memory for acc_xyz
	* e.g:
	* int *acc_xyz = malloc(sizeof(array_size))
	* dont froget to free memory after using it with free(array);
	*/
	static float acc_xyz[3];

	status_cmd[0] = STATUS_REG;
	i2c_transfer7(I2C2, LSM6DSL_ADDR, status_cmd, 1, (status_cmd+1), 1);

	if (status_cmd[1] & GET_XLDA) {
		// Read Y
		cmd[0] = OUTX_L_XL;
		i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
		outx_l_xl = cmd[1];

		cmd[0] = OUTX_H_XL;
		i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
		outx_h_xl = cmd[1];

		x = ((int16_t)outx_h_xl << 8 | (int16_t)outx_l_xl);

		// Read Y

		cmd[0] = OUTY_L_XL;
		i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
		outy_l_xl = cmd[1];

		cmd[0] = OUTY_H_XL;
		i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
		outy_h_xl = cmd[1];

		y = ((int16_t)outy_h_xl << 8 | (int16_t)outy_l_xl);

		// Read Z
		cmd[0] = OUTZ_L_XL;
		i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
		outz_l_xl = cmd[1];

		cmd[0] = OUTZ_H_XL;
		i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
		outz_h_xl = cmd[1];

		z = ((int16_t)outz_h_xl << 8 | (int16_t)outz_l_xl);

		acc_xyz[0] = (float)x * FS_XL_4G;
		acc_xyz[1] = (float)y * FS_XL_4G;
		acc_xyz[2] = (float)z * FS_XL_4G;
	}
	return acc_xyz;

}


float *gy_grades_read(void)
{
	uint8_t outx_l_gy;
	uint8_t outx_h_gy;
	uint8_t outy_l_gy;
	uint8_t outy_h_gy;
	uint8_t outz_l_gy;
	uint8_t outz_h_gy;

	int16_t x;
	int16_t y;
	int16_t z;

	uint8_t status_cmd[2];
	uint8_t cmd[2];

	/*
	* Alternatively to keyword static in this case
	* use malloc to dynamic allocate memory for gy_xyz
	* e.g:
	* int *gy_xyz = malloc(sizeof(array_size))
	* dont froget to free memory after using it with free(array);
	*/
	static float gy_xyz[3];

	status_cmd[0] = STATUS_REG;
	i2c_transfer7(I2C2, LSM6DSL_ADDR, status_cmd, 1, (status_cmd+1), 1);

	if (status_cmd[1] & GET_GYDA) {

		// Read Y
		cmd[0] = OUTX_L_GY;
		i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
		outx_l_gy = cmd[1];

		cmd[0] = OUTX_H_GY;
		i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
		outx_h_gy = cmd[1];

		x = ((int16_t)outx_h_gy << 8 | (int16_t)outx_l_gy);

		// Read Y
		cmd[0] = OUTY_L_GY;
		i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
		outy_l_gy = cmd[1];

		cmd[0] = OUTY_H_GY;
		i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
		outy_h_gy = cmd[1];

		y = ((int16_t)outy_h_gy << 8 | (int16_t)outy_l_gy);

		// Read Z
		cmd[0] = OUTZ_L_GY;
		i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
		outz_l_gy = cmd[1];

		cmd[0] = OUTZ_H_GY;
		i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
		outz_h_gy = cmd[1];

		z = ((int16_t)outz_h_gy << 8 | (int16_t)outz_l_gy);

		gy_xyz[0] = (float)x * FS_G_1000DPS;
		gy_xyz[1] = (float)y * FS_G_1000DPS;
		gy_xyz[2] = (float)z * FS_G_1000DPS;
	}
	return gy_xyz;
}


int16_t temperature_read(void)
{
	uint8_t temp_out_l;
	uint8_t temp_out_h;

	uint8_t status_cmd[2];
	uint8_t cmd[2];

	static int16_t temp = 0;

	status_cmd[0] = STATUS_REG;
	i2c_transfer7(I2C2, LSM6DSL_ADDR, status_cmd, 1, (status_cmd+1), 1);

	if (status_cmd[1] & GET_TDA) {
		cmd[0] = OUT_TEMP_L;
		i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
		temp_out_l = cmd[1];

		cmd[0] = OUT_TEMP_H;
		i2c_transfer7(I2C2, LSM6DSL_ADDR, cmd, 1, (cmd+1), 1);
		temp_out_h = cmd[1];

		temp = (int16_t)temp_out_h << 8 | (int16_t)temp_out_l;
	}
	return temp;
}


void who_i_am(uint8_t dev, uint8_t reg)
{

	uint8_t reg_cmd[2];
	reg_cmd[0] = reg;
	i2c_transfer7(I2C2, dev, reg_cmd, 1, (reg_cmd+1), 1);

	printf("Wo i am = 0x%02x\n",reg_cmd[1]);
}

