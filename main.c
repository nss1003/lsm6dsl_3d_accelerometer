/*
 * main.c
 *
 *  Created on: 23.04.2020
 *      Author: silvere
 */
#define STM32L4

#include <stdio.h>
#include "lsm6dsl.h"
#include "systick.h"


int main(void)
{
	int16_t temp;
	float *acc_xyz;
	float *gy_xyz;

	clock_setup();
	systick_ms_setup();
	uart_setup();
	i2c2_setup();
	lsm6dsl_enable();

	who_i_am(LSM6DSL_ADDR, WHO_I_AM);

	while (1) {
		msleep(2000);

		temp = temperature_read();
		acc_xyz = acc_axis_read();
		gy_xyz = gy_grades_read();

		printf("Actual temperature: %d\n",temp);
		printf("Accelerometer values:\n");
		printf("*************************\n");
		printf("* X = %.2f\n",*acc_xyz);
		printf("* Y = %.2f\n",*(acc_xyz + 1));
		printf("* Z = %.2f\n",*(acc_xyz + 2));
		printf("*************************\n");

		printf("Gyroscope values:\n");
		printf("+++++++++++++++++++++++++\n");
		printf("+ X = %.2f\n",*gy_xyz);
		printf("+ Y = %.2f\n",*(gy_xyz + 1));
		printf("+ Z = %.2f\n",*(gy_xyz + 2));
		printf("+++++++++++++++++++++++++\n");
	}

	return 0;
}
