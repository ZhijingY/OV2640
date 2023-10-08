#include "weight.h"

#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

float final_weight; // weight after calibrating
int visit = 0;

uint32_t DWT_Delay_Init(void) {
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  /* Enable TRC */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  /* Enable clock cycle counter */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;
  /* 3 NO OPERATION instructions */
  __NOP();
  __NOP();
  __NOP();
  /* Check if clock cycle counter has started */
  if(DWT->CYCCNT)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

void DWT_Delay_us(volatile uint32_t usec)
{
 uint32_t clk_cycle_start = DWT->CYCCNT;
 usec *= (HAL_RCC_GetHCLKFreq() / 1000000);
 while ((DWT->CYCCNT - clk_cycle_start) < usec);
}

unsigned long weight_sensor(void) {
	unsigned long data;
	float cast;
	int i;

	data = 0;

	HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, GPIO_PIN_RESET);
	//DWT_Delay_us(5);
	HAL_Delay(2);
	// set clk off
	HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
	//DWT_Delay_us(5);
	HAL_Delay(2);

	while(HAL_GPIO_ReadPin(DATA_GPIO_Port, DATA_Pin));

	for (i = 0; i < 24; i++) {
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
		DWT_Delay_us(4);
		//HAL_Delay(2);
		data = data << 1;
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
		DWT_Delay_us(4);
		//HAL_Delay(2);
		data += HAL_GPIO_ReadPin(DATA_GPIO_Port, DATA_Pin);
	}
	HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
	DWT_Delay_us(5);
	//HAL_Delay(2);

	data ^= 0x800000;

	HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
	//DWT_Delay_us(5);
	HAL_Delay(2);

	if (data > 0x800000) {
		cast = data & 0x7fffff;
		cast = -cast;
	}

	cast += 198650;
	cast /= 22.0;

	return cast;
}

float average_weight(void) {
	float average = 0;

	for(int i = 0; i < 10; i++) {
		average += weight_sensor();
	}

	return average/10;
}

float weight_cali(void) {
	float weight1;
	float weight2;
	float weight = 0;

	weight1 = weight_sensor();
	HAL_Delay(5000);
	weight2 = weight_sensor();

	if(visit == 0 && abs((int)weight2 - (int)weight1) < 50) {
		weight = 0;
		final_weight = (weight2 + weight1)/2;
	} else if(visit == 0 && (weight2 - weight1 > 50)) {
		visit = 1;
	} else if(visit == 1 && (weight1 - weight2) > 200) {
		visit = 0;
		HAL_Delay(500);
		weight = weight_sensor() - final_weight;
	}
	return weight;
}
