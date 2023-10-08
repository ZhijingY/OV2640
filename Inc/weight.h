#ifndef __WEIGHT_H
#define __WEIGHT_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

unsigned long weight_sensor(void);
float weight_cali(void);
uint32_t DWT_Delay_Init(void);
void DWT_Delay_us(volatile uint32_t usec);
float average_weight(void);

#ifdef __cplusplus
}
#endif


#endif
