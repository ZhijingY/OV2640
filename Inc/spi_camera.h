#ifndef __SPI_CAMERA_H
#define __SPI_CAMERA_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi);

#ifdef __cplusplus
}
#endif


#endif
