#ifndef __KTOOLS_H__
#define __KTOOLS_H__

#include "stm32f4xx_hal.h"
#include "string.h"


typedef float KTOOLS_SENSOR_DATA_TYPE;


typedef struct
{
	UART_HandleTypeDef * huart;
	void * sensor_data;
	uint8_t data_type;
	uint8_t tmp[20];

}KTools_t;

void KTools_Open( KTools_t * KTools, UART_HandleTypeDef * huart, void * sensor_data );
void KTools_MainTask( KTools_t * k );


#endif
