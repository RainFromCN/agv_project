#ifndef __AGVUI_H__
#define __AGVUI_H__

#include "stm32f4xx_hal.h"

typedef struct
{
	UART_HandleTypeDef * huart;
	uint8_t data_type;

	void * posx_add;
	void * posy_add;
	int head;
	float params[3];

	char tmp[30];

	/* 接收字节的长度 */
	uint8_t rx_length;

	/* DMA数据缓冲 */
	uint8_t rx_data1[32];
	uint8_t rx_data2[32];

	/* 保存目标地址 */
	uint8_t rx_addr[32];

} Agvui_t;


void AGVUI_Open( Agvui_t * a, UART_HandleTypeDef * huart );
void AGVUI_RxUpdate( Agvui_t * a );
void AGVUI_Callback( Agvui_t * a );
int AGVUI_GetHead( Agvui_t * a );
float * AGVUI_GetParams( Agvui_t * a );
void AGVUI_TransmitString( UART_HandleTypeDef* huart, const char * string );
void AGVUI_SetTxDataAdd( Agvui_t * a, void * posx, void * posy);
void AGVUI_MainTask( Agvui_t * a );


#endif
