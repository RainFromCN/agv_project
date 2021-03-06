#include "agvui.h"
#include <stdlib.h>
#include <string.h>

static void two_float2signal( float refer, float trace, char * tmp )
{
	int stack[18];
	int top = 0, cnt = 0;

	float f1 = refer * 1000;
	float f2 = trace * 1000;
	int n1 = f1, n2 = f2;

	//处理帧头
	tmp[cnt++] = '@';

	//处理第一个数据
	if ( n1 != 0 )
	{
		if ( n1 < 0 )
		{
			n1 = -n1;
			tmp[cnt++] = '-';
		}
		while ( n1 > 0 )
		{
			stack[top++] = n1 % 10;
			n1 /= 10;
		}
		while ( (--top) >= 0 )
		{
			tmp[cnt++] = '0' + stack[top];
		}
	}
	else
	{
		tmp[cnt++] = '0';
	}

	//分隔符
	tmp[cnt++] = ',';
	top = 0;

	//处理第二个数据
	if ( n2 != 0 )
	{
		if ( n2 < 0 )
		{
			n2 = -n2;
			tmp[cnt++] = '-';
		}
		while ( n2 > 0 )
		{
			stack[top++] = n2 % 10;
			n2 /= 10;
		}
		while ( (--top) >= 0 )
		{
			tmp[cnt++] = '0' + stack[top];
		}
	}
	else
	{
		tmp[cnt++] = '0';
	}

	//处理帧尾
	tmp[cnt++] = '!';
	tmp[cnt++] = '\0';
}

void AGVUI_Open( Agvui_t * a, UART_HandleTypeDef * huart )
{
	a->huart = huart;
	a->rx_length = 16;

	SET_BIT( huart->Instance->CR3, USART_CR3_DMAR );
	__HAL_UART_ENABLE_IT( huart, UART_IT_IDLE );
	__HAL_DMA_DISABLE( huart->hdmarx );
	while ( huart->hdmarx->Instance->CR & DMA_SxCR_EN )
	{
		__HAL_DMA_DISABLE( huart->hdmarx );
	}
	huart->hdmarx->Instance->PAR = ( uint32_t ) & ( huart->Instance->DR );
	huart->hdmarx->Instance->M0AR = ( uint32_t )( a->rx_data1 );
	huart->hdmarx->Instance->M1AR = ( uint32_t )( a->rx_data2 );
	huart->hdmarx->Instance->NDTR = a->rx_length * 2;
	SET_BIT( huart->hdmarx->Instance->CR, DMA_SxCR_DBM );
	__HAL_DMA_ENABLE( huart->hdmarx );
}

void AGVUI_RxUpdate( Agvui_t * a )
{
	static uint8_t i;

	if ( a->huart->Instance->SR & UART_FLAG_IDLE ) {

		static uint16_t this_time_rx_len = 0;
		static UART_HandleTypeDef * huart;
		huart = a->huart;
		__HAL_UART_CLEAR_PEFLAG( huart );
		if ( ( huart->hdmarx->Instance->CR & DMA_SxCR_CT ) == RESET )
		{
			__HAL_DMA_DISABLE( huart->hdmarx );

			this_time_rx_len = a->rx_length * 2 - huart->hdmarx->Instance->NDTR;
			huart->hdmarx->Instance->NDTR = a->rx_length * 2;
			huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
			__HAL_DMA_ENABLE( huart->hdmarx );

			if ( this_time_rx_len == a->rx_length )
			{
				for ( i = 0; i < a->rx_length; i ++ ) a->rx_addr[i] = a->rx_data2[i];
			}
		}

		else
		{
			__HAL_DMA_DISABLE( huart->hdmarx );

			this_time_rx_len = a->rx_length * 2 - huart->hdmarx->Instance->NDTR;
			huart->hdmarx->Instance->NDTR = a->rx_length * 2;
			huart->hdmarx->Instance->CR &= ~( DMA_SxCR_CT );
			__HAL_DMA_ENABLE( huart->hdmarx );

			if ( this_time_rx_len == a->rx_length ) {
				for ( i = 0; i < a->rx_length; i ++ ) a->rx_addr[i] = a->rx_data1[i];
			}
		}

		/* 接收到一帧数据 */
		int * pi = (int *)a->rx_addr;
		a->head = *pi;
		float * pf = (float *)(a->rx_addr+4);
		a->params[0] = *pf;
		pf = (float *)(a->rx_addr + 8);
		a->params[1] = *pf;
		pf = (float *)(a->rx_addr + 12);
		a->params[2] = *pf;
		AGVUI_Callback(a);
	}
}

__weak void AGVUI_Callback( Agvui_t * a )
{

}

int AGVUI_GetHead( Agvui_t * a )
{
	return a->head;
}

float * AGVUI_GetParams( Agvui_t * a )
{
	return a->params;
}

void AGVUI_SetTxDataAdd( Agvui_t * a, void * posx, void * posy)
{
	a->posx_add = posx;
	a->posy_add = posy;
}

static void transmit_data( Agvui_t * p )
{
	float a, b;
	a = * ( float * )p->posx_add;
	b = * ( float * )p->posy_add;
	two_float2signal( a, b, p->tmp );
	HAL_UART_Transmit( p->huart, (uint8_t*)p->tmp, strlen( p->tmp ), 0xffff );
}

void AGVUI_TransmitString( UART_HandleTypeDef* huart, const char * string )
{
	char head = '@';
	char tail = '!';
	char end = '\n';
	HAL_UART_Transmit( huart, (uint8_t*)&head, 1, 0xffff );
	HAL_UART_Transmit( huart, (uint8_t*)&string, strlen(string), 0xffff );
	HAL_UART_Transmit( huart, (uint8_t*)&tail, 1, 0xffff );
	HAL_UART_Transmit( huart, (uint8_t*)&end, 1, 0xffff);
}

void AGVUI_MainTask( Agvui_t * a )
{
	transmit_data( a );
}
