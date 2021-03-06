#include "kalmantools.h"

void KTools_Open( KTools_t * KTools, UART_HandleTypeDef * huart, void * sensor_data )
{
	KTools->huart = huart;
	KTools->sensor_data = sensor_data;
}

static void float2signal( float x, char * tmp )
{
	int n = x * 100000;
	int stack[18];
	int top = 0, cnt = 0;

	//处理帧头
	tmp[cnt++] = '@';

	//处理第一个数据
	if ( n != 0 )
	{
		if ( n < 0 )
		{
			n = -n;
			tmp[cnt++] = '-';
		}
		while ( n > 0 )
		{
			stack[top++] = n % 10;
			n /= 10;
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

static void transmit_data( KTools_t * k )
{
	float a;
	a = * ( KTOOLS_SENSOR_DATA_TYPE * )k->sensor_data;
	float2signal(a,(char*)k->tmp);
	HAL_UART_Transmit( k->huart, (uint8_t*)k->tmp, strlen( (char*)k->tmp ), 0xffff );
}

void KTools_MainTask( KTools_t * k )
{
	transmit_data(k);
}
