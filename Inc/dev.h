#ifndef __DEV_H__
#define __DEV_H__

#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"
#include "imu.h"

#include "dr16.h"
#include "spdm.h"
#include "easyMatrix.h"
#include "rxd.h"


#define ROUGH_CONV				(0)
#define ROUGH_DIV				(1)
#define ROUGH_ERROR				0.1f
#define GLOAB_ERROR				1e-6f
#define FIX_TRUE				(0)
#define FIX_FALSE				(1)


//state for the machine
typedef struct state
{
	float x;
	float y;
	float ax;
	float ay;
	float vx;
	float vy;
} state_t;

typedef struct
{
	float kp[2];
	float kd[2];
	float alpha[2];
	float lastUkd[2];
	float lastErr[2];

	float inc;
	float cnt;

	uint8_t rough;
	uint8_t wait;
} pospid_t;


extern SPDM_TypeDef S;
extern DR16_TypeDef D;
extern state_t observe; //状态观测量
extern state_t estimate; //状态估计量
extern pospid_t pospid;
extern float goalx, goaly; //目标xy值
extern float gainx, gainy; //全局XY位移增益
extern float fix_vel[4];
extern float rough_vel[4];
extern uint8_t fix_cmd;


void dev_init( void );

void datarec_callback( const uint8_t * );
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan);

void kalman_thread( void const * );
void spdm_thread( void const * );
void pospid_calc_thread( void const * );
void imu_thread( void const * );

#endif
