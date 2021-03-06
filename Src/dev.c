#include "dev.h"
#include "math.h"
#include "agvui.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"


SPDM_TypeDef S;
IMU_TypeDef I;
RXD_TypeDef R;

state_t observe;
state_t estimate;
pospid_t pospid;


float cmd_vel[4];
float rough_vel[4];
float fix_vel[4];
uint8_t fix_cmd = FIX_FALSE;

float goalx=0,goaly=0; //全局XY坐标目标值
float gainx,gainy; //全局XY位移增益


float ch[4]; //四个遥控器通道
float vel[4]; //保存四个电机转速
uint8_t rec[20];

uint32_t times = 0;
uint32_t systicks = 0;

void dev_init( void )
{
	//motor init
	S = SPDM_Open(&hcan1, 0x200);
	SPDM_SetDir(&S, -1, 1, 1, -1);
	float ctrlParams[4][4] = {
	{600.f,     600.f,    	600.f,  	600.f},
	{60.f,      60.f,    	60.f,    	60.f},
	{1000.f,    1000.f,   	1000.f,    	1000.f}
	};
	int16_t outputSat[4] = {10000, 10000, 10000, 10000};
	float exCtrlParams[3][4] = {
	{17.f,      17.f,    	17.f,    	17.f},
	{3.f,       3.f,    	3.f,    	3.f},
	{0.3f,      0.3f,    	0.3f,    	0.3f}
	};
	SPDM_CtrlParams(&S, ctrlParams[0], ctrlParams[1], ctrlParams[2], outputSat);
	SPDM_ExCtrlParams(&S, exCtrlParams[0], exCtrlParams[1], exCtrlParams[2]);

	//mpu 6500
	while (IMU_CheckSuccess(&I) != HAL_OK)
	{
		I = IMU_Open(&hspi5);
	}
	IMU_SetBoardState(&I, IMU_BOARD_UP);

	//set the position pid parameters
	pospid.inc = 2.5;
	pospid.lastUkd[0] = 0;
	pospid.lastUkd[1] = 0;
	pospid.lastErr[0] = 0;
	pospid.lastErr[1] = 0;
	pospid.rough = ROUGH_CONV;
	gainx = 0.9;
	gainy = 0.968;

	// receive and transmit with uart7
	R = RXD_Open(&huart7,(uint8_t*)&rec, 10);
	RXD_Enable(&R);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    SPDM_RxUpdate( &S, hcan );
}

void RXD_Callback(RXD_TypeDef * rxd)
{
	if (rxd == &R) {
		datarec_callback(rec);
	}
}

// begin1



// end1

void datarec_callback(const uint8_t * phead)
{
	// begin2

	// end2
}

void arm_calc_thread( void const * argument )
{
	while (1)
	{
		// begin3


		// end3

		osDelay(10); // 挂起10ms
	}
}

void kalman_thread( void const * argument )
{
	uint8_t i;

	double T = 0.001 / 0.98;

	float A_initval[] = {
		1,		T,		T*T/2,	0,		0,		0,
		0, 		1,		T,		0,		0,		0,
		0,		0,		1,		0,		0,		0,
		0,		0,		0,		1,		T,		T*T/2,
		0,		0,		0,		0,		1,		T,
		0,		0,		0,		0,		0,		1
	};
	float Xk_initval[] = {0,0,0,0,0,0};
	float Z_initval[] = {0,0,0,0};
	float H_initval[] = {
		0,1,0,0,0,0,
		0,0,1,0,0,0,
		0,0,0,0,1,0,
		0,0,0,0,0,1
	};
	float P_initval[] = {
		1,0,0,0,0,0,
		0,1,0,0,0,0,
		0,0,1,0,0,0,
		0,0,0,1,0,0,
		0,0,0,0,1,0,
		0,0,0,0,0,1,
	};
	float Q_initval[] = {
		1e-3, 0, 0, 0, 0, 0,
		0, 1e-7, 0, 0, 0, 0,
		0, 0, 1e-5, 0, 0, 0,
		0, 0, 0, 1e-3, 0, 0,
		0, 0, 0, 0, 1e-7, 0,
		0, 0, 0, 0, 0, 1e-5
	};
	float R_initval[] = {
		1.2e-5, 0, 0, 0,
		0, 7.0e-5, 0, 0,
		0, 0, 1.2e-5, 0,
		0, 0, 0, 7.5e-5
	};

	// 当前状态，上一时刻状态，先验状态
	CREATE_MATRIX_ONSTACK(6,1,Xk,Xk_initval); //当前时刻状态
	CREATE_MATRIX_ONSTACK(6,1,X_,NULL);

	// 当前误差协方差，上一时刻误差协方差，先验误差协方差
	CREATE_MATRIX_ONSTACK(6,6,Pk,P_initval);
	CREATE_MATRIX_ONSTACK(6,6,P_,NULL);

	// A,B,G,H
	CREATE_MATRIX_ONSTACK(6,6,A,A_initval); //状态转移矩阵
	CREATE_MATRIX_ONSTACK(4,6,H,H_initval); //观测矩阵

	// 控制矩阵，观测矩阵
	CREATE_MATRIX_ONSTACK(4,1,Z,Z_initval); //观测结果

	// 过程噪声协方差矩阵，量测噪声协方差矩阵
	CREATE_MATRIX_ONSTACK(6,6,Q,Q_initval); //过程噪声
	CREATE_MATRIX_ONSTACK(4,4,R,R_initval); //测量噪声

	// 卡尔曼增益矩阵
	CREATE_MATRIX_ONSTACK(6,4,Kk,NULL);

	// 用于暂存转置矩阵
	CREATE_MATRIX_ONSTACK(6,6,Atrans,NULL);
	CREATE_MATRIX_ONSTACK(6,4,Htrans,NULL);
	transMatrix(&A,&Atrans);
	transMatrix(&H,&Htrans);

	// 单位阵
	CREATE_MATRIX_ONSTACK(6,6,EYE,P_initval);

	// 公式二所用的缓冲矩阵
	CREATE_MATRIX_ONSTACK(6,6,TMP21,NULL);
	CREATE_MATRIX_ONSTACK(6,6,TMP22,NULL);

	// 公式三所用的缓冲矩阵
	CREATE_MATRIX_ONSTACK(6,4,TMP31,NULL);
	CREATE_MATRIX_ONSTACK(4,6,TMP32,NULL);
	CREATE_MATRIX_ONSTACK(4,4,TMP33,NULL);
	CREATE_MATRIX_ONSTACK(4,4,TMP34,NULL);
	CREATE_MATRIX_ONSTACK(4,4,TMP35,NULL);

	// 公式四所用的缓冲矩阵
	CREATE_MATRIX_ONSTACK(4,1,TMP41,NULL);
	CREATE_MATRIX_ONSTACK(4,1,TMP42,NULL);
	CREATE_MATRIX_ONSTACK(6,1,TMP43,NULL);

	// 公式五所用的缓冲矩阵
	CREATE_MATRIX_ONSTACK(6,6,TMP51,NULL);
	CREATE_MATRIX_ONSTACK(6,6,TMP52,NULL);

	osDelay(500);

	while (1)
	{

		/* 获取当前时刻观测加速度 */
		observe.ax = -I.ay;
		observe.ay = I.ax;

		/* 获取当前时刻观测速度 */
		for (i = 0; i < 4; i ++)
		{
			vel[i] = S.velocity[i] * 8.9e-5f * S.dir[i];
		}
		observe.vy = (vel[0] + vel[1] + vel[2] + vel[3]) / 4.f;
		observe.vx = (vel[3] - vel[2] + vel[1] - vel[0]) / 4.f;

		/* 准备好观测向量 */
		Z_initval[0] = observe.vx;
		Z_initval[1] = observe.ax;
		Z_initval[2] = observe.vy;
		Z_initval[3] = observe.ay;
		setMatrix(Z_initval,&Z);

		/* X_ = A*Xkprv */
		multiMatrix(&A,&Xk,&X_);

		/* P_ = A*Pkprv*AT + Q */
		multiMatrix(&A,&Pk,&TMP21);
		multiMatrix(&TMP21,&Atrans,&TMP22); // TMP2=A*Pkprv*AT
		addMatrix(&TMP22,&Q,&P_);

		/* Kk = P_*HT*(H*P_*HT + R)-1 */
		multiMatrix(&P_,&Htrans,&TMP31); // TMP1=P_*HT
		multiMatrix(&H,&P_,&TMP32); //TMP2=H*P_
		multiMatrix(&TMP32,&Htrans,&TMP33); //TMP3=H*P_*HT
		addMatrix(&TMP33,&R,&TMP34); //TMP4=H*P_*HT+R
		invMatrix(&TMP34,&TMP35); //TMP5=(H*P_*HT+R)-1
		multiMatrix(&TMP31,&TMP35,&Kk);

		/* Xk = X_ + Kk(Zk - H*X_) */
		multiMatrix(&H,&X_,&TMP41); // TMP1=H*X_
		subMatrix(&Z,&TMP41,&TMP42); // TMP2=Zk - H*X_
		multiMatrix(&Kk,&TMP42,&TMP43); // TMP3=Kk*(Zk - H*X_)
		addMatrix(&X_,&TMP43,&Xk); // Xk = X_ + Kk*(Zk - H*X_)

		/* Pk = (I - Kk*H)*P_ */
		multiMatrix(&Kk,&H,&TMP51); // TMP1=Kk*H
		subMatrix(&EYE,&TMP51,&TMP52); // TMP2=I - Kk*H
		multiMatrix(&TMP52,&P_,&Pk); // Pk=(I - Kk*H)*P_

		if (fix_cmd == FIX_TRUE)
		{
			fix_cmd = FIX_FALSE;
			Xk.element[0] = 0;
			Xk.element[3] = 0;
			Xk.element[1] = 0;
			Xk.element[2] = 0;
			Xk.element[4] = 0;
			Xk.element[5] = 0;
		}

		estimate.x  = Xk.element[0] * gainx;
		estimate.vx = Xk.element[1];
		estimate.ax = Xk.element[2];
		estimate.y  = Xk.element[3] * gainy;
		estimate.vy = Xk.element[4];
		estimate.ay = Xk.element[5];

		osDelay(1);
	}
}

void spdm_thread( void const * argument )
{
	while (1)
	{
		SPDM_CalcPid(&S);
		SPDM_SendCmd(&S, S.volt[0], S.volt[1], S.volt[2], S.volt[3]);
		osDelay(1);
	}
}

void pospid_calc_thread(void const * argument)
{
	float errx,erry;
	float forward, offset;
	float kp0, kp1;
	uint8_t mark, i;

	while (1)
	{
		if (pospid.rough == ROUGH_CONV)
		{
			SPDM_CmdVel(&S,\
				fix_vel[0],\
				fix_vel[1],\
				fix_vel[2],\
				fix_vel[3]\
			);
		}
		else
		{
			if (pospid.cnt < 300)
			{
				pospid.cnt += pospid.inc;
			}
			kp0 = pospid.cnt * 1.f;
			kp1 = pospid.cnt * 1.f;


			errx = goalx - estimate.x;
			erry = goaly - estimate.y;

			forward = kp1 * erry;
			offset = kp0 * errx;

			rough_vel[0] = forward - offset;
			rough_vel[1] = forward + offset;
			rough_vel[2] = forward - offset;
			rough_vel[3] = forward + offset;

			/* 检查是否有粗略移动速度 */
			if (pospid.rough == ROUGH_DIV)
			{
				mark = 1;

				for (i = 0; i < 4; i ++)
				{
					if (rough_vel[i] > ROUGH_ERROR || rough_vel[i] < -ROUGH_ERROR)
					{
						mark = 0;
					}
				}

				if (mark == 1)
				{
					uint8_t cmd[4];
					cmd[1] = '@';
					cmd[2] = '1';
					cmd[3] = '!';
					cmd[4] = '\n';
					pospid.rough = ROUGH_CONV;
					HAL_UART_Transmit(&huart8, cmd, 4, 0xffff);
					pospid.cnt = 0;
				}
			}

			/* 还在粗略移动 */
			SPDM_CmdVel(&S,\
				rough_vel[0],\
				rough_vel[1],\
				rough_vel[2],\
				rough_vel[3]\
			);
		}

		osDelay(10);
	}
}

void imu_thread(void const * argument)
{
	while ( 1 )
	{
		IMU_GetData(&I);
		osDelay(1);
	}
}
