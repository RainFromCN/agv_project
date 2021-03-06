/*
 * imu.c
 *
 *  Created on: 2020年10月16日
 *      Author: 哈理工RM电控组
 */

#include "imu.h"
#include <math.h>
#include <string.h>

struct {
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	int16_t temp;

	int16_t gx;
	int16_t gy;
	int16_t gz;

	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
} mpu_data;

static uint8_t mpu_buff[14];

/* 从MPU的寄存器中读取一个数据 */
uint8_t MPU_ReadByte(IMU_TypeDef* I, uint8_t const reg) {
	MPU_NSS_LOW;
	I->tx = reg | 0x80;
	HAL_SPI_TransmitReceive(I->spi, &I->tx, &I->rx, 1, 55);
	HAL_SPI_TransmitReceive(I->spi, &I->tx, &I->rx, 1, 55);
	MPU_NSS_HIGH;
	return I->rx;
}

/* 向MPU的寄存器中写入一个数据 */
uint8_t MPU_WriteByte(IMU_TypeDef* I, uint8_t const reg, uint8_t const data) {
	MPU_NSS_LOW;
	I->tx = reg & 0x7f;
	HAL_SPI_TransmitReceive(I->spi, &I->tx, &I->rx, 1, 55);
	I->tx = data;
	HAL_SPI_TransmitReceive(I->spi, &I->tx, &I->rx, 1, 55);
	MPU_NSS_HIGH;
	return 0;
}

/* 从MPU的寄存器中读取多个数据 */
uint8_t MPU_ReadBytes(IMU_TypeDef* I, uint8_t const regAddr, uint8_t* pData, uint8_t len) {
	MPU_NSS_LOW;
	I->tx = regAddr | 0x80;
	I->tx_buff[0] = I->tx;
	HAL_SPI_TransmitReceive(I->spi, &I->tx, &I->rx, 1, 55);
	HAL_SPI_TransmitReceive(I->spi, I->tx_buff, pData, len, 55);
	MPU_NSS_HIGH;
	return 0;
}

/* 设置imu的角速度测量范围 */
static uint8_t mpu_set_gyro_fsr(IMU_TypeDef* I, uint8_t fsr) {
	return MPU_WriteByte(I, MPU6500_GYRO_CONFIG, fsr << 3);
}

/* 设置imu的加速度测量范围 */
static uint8_t mpu_set_accel_fsr(IMU_TypeDef* I, uint8_t fsr) {
	return MPU_WriteByte(I, MPU6500_ACCEL_CONFIG, fsr << 3);
}

void IMU_GetData(IMU_TypeDef* I) {
	MPU_ReadBytes(I, MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
    I->ax = (double)mpu_data.ax / (double)4096;
    I->ay = (double)mpu_data.ay / (double)4096;
    I->az = (double)mpu_data.az / (double)4096;
}


/* 载入一个IMU单元 */
IMU_TypeDef IMU_Open(SPI_HandleTypeDef* hspi) {

	uint8_t i = 0;
	IMU_TypeDef I;
	uint8_t MPU6500_Init_Data[10][2] = {
			{ MPU6500_PWR_MGMT_1, 0x80 },     /* Reset Device */
			{ MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */
			{ MPU6500_PWR_MGMT_2, 0x00 },     /* Enable Acc & Gyro */
			{ MPU6500_CONFIG, 0x04 },         /* LPF 41Hz */
			{ MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */
			{ MPU6500_ACCEL_CONFIG, 0x10 },   /* +-2G */
			{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  Set Acc LPF */
			{ MPU6500_USER_CTRL, 0x20 } 	  /* Enable AUX */
	};

	HAL_Delay(100);
	I.spi = hspi;
	I.id = MPU_ReadByte(&I, MPU6500_WHO_AM_I);

	for (i = 0; i < 10; i++) {
		MPU_WriteByte(&I, MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		HAL_Delay(1);
	}

	mpu_set_gyro_fsr(&I, 3);
	mpu_set_accel_fsr(&I, 2);
	I.board_state = IMU_BOARD_DOWN;

	return I;
}

/* 检查IMU是否创建成功 */
uint8_t IMU_CheckSuccess(IMU_TypeDef* I) {
	if (I->id == MPU6500_ID) return HAL_OK;
	return HAL_ERROR;
}

/* IMU_BOARD_UP/DOWN */
void IMU_SetBoardState(IMU_TypeDef* I, uint8_t board_state) {
	I->board_state = board_state;
}
