#include "main.h"
#include "IMU.h"

extern I2C_HandleTypeDef hi2c3;
extern uint16_t IMU_address;
extern uint8_t IMU_Data_Ready;


uint8_t XL_Mode = 0x60;
uint8_t G_Mode =  0x60;
uint8_t IMU_IT = 0x03;
uint8_t buf[2];

void IMU_init(){
	HAL_I2C_Mem_Write  ( &hi2c3 ,IMU_address, 0x10, 1, &XL_Mode, 1, 10000); // acc init
	HAL_I2C_Mem_Write  ( &hi2c3 ,IMU_address, 0x11, 1, &G_Mode, 1, 10000); //gyro init
	HAL_I2C_Mem_Write  ( &hi2c3 ,IMU_address, 0x0D, 1, &IMU_IT, 1, 10000); //IT init
}

uint16_t get_ACC_X(){
	HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x1E, 1, &IMU_Data_Ready, 1, 10000); //STATUS_REG (1Eh)
	if((IMU_Data_Ready & 0x01) == 1){
		HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x28, 1, buf, 2, 10000); // OUTX_L_XL (28h)
	}
	uint16_t ACC_X = ((uint16_t)buf[2]<<8)+buf[1];
	return ACC_X;
}
