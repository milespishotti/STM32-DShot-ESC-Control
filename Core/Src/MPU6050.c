/*
 * MPU6050.c
 *
 *  Created on: Jun 9, 2026
 *      Author: miles
 */


#include "MPU6050.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MPU6050_ADDR (0x68 << 1)
#define WHO_AM_I (0x75)
#define PWR_MGMT_1 (0x6B)
#define SMPLRT_DIV (0x19)
#define GYRO_CONFIG (0x1B)
#define ACCEL_CONFIG (0x1C)
#define ACCEL_XOUT_H (0x3B)
#define GYRO_XOUT_H (0x43)

float Ax, Ay, Az;
float Gx, Gy, Gz;
float angleX, angleY, angleZ;

float GxOffset = 0.0f;
float GyOffset = 0.0f;
float GzOffset = 0.0f;

float GxFiltered = 0.0f;
float GyFiltered = 0.0f;
float GzFiltered = 0.0f;

float dt = 0.25f;

extern I2C_HandleTypeDef hi2c1;

uint8_t MPU6050_Init(void)
{
    uint8_t check = 0;


    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I, 1, &check, 1, 50);
    printf("status=%d, check=%d\r\n", status, check);

    if (status != HAL_OK || check != 0x68)
    {
        printf("check failed\r\n");
        return 0;
    }

    printf("check passed\r\n");



    uint8_t Data;



    Data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1, 50);
    printf("configured pwrmgmt1\r\n");

    Data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, 50);
    printf("set sample rate to 1 kHz\r\n");

    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, 50);
    printf("config gyro max = 250\r\n");

    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, 50);
    printf("config accel max = 2g \r\n");


    printf("\r\n");
    return 1;

}


void MPU6050_Read (void)
{
    uint8_t RecDataAccel[6];
    uint8_t RecDataGyro[6];

    HAL_StatusTypeDef status1;
    HAL_StatusTypeDef status2;

    status1 = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, RecDataAccel, 6, 50);
    status2 = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H, 1, RecDataGyro, 6, 50);

    if (status1 != HAL_OK || status2 != HAL_OK)
    {
        printf("MPU read failed: accel = %d, gyro = %d\r\n", status1, status2);
        return;
    }


    int16_t Accel_X_Raw = (RecDataAccel[0] << 8 | RecDataAccel[1]);
    int16_t Accel_Y_Raw = (RecDataAccel[2] << 8 | RecDataAccel[3]);
    int16_t Accel_Z_Raw = (RecDataAccel[4] << 8 | RecDataAccel[5]);

    int16_t Gyro_X_Raw = (RecDataGyro[0] << 8 | RecDataGyro[1]);
    int16_t Gyro_Y_Raw = (RecDataGyro[2] << 8 | RecDataGyro[3]);
    int16_t Gyro_Z_Raw = (RecDataGyro[4] << 8 | RecDataGyro[5]);

    Ax = Accel_X_Raw / 16384.0;
    Ay = Accel_Y_Raw / 16384.0;
    Az = Accel_Z_Raw / 16384.0;

    Gx = Gyro_X_Raw / 131.0;
    Gy = Gyro_Y_Raw / 131.0;
    Gz = Gyro_Z_Raw / 131.0;
}

void Calibrate_MPU6050 (void)
{
    float Gx_Sum = 0;
    float Gy_Sum = 0;
    float Gz_Sum = 0;

    for (int i=0; i < 1000; i++)
    {
        MPU6050_Read();
        Gx_Sum += Gx;
        Gy_Sum += Gy;
        Gz_Sum += Gz;
        HAL_Delay(2);
    }

    float Gx_Avg = (Gx_Sum) / 1000.0f;
    float Gy_Avg = (Gy_Sum) / 1000.0f;
    float Gz_Avg = (Gz_Sum) / 1000.0f;

    GxOffset = Gx_Avg;
    GyOffset = Gy_Avg;
    GzOffset = Gz_Avg;
}

void Update_Angle (void)
{
    angleX += GxFiltered * dt;
    angleY += GyFiltered * dt;
    angleZ += GzFiltered * dt;
//
//    float accelAngleX = atan2(Ay, Az) * 180.0f / M_PI;
//    float accelAngleY = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) * 180.0f / M_PI;
//


    if (angleX > 180.0)
    {
        angleX -= 360.0;
    }
    else if (angleX < -180.0)
    {
        angleX += 360.0;
    }

    if (angleY > 180.0)
    {
        angleY -= 360.0;
    }
    else if (angleY < -180.0f)
    {
        angleY += 360.0;
    }

    if (angleZ > 180.0)
    {
        angleZ -= 360.0;
    }
    else if (angleZ < -180.0)
    {
        angleZ += 360.0;
    }

}




