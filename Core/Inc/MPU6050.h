/*
 * MPU6050.h
 *
 *  Created on: Jun 9, 2026
 *      Author: miles
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"

uint8_t MPU6050_Init(void);
void MPU6050_Read(void);
void Calibrate_MPU6050(void);
void Update_Angle(void);

extern float Ax, Ay, Az;
extern float Gx, Gy, Gz;
extern float angleX, angleY, angleZ;

extern float GxFiltered, GyFiltered, GzFiltered;
extern float GxOffset, GyOffset, GzOffset;

extern float dt;


#endif /* INC_MPU6050_H_ */
