#include "Controller.h"
#include "MPU6050.h"
#include <math.h>

extern uint32_t pulse;
extern TIM_HandleTypeDef htim2;

float controller_angle = 0.0f;
float controller_error = 0.0f;
float controller_gyro = 0.0f;
float controller_integral_term = 0.0f;
float controller_correction = 0.0f;

static int filter_initialized = 0;

static uint16_t throttle_cmd = 0;

static const uint16_t min_throttle = 100;
static const uint16_t max_throttle = 650;
static const uint16_t base_throttle = 410;

static const float target_angle = 5.5f;

static const float kp_up = 10.0f;
static const float kd_up = 1.90f;

static const float kp_down = 10.5f;
static const float kd_down = 2.20f;

static const float ki = 0.75f;
static const float control_dt = 0.01f;
static const float integral_limit = 100.0f;

static float integral = 0.0f;

void Controller_Init(void)
{
    angleX = 0.0f;
    angleY = 0.0f;
    angleZ = 0.0f;

    filter_initialized = 0;
    integral = 0.0f;
    throttle_cmd = 0;
}

void Controller_UpdateSensors(void)
{
    MPU6050_Read();

    Gx = Gx - GxOffset;
    Gy = Gy - GyOffset;
    Gz = Gz - GzOffset;

    if (!filter_initialized)
    {
        GxFiltered = Gx;
        GyFiltered = Gy;
        GzFiltered = Gz;
        filter_initialized = 1;
    }
    else
    {
        GxFiltered = (0.9f * GxFiltered) + (0.1f * Gx);
        GyFiltered = (0.9f * GyFiltered) + (0.1f * Gy);
        GzFiltered = (0.9f * GzFiltered) + (0.1f * Gz);
    }

    float accelAngleX = atan2(Ay, Az) * 180.0f / M_PI;
    float accelAngleY = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) * 180.0f / M_PI;

    angleX = 0.98f * (angleX + GxFiltered * control_dt) + 0.02f * accelAngleX;
    angleY = 0.98f * (angleY + GyFiltered * control_dt) + 0.02f * accelAngleY;

    pulse = (angleX * 999) / 180;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);

    controller_error = target_angle - angleX;
    controller_angle = angleX;
    controller_gyro = GxFiltered;

}

uint16_t Controller_ComputeThrottle(void)
{
    controller_error = target_angle - angleX;

    float P = 0.0f;
    float D = 0.0f;
    float I = 0.0f;

    if (controller_error < 0)
    {
        P = kp_down * controller_error;
        D = -kd_down * GxFiltered;
    }
    else
    {
        P = kp_up * controller_error;
        D = -kd_up * GxFiltered;
    }

    if (fabs(controller_error) < 8.0f)
    {
        integral += controller_error * control_dt;
    }
    else
    {
        integral = 0.0f;
    }

    if (integral > integral_limit) integral = integral_limit;
    if (integral < -integral_limit) integral = -integral_limit;

    I = ki * integral;

    controller_integral_term = I;
    controller_correction = P + D + I;

    throttle_cmd = base_throttle + controller_correction;

    if (throttle_cmd > max_throttle) throttle_cmd = max_throttle;
    if (throttle_cmd < min_throttle) throttle_cmd = min_throttle;

    return throttle_cmd;
}
