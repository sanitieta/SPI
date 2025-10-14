//
// Created by xuhao on 2025/10/12.
//
#include "stm32f4xx_hal.h"
#include "IMU.h"

IMU bmi088;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        bmi088.update(0.001);
    }
}