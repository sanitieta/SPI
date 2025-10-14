//
// Created by xuhao on 2025/10/12.
//

#ifndef SPI_IMU_H
#define SPI_IMU_H
#include "stm32f4xx_hal.h"

#define PI 3.14159265358979323846

class IMU {
public:
    void get_accel_xyz(float* accel_x, float* accel_y, float* accel_z);
    void get_gyro_xyz(float* gyro_x, float* gyro_y, float* gyro_z);
    void update(float dt);

private:
    typedef struct Accel {
        float x = 0, y = 0, z = 0;
        float acc_range_ = 0;
        uint8_t rx_acc_data_[6]{ 0 };

        void acc_calculate();
    } Accel;

    typedef struct Gyro {
        float wx = 0, wy = 0, wz = 0;
        float last_wx = 0, last_wy = 0, last_wz = 0;
        float gyro_range_ = 0;
        uint8_t rx_gyro_data_[6]{ 0 };

        void gyro_calculate();
    } Gyro;

    typedef struct EulerAngle {
        float yaw = 0, roll = 0, pitch = 0;
    } EulerAngle;

    void complement_calculate(float dt, float comp_alpha_ = 0.02);
    Accel accel_;
    Gyro gyro_;
    EulerAngle euler_;

};


#endif //SPI_IMU_H