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
    void get_euler_angle(float* roll, float* pitch, float* yaw);
    void update(float dt);

private:
    typedef struct EulerAngle {
        float roll = 0, pitch = 0, yaw = 0;
    } EulerAngle;

    typedef struct Gyro {
        float roll_rate_ = 0, pitch_rate_ = 0, yaw_rate_ = 0;
        float last_roll_rate_ = 0, last_pitch_rate_ = 0, last_yaw_rate_ = 0;
        float gyro_range_ = 0;
        uint8_t rx_gyro_data_[6]{ 0 };

        void gyro_calculate(const EulerAngle& euler);
    } Gyro;

    typedef struct Accel {
        float x = 0, y = 0, z = 0;
        float acc_range_ = 0;
        uint8_t rx_acc_data_[6]{ 0 };

        void acc_calculate();
    } Accel;

    void complement_calculate(float dt, float comp_alpha_ = 0.02);
    void kalman_calculate(float dt);
    Accel accel_;
    Gyro gyro_;
    EulerAngle euler_;
};

#endif //SPI_IMU_H