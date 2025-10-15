//
// Created by xuhao on 2025/10/12.
//
#include "bmi088.h"
#include "IMU.h"
#include "IST8310.h"
#include <cmath>

// @param dt: time interval in seconds
void IMU::update(float dt) {
    static bool if_first = false;
    // 初始化
    if (!if_first) {
        for (size_t i = 0; i < 10; i++) {
            IST8310ReadMagData(nullptr, nullptr, nullptr);
            bmi088_gyro_read_data(nullptr, nullptr, nullptr, nullptr); // 读几次丢掉前几次不稳定的数据
            bmi088_accel_read_data(nullptr, nullptr, nullptr, nullptr);
            for (volatile int j = 0; j < 5000; j++); // 简单等待
        }

        accel_.acc_calculate();
        euler_.pitch = atan2f(accel_.y, accel_.z) * 180.0f / PI;
        euler_.roll = atan2f(-accel_.x, sqrt(accel_.y * accel_.y + accel_.z * accel_.z)) * 180.8f / PI; // 加速度计单位为 mg

        compass_.compass_calculate();
        euler_.yaw = 0;
        gyro_.gyro_calculate(this->euler_);
        if_first = true;
    }
    accel_.acc_calculate();
    gyro_.gyro_calculate(this->euler_);
    complement_calculate(dt);
}

void IMU::get_accel_xyz(float* accel_x, float* accel_y, float* accel_z) {
    *accel_x = accel_.x;
    *accel_y = accel_.y;
    *accel_z = accel_.z;
}

void IMU::get_gyro_xyz(float* gyro_x, float* gyro_y, float* gyro_z) {
    *gyro_x = gyro_.roll_rate_;
    *gyro_y = gyro_.pitch_rate_;
    *gyro_z = gyro_.yaw_rate_;
}

void IMU::get_euler_angle(float* roll, float* pitch, float* yaw) {
    *roll = euler_.roll;
    *pitch = euler_.pitch;
    *yaw = euler_.yaw;
}

void IMU::Accel::acc_calculate() {
    bmi088_accel_read_data(&acc_range_, &x, &y, &z);
    acc_pitch_ = atan2f(-x, sqrt(y * y + z * z)) * 180.0f / PI;
    acc_roll_ = atan2f(y, z) * 180.0f / PI;
}

void IMU::Gyro::gyro_calculate(const EulerAngle& euler) {
    // 0. 更新last值
    last_roll_rate_ = roll_rate_;
    last_pitch_rate_ = pitch_rate_;
    last_yaw_rate_ = yaw_rate_;
    // 1. 获取陀螺仪数据
    float imu_wx, imu_wy, imu_wz;
    bmi088_gyro_read_data(&gyro_range_, &imu_wx, &imu_wy, &imu_wz);
    // 2. 使用中值积分算出角度 注意坐标系变换
    static float transition_matrix[3][3] = { 0 };
    float roll_rad = euler.roll * PI / 180.0f;
    float pitch_rad = euler.pitch * PI / 180.0f; // 传入的pitch已经经过保护:tan(pitch)不会无穷大
    transition_matrix[0][0] = 1;
    transition_matrix[0][1] = sinf(roll_rad) * tanf(pitch_rad);
    transition_matrix[0][2] = cosf(roll_rad) * tanf(pitch_rad);
    transition_matrix[1][0] = 0;
    transition_matrix[1][1] = cosf(roll_rad);
    transition_matrix[1][2] = -sinf(roll_rad);
    transition_matrix[2][0] = 0;
    transition_matrix[2][1] = sinf(roll_rad) / cosf(pitch_rad);
    transition_matrix[2][2] = cosf(roll_rad) / cosf(pitch_rad);

    roll_rate_ = transition_matrix[0][0] * imu_wx + transition_matrix[0][1] * imu_wy + transition_matrix[0][2] * imu_wz;
    pitch_rate_ = transition_matrix[1][0] * imu_wx + transition_matrix[1][1] * imu_wy + transition_matrix[1][2] *
        imu_wz;
    yaw_rate_ = transition_matrix[2][0] * imu_wx + transition_matrix[2][1] * imu_wy + transition_matrix[2][2] * imu_wz;
}

void IMU::Compass::compass_calculate() {
    IST8310ReadMagData(&x, &y, &z);
}


void IMU::complement_calculate(float dt, float comp_alpha_) {
    // 中值积分
    float gyro_pitch = euler_.pitch + (gyro_.last_pitch_rate_ + gyro_.pitch_rate_) / 2.0f * dt;
    float gyro_roll = euler_.roll + (gyro_.last_roll_rate_ + gyro_.roll_rate_) / 2.0f * dt;

    euler_.yaw += (gyro_.last_yaw_rate_ + gyro_.yaw_rate_) / 2.0f * dt; // yaw角不使用互补滤波
    euler_.pitch = (1.0f - comp_alpha_) * gyro_pitch + comp_alpha_ * accel_.acc_pitch_;
    euler_.roll = (1.0f - comp_alpha_) * gyro_roll + comp_alpha_ * accel_.acc_roll_;

    // 保护pitch角，防止出现tan(pitch)无穷大的情况
    if (fabsf(fabsf(euler_.pitch) - 90.0f) < 1e-2f) {
        euler_.pitch = (euler_.pitch > 0 ? (90.0f - 1e-1f) : (-90.0f + 1e-3f));
    }
}

void IMU::kalman_calculate(float dt) {}