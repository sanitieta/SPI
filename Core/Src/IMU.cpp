//
// Created by xuhao on 2025/10/12.
//
#include "bmi088.h"
#include "IMU.h"
#include <cmath>

// @param dt: time interval in seconds
void IMU::update(float dt) {
    static bool if_first = false;
    // 初始化
    if (!if_first) {
        accel_.acc_calculate();
        euler_.pitch = atan2f(accel_.y, accel_.z) * 180.0f / PI;
        euler_.roll = atan2f(-accel_.x, sqrt(accel_.y * accel_.y + accel_.z * accel_.z)) * 180.8f / PI; // 加速度计单位为 mg
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
    // 1. 设置/读取acc0x41寄存器中的量程range参数，并换算为量程系数
    uint8_t raw_range;
    bmi088_accel_read_reg(0x41, &raw_range, 1);
    switch (raw_range) {
        case 0x00:
            acc_range_ = 3.0f;
            break;
        case 0x01:
            acc_range_ = 6.0f;
            break;
        case 0x02:
            acc_range_ = 12.0f;
            break;
        case 0x03:
            acc_range_ = 24.0f;
            break;
        default:
            acc_range_ = 0.0f;
    }
    // 2. 读取acc0x12寄存器中的6位acc数据
    bmi088_accel_read_reg(0x12, rx_acc_data_, 6);
    // 3. 用量程系数将原始数据转换为常用单位
    x = static_cast<int16_t>(rx_acc_data_[1] << 8 | rx_acc_data_[0]) / 32768.0f * 1000.0f *
        pow(2, raw_range + 1) * 1.5f;
    y = static_cast<int16_t>(rx_acc_data_[3] << 8 | rx_acc_data_[2]) / 32768.0f * 1000.0f *
        pow(2, raw_range + 1) * 1.5f;
    z = static_cast<int16_t>(rx_acc_data_[5] << 8 | rx_acc_data_[4]) / 32768.0f * 1000.0f *
        pow(2, raw_range + 1) * 1.5f;
}

void IMU::Gyro::gyro_calculate(const EulerAngle& euler) {
    // 1. 设置/读取acc0x0F寄存器中的量程range参数，并换算为量程系数
    uint8_t raw_range;
    static float transition_matrix[3][3] = { 0 };
    bmi088_gyro_read_reg(0x0F, &raw_range, 1);
    switch (raw_range) {
        case 0x00:
            gyro_range_ = 2000.0f;
            break;
        case 0x01:
            gyro_range_ = 1000.0f;
            break;
        case 0x02:
            gyro_range_ = 500.0f;
            break;
        case 0x03:
            gyro_range_ = 250.0f;
            break;
        case 0x04:
            gyro_range_ = 125.0f;
            break;
        default:
            gyro_range_ = 0.0f;
    }
    // 2. 读取acc0x12寄存器中的6位gyro数据
    bmi088_gyro_read_reg(0x02, rx_gyro_data_, 6);
    // 3. 用量程系数将原始数据转换为常用单位 并使用中值积分算出角度 注意坐标系变换
    last_roll_rate_ = roll_rate_;
    last_pitch_rate_ = pitch_rate_;
    last_yaw_rate_ = yaw_rate_;

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
    float imu_wx = static_cast<int16_t>(rx_gyro_data_[1] << 8 | rx_gyro_data_[0]) / 32767.0f * gyro_range_;
    float imu_wy = static_cast<int16_t>(rx_gyro_data_[3] << 8 | rx_gyro_data_[2]) / 32767.0f * gyro_range_;
    float imu_wz = static_cast<int16_t>(rx_gyro_data_[5] << 8 | rx_gyro_data_[4]) / 32767.0f * gyro_range_;
    roll_rate_ = transition_matrix[0][0] * imu_wx + transition_matrix[0][1] * imu_wy + transition_matrix[0][2] * imu_wz;
    pitch_rate_ = transition_matrix[1][0] * imu_wx + transition_matrix[1][1] * imu_wy + transition_matrix[1][2] *
        imu_wz;
    yaw_rate_ = transition_matrix[2][0] * imu_wx + transition_matrix[2][1] * imu_wy + transition_matrix[2][2] * imu_wz;
}

void IMU::complement_calculate(float dt, float comp_alpha_) {
    float acc_pitch = atan2f(-accel_.x, sqrt(accel_.y * accel_.y + accel_.z * accel_.z)) * 180.0f / PI;
    float acc_roll = atan2f(accel_.y, accel_.z) * 180.0f / PI;
    // 中值积分
    float gyro_pitch = euler_.pitch + (gyro_.last_pitch_rate_ + gyro_.pitch_rate_) / 2.0f * dt;
    float gyro_roll = euler_.roll + (gyro_.last_roll_rate_ + gyro_.roll_rate_) / 2.0f * dt;

    euler_.yaw += (gyro_.last_yaw_rate_ + gyro_.yaw_rate_) / 2.0f * dt; // yaw角不使用互补滤波
    euler_.pitch = (1.0f - comp_alpha_) * gyro_pitch + comp_alpha_ * acc_pitch;
    euler_.roll = (1.0f - comp_alpha_) * gyro_roll + comp_alpha_ * acc_roll;

    // 保护pitch角，防止出现tan(pitch)无穷大的情况
    if (fabsf(euler_.pitch) < 1e-2f) {
        euler_.pitch = (euler_.pitch > 0 ? (90.0f - 1e-1f) : (-90.0f + 1e-3f));
    }
}

void IMU::kalman_calculate(float dt) {}