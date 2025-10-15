//
// Created by xuhao on 2025/10/12.
//
#include "bmi088.h"
#include "IMU.h"
#include "IST8310.h"
#include <cmath>

#ifndef PI
#define PI 3.14159265358979323846
#endif
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)

// @param dt: time interval in seconds
void IMU::update(float dt) {
    static bool if_first = true;
    static size_t i = 0;
    // 初始化
    if (if_first) {
        imu_init();
        if_first = false;
    }
    accel_.acc_calculate();
    gyro_.gyro_calculate(this->euler_);
    if (i % 5 == 0) {
        compass_.compass_calculate(this->euler_); // 磁力计频率低一些 200Hz
        i = 0;
    }
    i++;
    complement_calculate(dt);
}

// 输出接口依旧用角度，方便上层使用
void IMU::get_euler_angle(float* roll, float* pitch, float* yaw) {
    *roll = euler_.roll_degree_;
    *pitch = euler_.pitch_degree_;
    *yaw = euler_.yaw_degree_;
}

// ---------------- 加速度计 ----------------
void IMU::Accel::acc_calculate() {
    bmi088_accel_read_data(&acc_range_, &ax_, &ay_, &az_);
    // 输出弧度
    acc_pitch_ = atan2f(-ax_, sqrtf(ay_ * ay_ + az_ * az_));
    acc_roll_ = atan2f(ay_, az_);
}

// ---------------- 陀螺仪 ----------------
void IMU::Gyro::gyro_calculate(const EulerAngle& euler) {
    // 0. 更新last值
    last_roll_rate_ = roll_rate_;
    last_pitch_rate_ = pitch_rate_;
    last_yaw_rate_ = yaw_rate_;

    // 1. 获取陀螺仪数据（单位：rad/s）
    float imu_wx, imu_wy, imu_wz;
    bmi088_gyro_read_data(&gyro_range_, &imu_wx, &imu_wy, &imu_wz);

    // 2. 坐标系变换（机体系 → 地面系）
    float roll = euler.roll_;
    float pitch = euler.pitch_;
    float R[3][3] = { 0 };
    R[0][0] = 1.0f;
    R[0][1] = sinf(roll) * tanf(pitch);
    R[0][2] = cosf(roll) * tanf(pitch);
    R[1][0] = 0.0f;
    R[1][1] = cosf(roll);
    R[1][2] = -sinf(roll);
    R[2][0] = 0.0f;
    R[2][1] = sinf(roll) / cosf(pitch);
    R[2][2] = cosf(roll) / cosf(pitch);

    roll_rate_ = R[0][0] * imu_wx + R[0][1] * imu_wy + R[0][2] * imu_wz;
    pitch_rate_ = R[1][0] * imu_wx + R[1][1] * imu_wy + R[1][2] * imu_wz;
    yaw_rate_ = R[2][0] * imu_wx + R[2][1] * imu_wy + R[2][2] * imu_wz;
}

// ---------------- 磁力计 ----------------
void IMU::Compass::compass_calculate(const EulerAngle& euler) {
    IST8310ReadMagData(&mx_, &my_, &mz_);

    float roll = euler.roll_;
    float pitch = euler.pitch_;
    float R[2][3] = { 0 };
    R[0][0] = cosf(pitch);
    R[0][1] = sinf(roll) * sinf(pitch);
    R[0][2] = cosf(roll) * sinf(pitch);
    R[1][0] = 0.0f;
    R[1][1] = cosf(roll);
    R[1][2] = -sinf(roll);

    // 坐标转换
    float mx_h = R[0][0] * mx_ + R[0][1] * my_ + R[0][2] * mz_;
    float my_h = R[1][0] * mx_ + R[1][1] * my_ + R[1][2] * mz_;

    compass_yaw_ = atan2f(-my_h, mx_h); // 输出为弧度
}

// ---------------- 初始化 ----------------
void IMU::imu_init() {
    // 空读几次丢掉前几次不稳定的数据
    for (size_t i = 0; i < 10; i++) {
        IST8310ReadMagData(nullptr, nullptr, nullptr);
        bmi088_gyro_read_data(nullptr, nullptr, nullptr, nullptr);
        bmi088_accel_read_data(nullptr, nullptr, nullptr, nullptr);
        for (volatile int j = 0; j < 5000; j++);
    }

    // 初始姿态
    accel_.acc_calculate();
    euler_.pitch_ = accel_.acc_pitch_;
    euler_.roll_ = accel_.acc_roll_;

    compass_.compass_calculate(euler_);
    euler_.yaw_ = compass_.compass_yaw_;

    gyro_.gyro_calculate(this->euler_);
}

// ---------------- 互补滤波 ----------------
void IMU::complement_calculate(float dt, float comp_alpha_) {
    // 中值积分
    float gyro_pitch = euler_.pitch_ + (gyro_.last_pitch_rate_ + gyro_.pitch_rate_) / 2.0f * dt;
    float gyro_roll = euler_.roll_ + (gyro_.last_roll_rate_ + gyro_.roll_rate_) / 2.0f * dt;

    euler_.yaw_ += (gyro_.last_yaw_rate_ + gyro_.yaw_rate_) / 2.0f * dt; // yaw 不互补
    euler_.pitch_ = (1.0f - comp_alpha_) * gyro_pitch + comp_alpha_ * accel_.acc_pitch_;
    euler_.roll_ = (1.0f - comp_alpha_) * gyro_roll + comp_alpha_ * accel_.acc_roll_;

    // 限制 pitch，防止 tan(pitch) → ∞
    if (fabsf(cosf(euler_.pitch_)) < 1e-3f) {
        euler_.pitch_ = (euler_.pitch_ > 0.0f) ? (PI / 2.0f - 1e-3f) : (-PI / 2.0f + 1e-3f);
    }

    euler_.roll_degree_ = euler_.roll_ * RAD2DEG;
    euler_.pitch_degree_ = euler_.pitch_ * RAD2DEG;
    euler_.yaw_degree_ = euler_.yaw_ * RAD2DEG;
}

void IMU::kalman_calculate(float dt) {}