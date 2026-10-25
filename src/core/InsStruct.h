#pragma once
#include <cstdint>
#include <vector>


// 单条 IMU 原始采样
struct ImuSample {
    int gnssWeek = 0;
    double tow = 0.0;                 // GPS 周内秒
    int32_t gx = 0, gy = 0, gz = 0;   // 陀螺 角增量原始计数（机体 x/y/z）
    int32_t ax = 0, ay = 0, az = 0;   // 加表 速度增量原始计数（机体 x/y/z）
};

using ImuLog = std::vector<ImuSample>;

// 单历元姿态（欧拉角，rad）
struct Attitude {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

// 带时间戳的姿态采样
struct AttitudeSample {
    double tow = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};
