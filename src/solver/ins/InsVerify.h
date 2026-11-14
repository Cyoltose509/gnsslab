#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "solver/ins/INS.h"
#include "core/ins/InsStruct.h"
#include "io/ins/BinReader.h"

namespace Ins {

    /// 四条验收判据
    struct VerifyTolerance {
        double latLonDeg = 1e-6;
        double hM = 2.0;
        double vMps = 1e-3;
        double attDeg = 1e-6;
    };

    /// 一个比对点的误差：位置换成北东天米制（度差在 1e-7 量级，米制才看得出手感），
    /// 速度按 NED，姿态按度。
    struct VerifySample {
        double t = 0.0;                                   // 相对起始时刻的秒数
        double dN = 0.0, dE = 0.0, dU = 0.0;              // m
        double dVn = 0.0, dVe = 0.0, dVd = 0.0;           // m/s
        double dRoll = 0.0, dPitch = 0.0, dHeading = 0.0; // deg
    };

    struct VerifyReport {
        std::size_t imuEpochs = 0;  // 参与机械编排的历元数
        std::size_t refEpochs = 0;  // 参考轨迹的记录数
        std::size_t compared = 0;   // 实际对齐上的点数
        double t0 = 0.0, t1 = 0.0;  // 比对区间的绝对时标 (s)

        double maxLatDeg = 0.0, maxLonDeg = 0.0, maxHM = 0.0, maxV = 0.0;
        double maxRollDeg = 0.0, maxPitchDeg = 0.0, maxHeadingDeg = 0.0;
        double maxHorizontalM = 0.0;  // 折成米的水平偏差，只作附注，不参与判定

        VerifyTolerance tol;
        bool pass = false;
        std::vector<VerifySample> series;  // 在线抽稀到 4000 点以内的曲线
    };

    struct VerifyOptions {
        InsInit init;              // 对准结束时刻的初始状态（指导书给定，不重新对准）
        double startTime = 0.0;    // 对准结束时刻；0 表示从 IMU 文件首历元开始
        VerifyTolerance tol;
    };

    /// 流式比对器：两个裸 bin 各按块预读，step() 推一条 IMU 增量并归并参考轨迹。
    /// 循环由调用方驱动（GUI 逐帧发布进度、CLI 一路跑到底），本层不回调界面。
    class Verifier {
    public:
        Verifier(const std::string &imuPath, const std::string &refPath, const VerifyOptions &opt);

        [[nodiscard]] bool ready() const { return m_ready; }

        [[nodiscard]] const std::string &error() const { return m_error; }

        /// 推进一条增量；返回 false 表示 IMU 流结束或读出错（查 error()）。
        bool step();

        /// 补上曲线末点并计算 pass，只在流结束后调一次。
        void finish();

        VerifyReport report;

    private:
        void compare(const NavState &ns, const double *rec);

        BinRecordReader m_imu, m_ref;
        INS m_ins;
        std::vector<double> m_refRec;
        double m_tPrev = 0.0, m_t0 = 0.0;
        std::size_t m_curveStride = 1, m_curveCounter = 0, m_imuEpochs = 0;
        bool m_refPending = false;
        bool m_ready = false;
        bool m_hasLast = false;
        VerifySample m_last;
        std::string m_error;
    };

    /// 一次性跑完整个验证（CLI / 自检用）；GUI 用 Verifier 自己驱动循环。
    bool verifySample(const std::string &imuPath, const std::string &refPath, const VerifyOptions &opt,
                      VerifyReport &out, std::string &error);
}
