#pragma once

#include <string>
#include <vector>

#include "solver/ins/INS.h"
#include "core/ins/InsStruct.h"
#include "io/ins/BinReader.h"

namespace Ins {

    struct VerifyTolerance {
        double latLonDeg = 1e-6;
        double hM = 2.0;
        double vMps = 1e-3;
        double attDeg = 1e-6;
    };

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
        NavState init;             // 对准结束时刻的初始状态（指导书给定，不重新对准）
        double startTime = 0.0;    // 对准结束时刻；0 表示从 IMU 文件首历元开始
        VerifyTolerance tol;
    };

    class Verifier {
    public:
        static constexpr std::size_t kImuStride = 7;       // IMU.bin：t, Δθ(3), Δv(3)
        static constexpr std::size_t kRefStride = 10;      // PureINS.bin：t, φ, λ, h, vN, vE, vD, roll, pitch, heading
        static constexpr std::size_t kMaxCurvePts = 4000;

        Verifier(const std::string &imuPath, const std::string &refPath, const VerifyOptions &opt);
        bool step();
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
        bool m_hasLast = false;
        VerifySample m_last;
    };
}
