#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "GnssStruct.h"
#include "EphemerisTable.h"

namespace SolveDriver {
    class DualObsPairing {
    public:
        struct IntervalEstimator {
            static constexpr int kWindow = 32; // 滑动窗口内的间隔样本数
            static constexpr int kMinSamples = 3; // 少于此数视为未就绪
            static constexpr double kMinDelta = 1e-6; // ≤此值视为重复历元，不计入
            static constexpr double kMaxSaneGap = 900.0; // ≥此值视为中断/时间回绕，不计入

            void reset();

            void observe(double epochSec);

            // 预置标称间隔(s)：>0 时 interval() 直接返回，不再从滑动窗口估计
            void force(const double intervalSec) { if (intervalSec > 0.0) mForced = intervalSec; }

            [[nodiscard]] double interval() const;

            [[nodiscard]] bool ready() const;

            static double pairWindow(const IntervalEstimator &a, const IntervalEstimator &b);

        private:
            bool mHave = false;
            double mLast = 0.0;
            double mForced = 0.0; // 用户声明的标称间隔(s)，0=按观测自适应估计
            std::vector<double> mDeltas;
        };

        using EphPtr = std::shared_ptr<EphemerisTable>;
        using Puller = std::function<bool(ObsData &out, EphPtr &eph)>;

        struct Pair {
            ObsData base, rover;
            EphPtr baseEph, roverEph;
        };

        enum class Action { Pair, AdvanceBase, AdvanceRover };

        DualObsPairing(Puller basePull, Puller roverPull);

        DualObsPairing() = default;

        void observeBase(double epochSec);

        void observeRover(double epochSec);

        // 用户声明标称采样间隔(s)：同时作用于基准/流动两路估计器，配对容差立即就绪
        void setInterval(const double intervalSec) {
            mBaseIv.force(intervalSec);
            mRoverIv.force(intervalSec);
        }

        [[nodiscard]] double pairWindow() const;

        bool next(Pair &out);

        static Action classify(const CommonTime &baseEpoch, const CommonTime &roverEpoch, double tol);

        static EphemerisTable *selectEph(const EphPtr &baseEph, const EphPtr &roverEph);

    private:
        bool ensurePrimed();

        bool stepBase();

        bool stepRover();

        Puller mBasePull, mRoverPull;
        IntervalEstimator mBaseIv, mRoverIv;
        ObsData mBase, mRover;
        EphPtr mBaseEph, mRoverEph;
        bool mHaveBase = false, mHaveRover = false;
    };
}
