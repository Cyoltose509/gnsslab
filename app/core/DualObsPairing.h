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
            static constexpr double kMaxSaneGap = 900.0; // ≥15min 视为中断/时间回绕，不计入

            void reset();

            /// 按时间顺序喂入该路读到的每个历元时间（秒）
            void observe(double epochSec);

            [[nodiscard]] double interval() const;

            [[nodiscard]] bool ready() const;

            static double pairWindow(const IntervalEstimator &a, const IntervalEstimator &b);

        private:
            bool mHave = false;
            double mLast = 0.0;
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
