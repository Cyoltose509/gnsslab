#include "DualObsPairing.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <vector>

namespace SolveDriver {
    void DualObsPairing::IntervalEstimator::reset() {
        mHave = false;
        mLast = 0.0;
        mDeltas.clear();
    }

    void DualObsPairing::IntervalEstimator::observe(const double epochSec) {
        if (mHave) {
            const double d = epochSec - mLast;
            if (d > kMinDelta && d < kMaxSaneGap) {
                mDeltas.push_back(d);
                if (static_cast<int>(mDeltas.size()) > kWindow)
                    mDeltas.erase(mDeltas.begin());
            }
        }
        mLast = epochSec;
        mHave = true;
    }

    [[nodiscard]] double DualObsPairing::IntervalEstimator::interval() const {
        if (static_cast<int>(mDeltas.size()) < kMinSamples) return 0.0;
        std::map<long long, int> cnt;
        for (const double d: mDeltas) ++cnt[llround(d * 1000.0)];
        const auto best = std::max_element(
            cnt.begin(), cnt.end(),
            [](const std::pair<const long long, int> &a,
               const std::pair<const long long, int> &b) {
                return a.second < b.second;
            });
        if (best->second >= 2) return static_cast<double>(best->first) / 1000.0;
        // 众数不可靠（间隔抖动到 1ms 量级分散）：退化为中位数
        std::vector<double> v(mDeltas);
        std::nth_element(v.begin(), v.begin() + static_cast<long>(v.size()) / 2, v.end());
        return v[v.size() / 2];
    }

    [[nodiscard]] bool DualObsPairing::IntervalEstimator::ready() const { return interval() > 0.0; }

    double DualObsPairing::IntervalEstimator::pairWindow(const IntervalEstimator &a, const IntervalEstimator &b) {
        const double ia = a.interval();
        const double ib = b.interval();
        if (ia <= 0.0 || ib <= 0.0) return 0.0;
        return 0.5 * (ia < ib ? ia : ib);
    }

    // ---------------- DualObsPairing ----------------
    DualObsPairing::DualObsPairing(Puller basePull, Puller roverPull)
        : mBasePull(std::move(basePull)), mRoverPull(std::move(roverPull)) {
    }

    void DualObsPairing::observeBase(double epochSec) { mBaseIv.observe(epochSec); }
    void DualObsPairing::observeRover(double epochSec) { mRoverIv.observe(epochSec); }

    [[nodiscard]] double DualObsPairing::pairWindow() const {
        return IntervalEstimator::pairWindow(mBaseIv, mRoverIv);
    }

    bool DualObsPairing::next(Pair &out) {
        for (;;) {
            if (!ensurePrimed()) return false;
            const double tol = IntervalEstimator::pairWindow(mBaseIv, mRoverIv);
            switch (classify(mBase.epoch, mRover.epoch, tol)) {
                case Action::Pair:
                    out.base = mBase;
                    out.rover = mRover;
                    out.baseEph = std::move(mBaseEph);
                    out.roverEph = std::move(mRoverEph);
                    // 已消费：下一轮重新拉取，避免重复配对同一历元
                    mHaveBase = mHaveRover = false;
                    return true;
                case Action::AdvanceBase:
                    if (!stepBase()) return false; // 基准站历元更旧，丢弃重读
                    break;
                case Action::AdvanceRover:
                    if (!stepRover()) return false; // 流动站历元更旧，丢弃重读
                    break;
            }
        }
    }

    DualObsPairing::Action DualObsPairing::classify(const CommonTime &baseEpoch, const CommonTime &roverEpoch, double tol) {
        if (std::fabs(baseEpoch - roverEpoch) <= tol) return Action::Pair;
        // |Δ| > tol 时等价于 (base < rover - tol) 的判定（差值已大于 tol，二者大小关系无歧义）
        return (baseEpoch < roverEpoch) ? Action::AdvanceBase : Action::AdvanceRover;
    }

    EphemerisTable *DualObsPairing::selectEph(const EphPtr &baseEph, const EphPtr &roverEph) {
        if (baseEph && !baseEph->empty()) return baseEph.get();
        if (roverEph && !roverEph->empty()) return roverEph.get();
        return baseEph ? baseEph.get() : roverEph.get();
    }

    bool DualObsPairing::ensurePrimed() {
        if (!mHaveBase && !stepBase()) return false;
        if (!mHaveRover && !stepRover()) return false;
        return mHaveBase && mHaveRover;
    }

    bool DualObsPairing::stepBase() {
        EphPtr eph;
        if (!mBasePull(mBase, eph)) {
            mHaveBase = false;
            return false;
        }
        mBaseEph = std::move(eph);
        mBaseIv.observe(mBase.weekSecond.sow); // 估采样间隔 / 配对窗口
        mHaveBase = true;
        return true;
    }

    bool DualObsPairing::stepRover() {
        EphPtr eph;
        if (!mRoverPull(mRover, eph)) {
            mHaveRover = false;
            return false;
        }
        mRoverEph = std::move(eph);
        mRoverIv.observe(mRover.weekSecond.sow);
        mHaveRover = true;
        return true;
    }
}
