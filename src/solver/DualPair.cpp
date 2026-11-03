#include "DualPair.h"

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <vector>

namespace Observer {

void DualPair::IntervalEstimator::reset() {
    mHave = false;
    mLast = 0.0;
    mForced = 0.0;
    mDeltas.clear();
}

void DualPair::IntervalEstimator::observe(const double epochSec) {
    if (mHave) {
        if (const double d = epochSec - mLast; d > kMinDelta && d < kMaxSaneGap) {
            mDeltas.push_back(d);
            if (static_cast<int>(mDeltas.size()) > kWindow)
                mDeltas.erase(mDeltas.begin());
        }
    }
    mLast = epochSec;
    mHave = true;
}

[[nodiscard]] double DualPair::IntervalEstimator::interval() const {
    if (mForced > 0.0) return mForced;
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

    std::vector v(mDeltas);
    std::nth_element(v.begin(), v.begin() + static_cast<long>(v.size()) / 2, v.end());
    return v[v.size() / 2];
}

[[nodiscard]] bool DualPair::IntervalEstimator::ready() const { return interval() > 0.0; }

double DualPair::IntervalEstimator::pairWindow(const IntervalEstimator &a, const IntervalEstimator &b) {
    const double ia = a.interval();
    const double ib = b.interval();
    if (ia <= 0.0 || ib <= 0.0) return 0.0;
    return 0.5 * (ia < ib ? ia : ib);
}

DualPair::DualPair(Puller basePull, Puller roverPull)
    : mBasePull(std::move(basePull)), mRoverPull(std::move(roverPull)) {
}

void DualPair::observeBase(const double epochSec) { mBaseIv.observe(epochSec); }
void DualPair::observeRover(const double epochSec) { mRoverIv.observe(epochSec); }

[[nodiscard]] double DualPair::pairWindow() const {
    return IntervalEstimator::pairWindow(mBaseIv, mRoverIv);
}

bool DualPair::next(Pair &out) {
    for (;;) {
        if (!ensurePrimed()) return false;
        switch (const double tol = IntervalEstimator::pairWindow(mBaseIv, mRoverIv);
            classify(mBase.epoch, mRover.epoch, tol)) {
            case Action::Pair:
                out.base = mBase;
                out.rover = mRover;
                out.baseEph = std::move(mBaseEph);
                out.roverEph = std::move(mRoverEph);
                mHaveBase = mHaveRover = false;
                return true;
            case Action::AdvanceBase:
                if (!stepBase()) return false;
                break;
            case Action::AdvanceRover:
                if (!stepRover()) return false;
                break;
        }
    }
}

DualPair::Action DualPair::classify(const CommonTime &baseEpoch, const CommonTime &roverEpoch, const double tol) {
    if (std::fabs(baseEpoch - roverEpoch) <= tol) return Action::Pair;
    return baseEpoch < roverEpoch ? Action::AdvanceBase : Action::AdvanceRover;
}

EphemerisTable *DualPair::selectEph(const EphPtr &baseEph, const EphPtr &roverEph) {
    if (baseEph && !baseEph->empty()) return baseEph.get();
    if (roverEph && !roverEph->empty()) return roverEph.get();
    return baseEph ? baseEph.get() : roverEph.get();
}

bool DualPair::ensurePrimed() {
    if (!mHaveBase && !stepBase()) return false;
    if (!mHaveRover && !stepRover()) return false;
    return mHaveBase && mHaveRover;
}

bool DualPair::stepBase() {
    EphPtr eph;
    if (!mBasePull(mBase, eph)) {
        mHaveBase = false;
        return false;
    }
    mBaseEph = std::move(eph);
    mBaseIv.observe(mBase.weekSecond.sow);
    mHaveBase = true;
    return true;
}

bool DualPair::stepRover() {
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
