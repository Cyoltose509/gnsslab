#include "Aligner.h"

#include <algorithm>
#include <cmath>

#include "MathUtils.h"

namespace {
    struct BlockMeans {
        std::vector<Eigen::Vector3d> acc;
        std::vector<Eigen::Vector3d> gyr;
    };

    BlockMeans collectBlockMeans(const ImuLog &log, const std::size_t begin, const std::size_t end,
                                 const std::size_t w) {
        BlockMeans out;
        if (w == 0) return out;
        for (std::size_t b = begin; b + w <= end; b += w) {
            Eigen::Vector3d acc = Eigen::Vector3d::Zero(), gyr = Eigen::Vector3d::Zero();
            for (std::size_t i = b; i < b + w; ++i) {
                acc += Eigen::Vector3d(log[i].ax, log[i].ay, log[i].az);
                gyr += Eigen::Vector3d(log[i].gx, log[i].gy, log[i].gz);
            }
            out.acc.push_back(acc / static_cast<double>(w));
            out.gyr.push_back(gyr / static_cast<double>(w));
        }
        return out;
    }

    Eigen::Vector3d medianOf(std::vector<Eigen::Vector3d> v) {
        if (v.empty()) return Eigen::Vector3d::Zero();
        const auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
        Eigen::Vector3d m;
        for (int k = 0; k < 3; ++k) {
            std::nth_element(v.begin(), mid, v.end(),
                             [k](const Eigen::Vector3d &a, const Eigen::Vector3d &b) { return a[k] < b[k]; });
            m[k] = (*mid)[k];
        }
        return m;
    }

    // 逐块均值取中位数：短时搬动、触碰只污染少数块，中位数把它们挡在估计之外。
    DCM alignMeanDCM(const ImuLog &log,
                      const std::size_t begin, const std::size_t end, const SiteParams &sp,
                      Eigen::Vector3d &accMean, Eigen::Vector3d &gyrMean) {
        accMean = Eigen::Vector3d::Zero();
        gyrMean = Eigen::Vector3d::Zero();
        if (end <= begin) return DCM();

        const std::size_t n = end - begin;
        const std::size_t w = std::min(Aligner::kStatBlock, n);
        if (const BlockMeans bm = collectBlockMeans(log, begin, end, w); bm.gyr.size() >= 3) {
            accMean = medianOf(bm.acc);
            gyrMean = medianOf(bm.gyr);
            return alignDCM(-accMean, gyrMean, sp);
        }

        for (std::size_t i = begin; i < end; ++i) {
            accMean += Eigen::Vector3d(log[i].ax, log[i].ay, log[i].az);
            gyrMean += Eigen::Vector3d(log[i].gx, log[i].gy, log[i].gz);
        }
        accMean /= static_cast<double>(n);
        gyrMean /= static_cast<double>(n);
        return alignDCM(-accMean, gyrMean, sp);
    }
}

StaticSpan Aligner::detectStatic(const ImuLog &log, const ImuScale &scale) {
    const double rate = scale.rateHz;
    const double lsbToRad = scale.gyroRadPerLsb * rate;   // LSB → rad/s
    const auto w = static_cast<std::size_t>(std::lround(kBlockSec * rate));
    if (!(lsbToRad > 0.0) || w == 0) return {};

    const auto [acc, gyr] = collectBlockMeans(log, 0, log.size(), w);
    const std::size_t n = gyr.size();
    if (n < std::max<std::size_t>(static_cast<std::size_t>(std::lround(kMinStaticSec * rate)) / w, 2)) return {};

    constexpr double tol = kRotateFrac * OMEGA_EARTH;
    std::vector over(n, false);
    for (std::size_t k = 1; k < n; ++k) {
        const std::size_t lo = k > kRefBlocks ? k - kRefBlocks : 0;
        const auto from = static_cast<std::ptrdiff_t>(lo);
        const auto to = static_cast<std::ptrdiff_t>(k);
        const Eigen::Vector3d ref = medianOf({gyr.begin() + from, gyr.begin() + to});
        const double gRef = medianOf({acc.begin() + from, acc.begin() + to}).norm();
        over[k] = (gyr[k] - gyr[k - 1]).norm() * lsbToRad > tol ||
                  (gyr[k] - ref).norm() * lsbToRad > tol ||
                  (gRef > 0.0 && std::fabs(acc[k].norm() / gRef - 1.0) > kAccelFrac);
    }

    // 单块超限是触碰、说话这类扰动，持续 kMoveRun 块以上才认作真的在动。
    std::vector moving(n, false);
    for (std::size_t k = 0; k < n;) {
        if (!over[k]) {
            ++k;
            continue;
        }
        const std::size_t s = k;
        while (k < n && over[k]) ++k;
        if (k - s >= kMoveRun)
            for (std::size_t i = s; i < k; ++i) moving[i] = true;
    }

    // 取最长的那段静止：搬运把记录切成前后两段时，长的那段才是设备就位后的静置。
    std::size_t bestBegin = 0, bestCount = 0;
    for (std::size_t k = 0; k < n;) {
        if (moving[k]) {
            ++k;
            continue;
        }
        const std::size_t s = k;
        while (k < n && !moving[k]) ++k;
        if (k - s > bestCount) {
            bestBegin = s;
            bestCount = k - s;
        }
    }

    // 段末之后连续运动的长度：够长才说明这段静置真的到头了，否则只是又被搬了一下。
    std::size_t tailBlocks = 0;
    for (std::size_t k = bestBegin + bestCount; k < n && moving[k]; ++k) ++tailBlocks;
    return {bestBegin * w, (bestBegin + bestCount) * w, tailBlocks * w};
}

Euler Aligner::solve(const ImuLog &log,
                     const std::size_t begin, std::size_t end,
                     const SiteParams &sp) {
    if (end == 0 || end > log.size()) end = log.size();
    if (end <= begin) return {};

    Eigen::Vector3d acc, gyr;
    return alignMeanDCM(log, begin, end, sp, acc, gyr).toEuler();
}

AlignQuality Aligner::diagnose(const ImuLog &log,
                               const std::size_t begin, const std::size_t end,
                               const SiteParams &sp,
                               const ImuScale &sen) {
    AlignQuality q;
    std::size_t e = end;
    if (e == 0 || e > log.size()) e = log.size();
    if (e <= begin) return q;

    Eigen::Vector3d acc, gyr;
    alignMeanDCM(log, begin, e, sp, acc, gyr);
    return diagnoseMeans(acc, gyr, sp, sen);
}

AlignQuality Aligner::diagnoseMeans(const Eigen::Vector3d &accMean, const Eigen::Vector3d &gyrMean,
                                    const SiteParams &sp,
                                    const ImuScale &sen) {
    AlignQuality q;
    const Eigen::Vector3d gB = -accMean;
    const Eigen::Vector3d& wB = gyrMean;
    const DCM C = alignDCM(gB, wB, sp);
    const ATTQuat qAtt = ATTQuat::fromDCM(C);

    q.att = C.toEuler();
    q.axisX = C.col(0);
    q.axisY = C.col(1);
    q.axisZ = C.col(2);


    q.dcmDet = C.det();
    q.dcmOrthoErr = C.orthonormalityError();


    const Eigen::Vector3d wRef = navEarthRate(sp.latDeg);
    if (wB.norm() > 0.0) q.earthRateErrDeg = Math::angleBetween(qAtt.rotate(wB), wRef) * RAD_TO_DEG;


    q.hasScale = sen.known();
    q.gravityRef = sp.g;
    q.gyroRef = OMEGA_EARTH;
    if (q.hasScale) {
        q.accelNorm = accMean.norm() * sen.accelMpsPerLsb * sen.rateHz;
        q.gyroNorm = gyrMean.norm() * sen.gyroRadPerLsb * sen.rateHz;
    }


    if (gB.norm() > 1e-9) {
        const Eigen::Quaterniond qLevel =
                Eigen::Quaterniond::FromTwoVectors(gB.normalized(), Eigen::Vector3d::UnitZ());
        const Eigen::Vector3d wLevel = qLevel * wB;
        if (const double wH = std::hypot(wLevel.x(), wLevel.y()); wH > 0.0) {
            q.latDerivedDeg = std::atan2(-wLevel.z(), wH) * RAD_TO_DEG;
            q.latDerivedValid = true;
        }
    }
    return q;
}

std::vector<AttitudeSample> Aligner::alignSeries(const ImuLog &log, const int win,
                                                 const SiteParams &sp) {
    std::vector<AttitudeSample> out;
    if (log.empty() || win <= 0) return out;
    out.reserve(log.size());

    Eigen::Vector3d acc = Eigen::Vector3d::Zero(), gyr = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < log.size(); ++i) {
        const std::size_t lo = i >= static_cast<std::size_t>(win) ? i - win + 1 : 0;
        const auto n = static_cast<double>(i - lo + 1);
        acc += (Eigen::Vector3d(log[i].ax, log[i].ay, log[i].az) - acc) / n;
        gyr += (Eigen::Vector3d(log[i].gx, log[i].gy, log[i].gz) - gyr) / n;

        const auto [roll, pitch, yaw] = alignDCM(-acc, gyr, sp).toEuler();
        out.push_back({log[i].tow, {roll, pitch, yaw}});
    }
    return out;
}

void Aligner::AlignStream::init(const int window, const SiteParams &site) {
    win = window;
    sp = site;
    acc.setZero();
    gyr.setZero();
    count = 0;
}

void Aligner::AlignStream::step(const ImuSample &s, AttitudeSample &out) {
    const std::size_t lo = count >= static_cast<std::size_t>(win) ? count - win + 1 : 0;
    const auto n = static_cast<double>(count - lo + 1);
    acc += (Eigen::Vector3d(s.ax, s.ay, s.az) - acc) / n;
    gyr += (Eigen::Vector3d(s.gx, s.gy, s.gz) - gyr) / n;
    const auto [roll, pitch, yaw] = alignDCM(-acc, gyr, sp).toEuler();
    out.tow = s.tow;
    out.att = {roll, pitch, yaw};
    ++count;
}
