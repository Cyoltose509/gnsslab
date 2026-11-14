#include "Aligner.h"

#include <algorithm>
#include <cmath>

#include "MathUtils.h"

namespace {
    DCM alignMeanDCM(const ImuLog &log,
                                 const std::size_t begin, const std::size_t end, const SiteParams &sp,
                                 Eigen::Vector3d &accMean, Eigen::Vector3d &gyrMean) {
        accMean = Eigen::Vector3d::Zero();
        gyrMean = Eigen::Vector3d::Zero();
        if (end <= begin) return DCM();
        for (std::size_t i = begin; i < end; ++i) {
            accMean += Eigen::Vector3d(log[i].ax, log[i].ay, log[i].az);
            gyrMean += Eigen::Vector3d(log[i].gx, log[i].gy, log[i].gz);
        }
        const auto n = static_cast<double>(end - begin);
        accMean /= n;
        gyrMean /= n;
        return alignDCM(-accMean, gyrMean, sp);
    }
}

std::size_t Aligner::staticHead(const ImuLog &log, const ImuScale &scale, const SiteParams &sp) {
    const double rate = scale.rateHz > 0.0 ? scale.rateHz : 200.0;
    const std::size_t w = std::max<std::size_t>(200, static_cast<std::size_t>(2.0 * rate));
    if (log.size() < 2 * w) return 0;

    const Euler ref = solve(log, 0, w, sp);
    Eigen::Vector3d accRef = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < w; ++i)
        accRef += Eigen::Vector3d(log[i].ax, log[i].ay, log[i].az);

    constexpr double kAttTolRad = 0.5 * DEG_TO_RAD;
    constexpr double kGyroTolRad = 1.0e-3;
    constexpr double kAccelTol = 0.02;
    constexpr int kRun = 3;
    std::size_t first = 0;
    int run = 0;
    for (std::size_t b = w; b + w <= log.size(); b += w) {
        const Euler att = solve(log, b, b + w, sp);
        Eigen::Vector3d acc = Eigen::Vector3d::Zero(), gyr = Eigen::Vector3d::Zero();
        for (std::size_t i = b; i < b + w; ++i) {
            acc += Eigen::Vector3d(log[i].ax, log[i].ay, log[i].az);
            gyr += Eigen::Vector3d(log[i].gx, log[i].gy, log[i].gz);
        }
        const double omega = gyr.norm() / static_cast<double>(w) * scale.gyroRadPerLsb * rate;
        const bool changed = std::fabs(att.roll - ref.roll) > kAttTolRad ||
                             std::fabs(att.pitch - ref.pitch) > kAttTolRad ||
                             omega > kGyroTolRad ||
                             (accRef.norm() > 0.0 &&
                              std::fabs(acc.norm() / accRef.norm() - 1.0) > kAccelTol);
        if (changed) {
            if (run == 0) first = b;
            if (++run >= kRun) return first;
        } else {
            run = 0;
        }
    }
    return log.size();
}

Euler Aligner::solve(const ImuLog &log,
                     const std::size_t begin, std::size_t end,
                     const SiteParams &sp) {
    if (end == 0 || end > log.size()) end = log.size();
    if (end <= begin) return {};

    Eigen::Vector3d acc, gyr;
    return alignMeanDCM(log, begin, end, sp, acc, gyr).toEuler();
}

ATTQuat Aligner::solveQuat(const ImuLog &log,
                           const std::size_t begin, std::size_t end,
                           const SiteParams &sp) {
    if (end == 0 || end > log.size()) end = log.size();
    if (end <= begin) return ATTQuat::identity();

    Eigen::Vector3d acc, gyr;
    return ATTQuat::fromDCM(alignMeanDCM(log, begin, end, sp, acc, gyr));
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

        const Euler e = alignDCM(-acc, gyr, sp).toEuler();
        out.push_back({log[i].tow, e.roll, e.pitch, e.yaw});
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
    const Euler e = alignDCM(-acc, gyr, sp).toEuler();
    out.roll = e.roll;
    out.pitch = e.pitch;
    out.yaw = e.yaw;
    out.tow = s.tow;
    ++count;
}
