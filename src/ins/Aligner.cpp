#include "Aligner.h"

#include "core/CoordConvert.h"

namespace {
    Eigen::Matrix3d alignMeanDCM(const ImuLog &log,
                                 const size_t begin, const size_t end, const SiteParams &sp,
                                 Eigen::Vector3d &accMean, Eigen::Vector3d &gyrMean) {
        accMean = Eigen::Vector3d::Zero();
        gyrMean = Eigen::Vector3d::Zero();
        if (end <= begin) return Eigen::Matrix3d::Identity();
        for (size_t i = begin; i < end; ++i) {
            accMean += Eigen::Vector3d(log[i].ax, log[i].ay, log[i].az);
            gyrMean += Eigen::Vector3d(log[i].gx, log[i].gy, log[i].gz);
        }
        const auto n = static_cast<double>(end - begin);
        accMean /= n;
        gyrMean /= n;
        return alignDCM(-accMean, gyrMean, sp);
    }
}

Attitude Aligner::solve(const ImuLog &log,
                        const size_t begin, size_t end,
                        const SiteParams &sp) {
    if (end == 0 || end > log.size()) end = log.size();
    Attitude a;
    if (end <= begin) return a;

    Eigen::Vector3d acc, gyr;
    const Eigen::Matrix3d C = alignMeanDCM(log, begin, end, sp, acc, gyr);
    DCMToEuler(C, a.roll, a.pitch, a.yaw);
    return a;
}

AttQuat Aligner::solveQuat(const ImuLog &log,
                           const size_t begin, size_t end,
                           const SiteParams &sp) {
    if (end == 0 || end > log.size()) end = log.size();
    if (end <= begin) return AttQuat::identity();

    Eigen::Vector3d acc, gyr;
    return AttQuat::fromDCM(alignMeanDCM(log, begin, end, sp, acc, gyr));
}

AlignQuality Aligner::diagnose(const ImuLog &log,
                               const size_t begin, const size_t end,
                               const SiteParams &sp,
                               const ImuScale &sen) {
    AlignQuality q;
    size_t e = end;
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
    const Eigen::Vector3d wB = gyrMean;
    const Eigen::Matrix3d C = alignDCM(gB, wB, sp);
    const AttQuat qAtt = AttQuat::fromDCM(C);

    q.axisX = C.col(0);
    q.axisY = C.col(1);
    q.axisZ = C.col(2);


    q.dcmDet = C.determinant();
    q.dcmOrthoErr = (C.transpose() * C - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();



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
        const double wH = std::hypot(wLevel.x(), wLevel.y());
        if (wH > 0.0) {
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
    for (size_t i = 0; i < log.size(); ++i) {
        const size_t lo = i >= static_cast<size_t>(win) ? i - win + 1 : 0;
        const auto n = static_cast<double>(i - lo + 1);
        acc += (Eigen::Vector3d(log[i].ax, log[i].ay, log[i].az) - acc) / n;
        gyr += (Eigen::Vector3d(log[i].gx, log[i].gy, log[i].gz) - gyr) / n;

        const Eigen::Matrix3d C = alignDCM(-acc, gyr, sp);
        AttitudeSample s;
        s.tow = log[i].tow;
        DCMToEuler(C, s.roll, s.pitch, s.yaw);
        out.push_back(s);
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
    const size_t lo = count >= static_cast<size_t>(win) ? count - win + 1 : 0;
    const auto n = static_cast<double>(count - lo + 1);
    acc += (Eigen::Vector3d(s.ax, s.ay, s.az) - acc) / n;
    gyr += (Eigen::Vector3d(s.gx, s.gy, s.gz) - gyr) / n;
    const Eigen::Matrix3d C = alignDCM(-acc, gyr, sp);
    DCMToEuler(C, out.roll, out.pitch, out.yaw);
    out.tow = s.tow;
    ++count;
}
