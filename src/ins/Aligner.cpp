#include "Aligner.h"

#include "core/CoordConvert.h"

namespace {
    Eigen::Matrix3d alignMeanDCM(const ImuLog &log, const Eigen::Vector3d &gyroBiasCounts,
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
        return alignDCM(-accMean, gyrMean - gyroBiasCounts, sp);
    }
}

Attitude Aligner::solve(const ImuLog &log,
                        const Eigen::Vector3d &gyroBiasCounts,
                        const size_t begin, size_t end,
                        const SiteParams &sp) {
    if (end == 0 || end > log.size()) end = log.size();
    Attitude a;
    if (end <= begin) return a;

    Eigen::Vector3d acc, gyr;
    const Eigen::Matrix3d C = alignMeanDCM(log, gyroBiasCounts, begin, end, sp, acc, gyr);
    DCMToEuler(C, a.roll, a.pitch, a.yaw);
    return a;
}

AttQuat Aligner::solveQuat(const ImuLog &log,
                           const Eigen::Vector3d &gyroBiasCounts,
                           const size_t begin, size_t end,
                           const SiteParams &sp) {
    if (end == 0 || end > log.size()) end = log.size();
    if (end <= begin) return AttQuat::identity();

    Eigen::Vector3d acc, gyr;
    return AttQuat::fromDCM(alignMeanDCM(log, gyroBiasCounts, begin, end, sp, acc, gyr));
}

AlignQuality Aligner::diagnose(const ImuLog &log,
                               const Eigen::Vector3d &gyroBiasCounts,
                               const size_t begin, size_t end,
                               const SiteParams &sp) {
    AlignQuality q;
    if (end == 0 || end > log.size()) end = log.size();
    if (end <= begin) return q;

    Eigen::Vector3d acc, gyr;
    alignMeanDCM(log, gyroBiasCounts, begin, end, sp, acc, gyr);
    return diagnoseMeans(acc, gyr, gyroBiasCounts, sp);
}

AlignQuality Aligner::diagnoseMeans(const Eigen::Vector3d &accMean, const Eigen::Vector3d &gyrMean,
                                    const Eigen::Vector3d &gyroBiasCounts,
                                    const SiteParams &sp,
                                    const double gyroScaleCountsPerRadS) {
    AlignQuality q;
    const Eigen::Vector3d wB = gyrMean - gyroBiasCounts; // counts（罗盘只用方向，与单位无关）
    const Eigen::Matrix3d C = alignDCM(-accMean, wB, sp);
    const AttQuat qAtt = AttQuat::fromDCM(C);

    // C_b^n 的列即机体各轴在当地 NED 的方向（v^n = C_b^n·v^b）
    q.axisX = C.col(0);
    q.axisY = C.col(1);
    q.axisZ = C.col(2);

    // 重力方向残差：旋进 n 系后应与当地天向 [0,0,g] 同向
    const Eigen::Vector3d gB = -accMean;
    const Eigen::Vector3d gRef = navGravity(sp.g);
    if (gB.norm() > 0.0 && gRef.norm() > 0.0) {
        q.gravityErrDeg = Math::angleBetween(qAtt.rotate(gB), gRef) * RAD_TO_DEG;
    }

    // 地球自转方向残差
    const Eigen::Vector3d wRef = navEarthRate(sp.latDeg, sp.earthRate);
    if (wB.norm() > 0.0 && wRef.norm() > 0.0) {
        q.earthRateErrDeg = Math::angleBetween(qAtt.rotate(wB), wRef) * RAD_TO_DEG;
    }

    // 大小残差：换算到 rad/s 后才可与 ω_ie^n 比模长（未给标度因数则保持 0）
    if (gyroScaleCountsPerRadS > 0.0) {
        const Eigen::Vector3d wRad = qAtt.rotate(wB) / gyroScaleCountsPerRadS;
        q.earthRateErrRad = (wRad - wRef).norm();
    }
    return q;
}

std::vector<AttitudeSample> Aligner::alignSeries(const ImuLog &log, const int win,
                                                 const Eigen::Vector3d &gyroBiasCounts,
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

        const Eigen::Matrix3d C = alignDCM(-acc, gyr - gyroBiasCounts, sp);
        AttitudeSample s;
        s.tow = log[i].tow;
        DCMToEuler(C, s.roll, s.pitch, s.yaw);
        out.push_back(s);
    }
    return out;
}

void Aligner::AlignStream::init(const int window, const SiteParams &site, const Eigen::Vector3d &gyroBiasCounts) {
    win = window;
    sp = site;
    gyroBias = gyroBiasCounts;
    acc.setZero();
    gyr.setZero();
    count = 0;
}

// 喂入第 count 个样本（0 基），写出该历元姿态。与 alignSeries 第 i=count 步结果逐位一致。
void Aligner::AlignStream::step(const ImuSample &s, AttitudeSample &out) {
    const size_t lo = count >= static_cast<size_t>(win) ? count - win + 1 : 0;
    const auto n = static_cast<double>(count - lo + 1);
    acc += (Eigen::Vector3d(s.ax, s.ay, s.az) - acc) / n;
    gyr += (Eigen::Vector3d(s.gx, s.gy, s.gz) - gyr) / n;
    const Eigen::Matrix3d C = alignDCM(-acc, gyr - gyroBias, sp);
    DCMToEuler(C, out.roll, out.pitch, out.yaw);
    out.tow = s.tow;
    ++count;
}
