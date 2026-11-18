#include "ImuScaleProbe.h"

#include <algorithm>
#include <cmath>
#include <exception>

#include "core/Const.h"
#include "Observer.h"
#include "i18n/I18n.h"

namespace Solvers::INS {
    namespace {
        /// 内置 IMU 型号表：NovAtel 日志里的 Type 号 → 型号名与三个解码量。
        /// 型号名只用于界面说明，标称当量是反推与自描述的落点。
        struct ImuModel {
            int type{};
            const char *name{};
            ImuScale scale;
        };

        constexpr double kFtToMps = 0.3048;
        constexpr double kAccelHG1700 = 0x1p-27 * kFtToMps;

        const ImuModel kImuModels[] = {
            {8, "IMU-LN200", {0x1p-14, 0x1p-19, 200.0}},
            {11, "HG1700-AG58", {kAccelHG1700, 0x1p-33, 100.0}},
            {12, "HG1700-AG62", {0x1p-26 * kFtToMps, 0x1p-33, 100.0}},
            {16, "IMU-CPT (KVH)", {0.05 / 32768.0, 0.1 / 3600.0 / 256.0 * DEG_TO_RAD, 200.0}},
            {20, "HG1930-AA99", {kAccelHG1700, 0x1p-33, 100.0}},
            {26, "IMU-ISA-100C", {2.0e-8, 1.0e-9, 200.0}},
            {27, "HG1900-CA50", {kAccelHG1700, 0x1p-33, 100.0}},
            {28, "HG1930-CA50", {kAccelHG1700, 0x1p-33, 100.0}},
            {32, "OEM-IMU-STIM300", {0x1p-22, 0x1p-21 * DEG_TO_RAD, 125.0}},
            {33, "IMU-P1750", {0.05 / 32768.0, 0.1 / 3600.0 / 256.0 * DEG_TO_RAD, 200.0}},
            {
                41, "OEM-IMU-EG320N", {
                    0.200 / 65536.0 * (9.80665 / 1000.0) / 125.0,
                    0.008 / 65536.0 / 125.0 * DEG_TO_RAD, 125.0
                }
            },
            {52, "IMU-µIMU-IC", {2.0e-8, 1.0e-9, 200.0}},
            {56, "OEM-IMU-STIM300D", {0x1p-22, 0x1p-21 * DEG_TO_RAD, 125.0}},
            {58, "OEM-IMU-HG4930 AN01", {0x1p-29, 0x1p-33, 100.0}},
            {
                61, "OEM-IMU-EG370N", {
                    0.400 / 65536.0 * (9.80665 / 1000.0) / 200.0,
                    0.0151515 / 65536.0 / 200.0 * DEG_TO_RAD, 200.0
                }
            },
            {
                62, "OEM-IMU-EG320N (200Hz)", {
                    0.200 / 65536.0 * (9.80665 / 1000.0) / 200.0,
                    0.008 / 65536.0 / 200.0 * DEG_TO_RAD, 200.0
                }
            },
            {68, "OEM-IMU-HG4930 AN04", {0x1p-29, 0x1p-33, 100.0}},
            {69, "OEM-IMU-HG4930 AN04 (400Hz)", {0x1p-29, 0x1p-33, 400.0}},
            {71, "OEM-IMU-µIMU-IC-UART (400Hz)", {2.0e-8, 1.0e-9, 400.0}},
        };

        /// Type 号查表；日志没声明 Type（-1）或表里没有就返回空。
        const ImuModel *findImuModel(const int type) {
            for (const auto &m: kImuModels)
                if (m.type == type) return &m;
            return nullptr;
        }

        /// 反推窗口：一段 4000 样本（约 20 s），最多试开头三段，避开上电后的沉降。
        constexpr std::size_t kProbeSamples = 4000;
        constexpr std::size_t kMinProbeSamples = 200;
        constexpr int kProbeSegments = 3;
        /// 静置判据：|f| 的离散度与型号匹配容差（采样率按时间戳，容差放宽）。
        constexpr double kStaticSpread = 0.02;
        constexpr double kAccelTol = 0.05;
        constexpr double kRateTol = 0.10;

        /// 相邻样本间隔的中位数：反推型号时的采样率就是它的倒数。
        double medianGap(const ImuLog &seg) {
            if (seg.size() < 3) return 0.0;
            std::vector<double> d;
            d.reserve(seg.size() - 1);
            for (std::size_t i = 1; i < seg.size(); ++i) d.push_back(seg[i].tow - seg[i - 1].tow);
            std::sort(d.begin(), d.end());
            return d[d.size() / 2];
        }

        /// 一段静置样本的 |f| 均值；离散度超限说明不是静置段。
        bool staticMagnitude(const ImuLog &seg, double &mean) {
            double sum = 0.0, sum2 = 0.0;
            for (const auto &s: seg) {
                const double m = std::sqrt(static_cast<double>(s.ax) * s.ax +
                                           static_cast<double>(s.ay) * s.ay +
                                           static_cast<double>(s.az) * s.az);
                sum += m;
                sum2 += m * m;
            }
            const auto n = static_cast<double>(seg.size());
            mean = sum / n;
            if (!(mean > 0.0)) return false;
            const double var = sum2 / n - mean * mean;
            return std::sqrt(std::max(0.0, var)) / mean <= kStaticSpread;
        }

        /// 由静置段 |f| 反推加速度当量，再按当量与采样率在型号表里找唯一一组参数。
        const ImuModel *matchStaticSegment(const ImuLog &seg, const double gravity) {
            if (seg.size() < kMinProbeSamples) return nullptr;
            double mean = 0.0;
            if (!staticMagnitude(seg, mean)) return nullptr;
            const double dt = medianGap(seg);
            if (!(dt > 1e-4) || dt > 0.1) return nullptr;
            const double accel = gravity * dt / mean;
            const double rate = 1.0 / dt;
            const ImuModel *hit = nullptr;
            for (const auto &m: kImuModels) {
                if (std::fabs(accel / m.scale.accelMpsPerLsb - 1.0) > kAccelTol) continue;
                if (std::fabs(rate / m.scale.rateHz - 1.0) > kRateTol) continue;
                if (!hit) {
                    hit = &m;
                } else if (m.scale.accelMpsPerLsb != hit->scale.accelMpsPerLsb ||
                           m.scale.gyroRadPerLsb != hit->scale.gyroRadPerLsb ||
                           m.scale.rateHz != hit->scale.rateHz) {
                    return nullptr; // 多个不同型号都落在容差内，无法唯一确定
                }
            }
            return hit;
        }
    }

    ScaleProbe probeScale(const std::string &path, const double gravity) {
        ScaleProbe r;
        try {
            const auto reader = Observer::makeNovAtelReader(path);
            if (!reader || !reader->open(path)) {
                r.note = I18n::tr("ins.err.open_imu", path);
                return r;
            }
            if (const ImuModel *detected = findImuModel(reader->detectedImuType())) {
                r.level = ScaleProbe::Level::Pass;
                r.scale = detected->scale;
                r.note = I18n::tr("ins.scale.probe_record", detected->name, detected->type,
                                  detected->scale.rateHz);
                return r;
            }

            ImuLog seg;
            seg.reserve(kProbeSamples);
            ImuSample sample{};
            for (int k = 0; k < kProbeSegments && gravity > 0.0; ++k) {
                seg.clear();
                while (seg.size() < kProbeSamples && reader->getNextImu(sample)) seg.push_back(sample);
                if (const ImuModel *hit = matchStaticSegment(seg, gravity)) {
                    r.level = ScaleProbe::Level::Warn;
                    r.scale = hit->scale;
                    r.note = I18n::tr("ins.scale.probe_reverse", hit->name, hit->type,
                                      hit->scale.rateHz);
                    return r;
                }
                if (seg.size() < kProbeSamples) break;
            }
            r.note = I18n::tr("ins.scale.probe_failed");
        } catch (const std::exception &ex) {
            r.level = ScaleProbe::Level::Fail;
            r.note = I18n::tr("ins.err.decode", ex.what());
        }
        return r;
    }

    ScaleDetect::~ScaleDetect() {
        if (worker.joinable()) worker.join();
    }

    void ScaleDetect::start(const std::string &path, const double gravity) {
        if (worker.joinable()) worker.join();
        busy = true;
        {
            std::lock_guard lk(mtx);
            done = ScaleProbe();
        }
        worker = std::thread([this, path, gravity] {
            const ScaleProbe r = probeScale(path, gravity);
            std::lock_guard lk(mtx);
            done = r;
            busy = false; // 最后一步：界面此后才认为本轮结束
        });
    }

    void ScaleDetect::poll(ImuScale &scale) {
        if (busy || !worker.joinable()) return;
        worker.join();
        ScaleProbe r;
        {
            std::lock_guard lk(mtx);
            r = done;
        }
        text = r.note;
        switch (r.level) {
            case ScaleProbe::Level::Pass: level = Level::Pass; break;
            case ScaleProbe::Level::Warn: level = Level::Warn; break;
            case ScaleProbe::Level::Fail: level = Level::Fail; break;
        }
        if (r.ok()) scale = r.scale;
    }
}
