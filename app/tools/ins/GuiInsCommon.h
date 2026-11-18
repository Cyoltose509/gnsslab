#pragma once


#include "Log.h"
#include "GuiInsPanel.h"
#include "solvers/view/GuiHelpers.h"
#include "i18n/I18n.h"
#include "ui/FileDialog.h"
#include "ui/GuiColors.h"

#include <Eigen/Eigen>

#include "core/Const.h"
#include "CSV.h"
#include "Observer.h"
#include "io/novatel/LogReader.h"
#include "FileUtils.h"

#include "implot.h"
#include <atomic>
#include <functional>
#include <filesystem>
#include <string>
#include <vector>

namespace Tools::Ins {
    /// IMU 泵：轮询读取器直到读完，逐样本回调 onSample，每 2048 样本用总数回调一次 onBatch。
    inline void pumpImu(NovAtel::LogReader &r, const std::atomic<bool> &stop,
                        const std::function<void(const ImuSample &)> &onSample,
                        const std::function<void(long long)> &onBatch) {
        std::size_t last = 0, batchMark = 0;
        while (!stop) {
            ImuSample t;
            const bool more = r.getNextImu(t);
            const ImuLog &L = r.getImuLog();
            for (std::size_t i = last; i < L.size(); ++i) onSample(L[i]);
            last = L.size();
            if (!more) break;
            if (L.size() - batchMark >= 2048) {
                onBatch(static_cast<long long>(L.size()));
                batchMark = L.size();
            }
        }
        if (!stop) onBatch(static_cast<long long>(r.getImuLog().size()));
    }

    /// NovAtel 日志按内容定读取器（二进制/ASC 文本，后缀不作数），open 成功后进入 IMU 泵。
    inline bool pumpImuFile(const std::string &path, const std::atomic<bool> &stop,
                            const std::function<void(const ImuSample &)> &onSample,
                            const std::function<void(long long)> &onBatch) {
        const auto reader = Observer::makeNovAtelReader(path);
        if (!reader->open(path)) return false;
        pumpImu(*reader, stop, onSample, onBatch);
        return true;
    }


    using UI::kColPass;
    using UI::kColFail;
    using UI::kColWarn;
    using UI::kColInfo;

    void drawAlert(AlertLevel lv, const char *text);

    void renderInsParams(CalibParams &par);

    void renderInsParams(AlignParams &par);

    void renderInsParams(VerifyParams &par);

    void saveInsParams(const CalibParams &par);

    void saveInsParams(const AlignParams &par);

    void saveInsParams(const VerifyParams &par);

    void loadInsParams(CalibParams &par);

    void loadInsParams(AlignParams &par);

    void loadInsParams(VerifyParams &par);

    constexpr float kChartHeight = 360.0f;
    constexpr float kFileChartH = 220.0f;

    std::size_t decimateStride(std::size_t n, std::size_t maxPts = 4000);

    void decimateImu(const ImuLog &src, ImuLog &out);

    using UI::kColX;
    using UI::kColY;
    using UI::kColZ;

    ImPlotSpec lineSpec(const ImVec4 &c);

    void toElapsed(const std::vector<double> &tow, std::vector<double> &out);

    void fitXToLatest(bool newData, double xmax);

    template<class Task>
    void exportCSVDialog(Task &task, const std::string &defaultName, const std::string &displayName,
                         const std::function<void(CSV::Writer &)> &build) {
        std::wstring out;
        if (!FileDialog::saveCSV(defaultName, out)) return;
        CSV::Writer w;
        build(w);
        std::string error;
        if (!File::safeWrite(std::filesystem::path(out), [&](std::ostream &f) {
                CSV::Writer::writeBom(f);
                f << w.str();
            }, error)) {
            task.setError(error);
            return;
        }
        task.clearError();
        LOG_INFO << "[" << task.fileName << "] " << I18n::tr("ins.tool.log.export", displayName,
                                                             std::filesystem::path(out).string());
    }

    struct CalibUnit {
        const char *ui;
        const char *csv;
        double factor;
        bool perG;
        bool usePhys;
        const char *uiFmt;
    };

    struct CalibBasis {
        Eigen::Vector3d CalibReport::*vec = nullptr;
        Eigen::Vector3d ImuCalibration::*cal = nullptr;
        double CalibReport::*sca = nullptr;
        bool timesG = false;
    };

    constexpr CalibBasis topVec(Eigen::Vector3d CalibReport::*m, const bool timesG = false) {
        return {m, nullptr, nullptr, timesG};
    }

    constexpr CalibBasis calVec(Eigen::Vector3d ImuCalibration::*m) {
        return {nullptr, m, nullptr, false};
    }

    constexpr CalibBasis topScalar(double CalibReport::*m, const bool timesG = false) {
        return {nullptr, nullptr, m, timesG};
    }

    Eigen::Vector3d basisValue(const CalibBasis &b, const CalibReport &r, double g);

    struct CalibQty {
        const char *uiKey{};
        const char *csv{};
        CalibBasis counts;
        CalibBasis phys;
        const CalibUnit *units{};
        int nUnits{};
        int csvDigits{};
        bool csvSci{};
        bool scalar{};
        int sigmaQty{};
    };

    enum CalibQtyId {
        Q_ACC_SCALE = 0, Q_ACC_SCALE_SIGMA, Q_ACC_BIAS, Q_ACC_KNOM, Q_ACC_DS,
        Q_GYRO_KEFF, Q_GYRO_KEFF_NOM, Q_GYRO_SCALE, Q_GYRO_BIAS, Q_GYRO_DS, Q_COUNT
    };

    inline constexpr CalibUnit kAccScaleUnits[] = {
        {"LSB/g", "LSB/g", 1.0, false, false, ""},
        {"LSB/(m/s²)", "LSB/(m/s^2)", 1.0, true, false, ""},
        {"LSB/Gal", "LSB/Gal", 0.01, true, false, ""},
    };
    inline constexpr CalibUnit kAccBiasUnits[] = {
        {"LSB", "LSB", 1.0, false, false, "%.2f"},
        {"m/s²", "m/s^2", 1.0, false, true, "%.6f"},
        {"mg", "mg", 1e3, true, true, "%.4f"},
        {"mGal", "mGal", 1e5, false, true, "%.2f"},
        {"Gal", "Gal", 1e2, false, true, "%.4f"},
    };
    inline constexpr CalibUnit kDsUnits[] = {
        {"ins.unit.unitless", "ins.unit.unitless", 1.0, false, false, "%.6f"},
        {"ppm", "ppm", 1e6, false, false, "%.2f"},
        {"%", "%", 1e2, false, false, "%.4f"},
    };
    inline constexpr CalibUnit kGyroKeffUnits[] = {
        {"LSB/rad", "LSB/rad", 1.0, false, false, "%.1f"},
        {"LSB/°", "LSB/deg", DEG_TO_RAD, false, false, "%.1f"},
    };
    inline constexpr CalibUnit kGyroScaleUnits[] = {
        {"LSB/(rad/s)", "LSB/(rad/s)", 1.0, false, false, "%.1f"},
        {"LSB/(°/h)", "LSB/(deg/h)", DEG_TO_RAD / 3600.0, false, false, "%.4f"},
    };
    inline constexpr CalibUnit kGyroBiasUnits[] = {
        {"ins.unit.lsb_epoch", "ins.unit.lsb_epoch", 1.0, false, false, "%.3f"},
        {"rad/s", "rad/s", 1.0, false, true, "%.9f"},
        {"°/h", "deg/h", RAD_TO_DEG * 3600.0, false, true, "%.3f"},
    };

    inline constexpr CalibQty kCalibQty[Q_COUNT] = {
        {
            "ins.calib.q.acc_scale", "ins.calib.q.acc_scale",
            topVec(&CalibReport::accelScaleG), topVec(&CalibReport::accelScaleG),
            kAccScaleUnits, 3, 4, false, false, Q_ACC_SCALE_SIGMA
        },
        {
            "ins.calib.q.acc_scale_sigma", "ins.calib.q.acc_scale_sigma",
            topVec(&CalibReport::accelScaleSigma, true), topVec(&CalibReport::accelScaleSigma, true),
            kAccScaleUnits, 3, 4, false, false, -1
        },
        {
            "ins.calib.q.acc_bias", "ins.calib.q.acc_bias",
            calVec(&ImuCalibration::accelB), topVec(&CalibReport::accelBiasPhys),
            kAccBiasUnits, 5, 4, false, false, -1
        },
        {
            "ins.calib.q.acc_knom", "ins.calib.q.acc_knom",
            topScalar(&CalibReport::nominalAccelK, true), topScalar(&CalibReport::nominalAccelK, true),
            kAccScaleUnits, 3, 6, false, true, -1
        },
        {
            "ins.calib.q.acc_ds", "ins.calib.q.acc_ds",
            topVec(&CalibReport::accelDeltaS), topVec(&CalibReport::accelDeltaS),
            kDsUnits, 3, 9, true, false, -1
        },
        {
            "ins.calib.q.gyro_keff", "ins.calib.q.gyro_keff",
            topVec(&CalibReport::gyroKeff), topVec(&CalibReport::gyroKeff),
            kGyroKeffUnits, 2, 6, true, false, -1
        },
        {
            "ins.calib.q.gyro_keff_nom", "ins.calib.q.gyro_keff_nom",
            topScalar(&CalibReport::nominalKeff), topScalar(&CalibReport::nominalKeff),
            kGyroKeffUnits, 2, 6, true, true, -1
        },
        {
            "ins.calib.q.gyro_scale", "ins.calib.q.gyro_scale",
            calVec(&ImuCalibration::gyroScale), calVec(&ImuCalibration::gyroScale),
            kGyroScaleUnits, 2, 6, true, false, -1
        },
        {
            "ins.calib.q.gyro_bias", "ins.calib.q.gyro_bias",
            calVec(&ImuCalibration::gyroBias), topVec(&CalibReport::gyroBiasPhys),
            kGyroBiasUnits, 3, 4, false, false, -1
        },
        {
            "ins.calib.q.gyro_ds", "ins.calib.q.gyro_ds",
            topVec(&CalibReport::gyroDeltaS), topVec(&CalibReport::gyroDeltaS),
            kDsUnits, 3, 6, true, false, -1
        },
    };

    /// 单位显示名：给的是文案 key 就按当前语言取，纯单位符号原样返回。
    inline const char *unitText(const char *ui) {
        return I18n::exists(ui) ? I18n::trc(ui) : ui;
    }

    inline constexpr int kCalibUiRows[] = {
        Q_ACC_SCALE, Q_ACC_BIAS, Q_ACC_DS, Q_GYRO_KEFF, Q_GYRO_SCALE, Q_GYRO_BIAS, Q_GYRO_DS
    };

    Eigen::Vector3d calibQtyValue(const CalibQty &q, const CalibUnit &u, const CalibReport &r, double g);

    int pmDecimals(double s);

    void drawCompChart(const char *title, const std::vector<double> &t,
                       const std::vector<double> &a, const std::vector<double> &b,
                       const std::vector<double> &c, bool chartNew, float h);

    void drawScaleBars(const char *title, bool ready, const double v[3], const ImVec4 &fill,
                       float h, const char *fmt = "%.1f");

    void vec3Row(const char *label, double x, double y, double z, const char *fmt = "%.3f");

    void steadyStats(const std::vector<AttitudeSample> &s,
                     double &ym, double &ysd, double &pm, double &psd, double &rm, double &rsd);
}
