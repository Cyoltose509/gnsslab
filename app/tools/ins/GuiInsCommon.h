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
#include <thread>
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
        onBatch(static_cast<long long>(r.getImuLog().size()));
    }

    /// NovAtel 日志按内容定读取器（二进制/ASC 文本，后缀不作数）；open 成功且 onReader
    /// 通过后进入 IMU 泵，任一步失败返回 false。
    inline bool pumpImuFile(const std::string &path, const std::atomic<bool> &stop,
                            const std::function<bool(NovAtel::LogReader &)> &onReader,
                            const std::function<void(const ImuSample &)> &onSample,
                            const std::function<void(long long)> &onBatch) {
        const auto reader = Observer::makeNovAtelReader(path);
        if (!reader->open(path) || !onReader(*reader)) return false;
        pumpImu(*reader, stop, onSample, onBatch);
        return true;
    }


    template<class Task>
    void beginRun(Task &task) {
        if (task.worker.joinable()) task.worker.join();
        task.clearError();
        task.state = Task::State::Running;
        task.loading = true;
        task.stop = false;
        task.hasResult.store(false);
        task.runFailed.store(false);
        task.sampleCount.store(0);
        task.chartDirty.store(false);
        LOG_INFO << "[" << task.fileName << "] " << I18n::tr("ins.tool.log.start");
    }

    template<class Task>
    void endRun(Task &task) {
        task.loading = false;
        task.state = Task::State::Done;
    }

    template<class Task>
    void abortRun(Task &task, const std::string &msg) {
        LOG_ERROR << "[" << task.fileName << "] " << msg;
        task.runFailed.store(true);
        task.setError(msg);
        endRun(task);
    }

    template<class Task>
    void finishRun(Task &task) {
        task.chartDirty.store(true);
        task.hasResult.store(true);
        endRun(task);
    }

    template<class Task>
    bool stoppedAndEnd(Task &task) {
        if (!task.stop.load()) return false;
        endRun(task);
        return true;
    }

    template<class Task, class Fn>
    void launchWorker(Task &task, Fn fn) {
        task.worker = std::thread([fn] { fn(); });
    }

    using UI::kColPass;
    using UI::kColFail;
    using UI::kColWarn;
    using UI::kColInfo;

    void drawAlert(AlertLevel lv, const char *text);

    template<class Task>
    bool drawResultError(const Task &task) {
        if (!task.errorFlag()) return false;
        drawAlert(AlertLevel::Error, task.errorMessage().c_str());
        return true;
    }

    template<class Task>
    void drawRunError(const Task &task) {
        if (task.errorFlag() && !task.hasResult.load())
            drawAlert(AlertLevel::Error, task.errorMessage().c_str());
    }

    template<class Task, class FileTabFn, class ParamTabFn, class StartFn>
    void renderInsConfig(const char *title, Task &task, const char *startLabel,
                         FileTabFn fileTab, ParamTabFn paramTab, StartFn onStart) {
        const std::string childId = std::string(title) + std::to_string(task.cfgIndex);
        ImGui::BeginChild(childId.c_str(), ImVec2(0.0f, 0.0f), ImGuiChildFlags_None);
        if (ImGui::BeginTabBar("##cfg_tabs")) {
            if (ImGui::BeginTabItem(I18n::trc("common.file"))) {
                ImGui::Spacing();
                fileTab();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem(I18n::trc("common.params"))) {
                ImGui::PushItemWidth(260);
                paramTab();
                ImGui::PopItemWidth();
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::Spacing();
        if (ImGui::Button(I18n::trc(startLabel), ImVec2(200, 40))) onStart();
        ImGui::SameLine();
        task.renderCancelButton();
        drawRunError(task);
        ImGui::EndChild();
    }

    void renderInsParams(CalibParams &par);

    void renderInsParams(AlignParams &par);

    void renderInsParams(VerifyParams &par);

    void saveInsParams(const CalibParams &par);

    void saveInsParams(const AlignParams &par);

    void saveInsParams(const VerifyParams &par);

    void loadInsParams(CalibParams &par);

    void loadInsParams(AlignParams &par);

    void loadInsParams(VerifyParams &par);

    enum class ScaleSource { None, Record, Manual, Guess };

    enum class RecordScale { Model, UnknownType, NoType };

    struct RecordScaleInfo {
        RecordScale kind = RecordScale::NoType;
        const ImuModel *model = nullptr;
        int type = -1;
    };

    inline RecordScaleInfo readRecordScale(const NovAtel::LogReader &r) {
        if (const ImuModel *m = r.detectedImuModel()) return {RecordScale::Model, m, m->type};
        if (const int t = r.detectedImuType(); t >= 0) return {RecordScale::UnknownType, nullptr, t};
        return {RecordScale::NoType, nullptr, -1};
    }

    struct ScaleResolution {
        const ImuModel *recordModel = nullptr;
        int unknownType = -1;
        int noType = 0;
        std::string conflict;

        ImuScale scale{};
        ScaleSource src = ScaleSource::None;
        std::string modelName;
        int modelType = -1;
        std::string note;
        double guessAccel = 0.0;
        double guessRate = 0.0;

        [[nodiscard]] bool known() const { return scale.known(); }
        [[nodiscard]] bool hasRecord() const { return recordModel != nullptr; }

        [[nodiscard]] ImuScale currentScale() const { return recordModel ? recordModel->scale : ImuScale{}; }

        bool observe(const RecordScaleInfo &info);

        bool resolve(ScaleFallback &sf, const std::vector<ImuLog> &staticCandidates, double g);

        [[nodiscard]] std::string failMessage() const;

        [[nodiscard]] std::string describe() const;

        [[nodiscard]] std::string recordLabel() const;

        [[nodiscard]] ScaleNotice notice(bool settled) const;

    private:
        void adoptManual(const ImuScale &s);

        bool tryGuess(const ImuLog &staticSeg, double g);
    };

    constexpr std::size_t kScaleProbeSamples = 4000;

    struct ScaleIdentifier {
        explicit ScaleIdentifier(ScaleFallback &fallback) : sf(fallback) {
        }

        ScaleFallback &sf;
        ScaleResolution res;
        std::vector<ImuLog> candidates;
        ImuLog probe;

        bool observe(const NovAtel::LogReader &r,
                     const std::function<void(const std::string &, const ScaleNotice &)> &sink) {
            if (!res.observe(readRecordScale(r))) return false;
            if (const std::string label = res.recordLabel(); !label.empty())
                sink(label, res.notice(false));
            return true;
        }

        void probeSample(const ImuSample &s);

        void offerProbe();

        void offerStatic(const ImuLog &seg);

        bool resolve(double g);

        [[nodiscard]] std::string failText() const;

        [[nodiscard]] bool mixedModel() const;

        [[nodiscard]] std::string failureMessage(const std::string &path) const;
    };

    void keepManualScaleFor(ScaleFallback &sf, const std::string &sig);

    bool drawScaleFallback(ScaleFallback &sf, bool allowSkip);

    template<class Task>
    bool renderResultPrologue(Task &task, const bool allowSkipScale, const std::function<void()> &restart) {
        ScaleNotice notice;
        {
            std::lock_guard lk(task.resultMutex);
            notice = task.scaleNotice;
        }
        const bool errored = drawResultError(task);
        if (notice.show && !errored && !task.scaleSf.asked.load(std::memory_order_relaxed))
            drawAlert(notice.level, notice.text.c_str());
        if (!task.loading.load() && drawScaleFallback(task.scaleSf, allowSkipScale)) {
            restart();
            return true;
        }
        return errored && task.runFailed.load();
    }

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

    /// 单位表两列：ui 是界面显示名（中文的给文案 key，纯 ASCII 单位符号直接写原文），
    /// csv 是导出表头，属于数据契约，不随语言变。
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
                       const std::vector<double> &c, bool chartNew);

    void drawScaleBars(const char *title, bool ready, const double v[3], const ImVec4 &fill,
                       const char *fmt = "%.1f");

    template<class F1, class F2>
    void twoCol(const F1 &left, const F2 &right) {
        ImGui::Columns(2, nullptr, false);
        left();
        ImGui::NextColumn();
        right();
        ImGui::Columns(1);
    }

    void vec3Row(const char *label, double x, double y, double z, const char *fmt = "%.3f");

    void steadyStats(const std::vector<AttitudeSample> &s,
                     double &ym, double &ysd, double &pm, double &psd, double &rm, double &rsd);
}
