#pragma once


#include "GuiInsPanel.h"
#include "GuiHelpers.h"

#include <Eigen/Eigen>

#include "Const.h"
#include "core/Csv.h"
#include "io/AsciiLogReader.h"
#include "io/OEM7Reader.h"
#include "util/StringUtils.h"

#include "implot.h"
#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace Tools::InsPanel {
    template<class Reader, class ReadFn, class OnSample, class OnBatch>
    void pumpImu(Reader &r, ReadFn readOne, const std::atomic<bool> &stop,
                 OnSample onSample, OnBatch onBatch) {
        size_t last = 0, batchMark = 0;
        while (!stop) {
            const bool more = readOne();
            const ImuLog &L = r.getImuLog();
            for (size_t i = last; i < L.size(); ++i) onSample(L[i]);
            last = L.size();
            if (!more) break;
            if (L.size() - batchMark >= 2048) {
                onBatch(static_cast<long long>(L.size()));
                batchMark = L.size();
            }
        }
        onBatch(static_cast<long long>(r.getImuLog().size()));
    }

    template<class Reader, class OnSample, class OnBatch>
    void pumpImuReader(Reader &r, const std::atomic<bool> &stop,
                       OnSample onSample, OnBatch onBatch) {
        ImuSample t;
        pumpImu(r, [&] { return r.getNextImu(t); }, stop, onSample, onBatch);
    }

    template<class OnReader, class OnSample, class OnBatch>
    bool pumpImuFile(const std::string &path, const std::atomic<bool> &stop,
                     OnReader onReader, OnSample onSample, OnBatch onBatch) {
        auto run = [&](auto &r) {
            if (!r.open(path) || !onReader(r)) return false;
            pumpImuReader(r, stop, onSample, onBatch);
            return true;
        };
        if (endsWithIC(path, ".asc")) {
            AsciiLogReader r;
            return run(r);
        }
        OEM7Reader r;
        return run(r);
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
        LOG_INFO << "[" << task.fileName << "] 开始";
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
        auto self = task.shared_from_this();
        task.worker = std::thread([self, fn] { fn(); });
    }

    constexpr ImVec4 kColPass(0.35f, 0.85f, 0.45f, 1.00f);
    constexpr ImVec4 kColFail(0.95f, 0.40f, 0.35f, 1.00f);
    constexpr ImVec4 kColWarn(0.98f, 0.76f, 0.32f, 1.00f);
    constexpr ImVec4 kColInfo(0.55f, 0.72f, 1.00f, 1.00f);
    void drawAlert(AlertLevel lv, const char *text);

    template<class Task>
    bool drawResultError(const Task &task) {
        if (!task.errorFlag()) return false;
        drawAlert(AlertLevel::Error, task.errorMessage().c_str());
        return true;
    }

    template<class Task>
    bool errorBlocksResult(const Task &task) {
        return task.errorFlag() && task.runFailed.load();
    }

    template<class Task>
    void drawRunError(const Task &task) {
        if (task.errorFlag() && !task.hasResult.load())
            drawAlert(AlertLevel::Error, task.errorMessage().c_str());
    }

    template<class Task, class FileTabFn, class ParamTabFn, class StartFn>
    void renderInsConfig(const char *title, Task &task, const char *startLabel,
                         FileTabFn fileTab, ParamTabFn paramTab, StartFn onStart) {
        Solvers::Helpers::beginConfigBody(title, task);
        if (ImGui::BeginTabBar("##cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                ImGui::Spacing();
                fileTab();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("参数")) {
                ImGui::PushItemWidth(260);
                paramTab();
                ImGui::PopItemWidth();
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::Spacing();
        if (ImGui::Button(startLabel, ImVec2(200, 40))) onStart();
        ImGui::SameLine();
        Solvers::Helpers::renderCancelButton(&task);
        drawRunError(task);
        Solvers::Helpers::endConfigBody();
    }

    void renderInsParams(CalibParams &par);

    void renderInsParams(AlignParams &par);

    void saveInsParams(const CalibParams &par);

    void saveInsParams(const AlignParams &par);

    void loadInsParams(CalibParams &par);

    void loadInsParams(AlignParams &par);

    enum class ScaleSource { None, Record, Manual, Guess };
    enum class RecordScale { Model, UnknownType, NoType };

    struct RecordScaleInfo {
        RecordScale kind = RecordScale::NoType;
        const ImuModel *model = nullptr;
        int type = -1;
    };

    template<class Reader>
    RecordScaleInfo readRecordScale(const Reader &r) {
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

    constexpr size_t kScaleProbeSamples = 4000;
    struct ScaleIdentifier {
        explicit ScaleIdentifier(ScaleFallback &fallback) : sf(fallback) {}

        ScaleFallback &sf;
        ScaleResolution res;
        std::vector<ImuLog> candidates;
        ImuLog probe;
        template<class Reader, class Sink>
        bool observe(const Reader &r, Sink sink) {
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

        [[nodiscard]] std::string openFailure(const std::string &path) const;

        [[nodiscard]] std::string label() const;
        [[nodiscard]] ImuScale currentScale() const;
        [[nodiscard]] ImuScale scale() const;
        [[nodiscard]] ScaleNotice notice(bool settled) const;
    };

    void keepManualScaleFor(ScaleFallback &sf, const std::string &sig);

    bool drawScaleFallback(ScaleFallback &sf, bool allowSkip);

    template<class Task, class RestartFn>
    bool renderResultPrologue(Task &task, const bool allowSkipScale, RestartFn restart) {
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
        return errored && errorBlocksResult(task);
    }

    constexpr float kChartHeight = 360.0f;
    constexpr float kFileChartH = 220.0f;

    size_t decimateStride(size_t n, size_t maxPts = 4000);

    void decimateImu(const ImuLog &src, ImuLog &out);

    constexpr ImVec4 kColX(0.90f, 0.30f, 0.30f, 1.00f);
    constexpr ImVec4 kColY(0.30f, 0.90f, 0.30f, 1.00f);
    constexpr ImVec4 kColZ(0.30f, 0.50f, 1.00f, 1.00f);

    ImPlotSpec lineSpec(const ImVec4 &c);

    void toElapsed(const std::vector<double> &tow, std::vector<double> &out);

    void fitXToLatest(bool newData, double xmax);

    template<class Task, class BuildFn>
    void exportCsvDialog(Task &task, const std::string &defaultName, const std::string &displayName,
                         BuildFn build) {
        std::wstring out;
        if (!Solvers::Helpers::saveCSVDialog(defaultName, out)) return;
        Csv::Writer w;
        build(w);
        if (!w.save(out, true)) {
            task.setError(displayName +
                          " 导出失败：无法写入文件（目录不存在、无写权限，或被 Excel 等程序占用）");
            return;
        }
        task.clearError();
        LOG_INFO << "[" << task.fileName << "] 导出 " << displayName << " → "
                 << std::filesystem::path(out).string();
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
        const char *ui{};
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
        {"无量纲", "无量纲", 1.0, false, false, "%.6f"},
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
        {"LSB/历元", "LSB/历元", 1.0, false, false, "%.3f"},
        {"rad/s", "rad/s", 1.0, false, true, "%.9f"},
        {"°/h", "deg/h", RAD_TO_DEG * 3600.0, false, true, "%.3f"},
    };

    inline constexpr CalibQty kCalibQty[Q_COUNT] = {
        {"加速度计比例因子 K_a", "加速度计比例因子",
            topVec(&CalibReport::accelScaleG), topVec(&CalibReport::accelScaleG),
            kAccScaleUnits, 3, 4, false, false, Q_ACC_SCALE_SIGMA},
        {"加速度计比例因子 1σ", "加速度计比例因子1σ",
            topVec(&CalibReport::accelScaleSigma, true), topVec(&CalibReport::accelScaleSigma, true),
            kAccScaleUnits, 3, 4, false, false, -1},
        {"加速度计零偏 b_a", "加速度计零偏",
            calVec(&ImuCalibration::accelB), topVec(&CalibReport::accelBiasPhys),
            kAccBiasUnits, 5, 4, false, false, -1},
        {"标称比例因子 K_nom", "标称比例因子K_nom",
            topScalar(&CalibReport::nominalAccelK, true), topScalar(&CalibReport::nominalAccelK, true),
            kAccScaleUnits, 3, 6, false, true, -1},
        {"加速度计比例因子误差 δs_a", "加速度计比例因子误差delta_s",
            topVec(&CalibReport::accelDeltaS), topVec(&CalibReport::accelDeltaS),
            kDsUnits, 3, 9, true, false, -1},
        {"陀螺有效比例因子 K_eff,g", "陀螺有效比例因子K_eff,g",
            topVec(&CalibReport::gyroKeff), topVec(&CalibReport::gyroKeff),
            kGyroKeffUnits, 2, 6, true, false, -1},
        {"标称有效比例因子 K_eff,nom", "标称有效比例因子K_eff,nom",
            topScalar(&CalibReport::nominalKeff), topScalar(&CalibReport::nominalKeff),
            kGyroKeffUnits, 2, 6, true, true, -1},
        {"陀螺比例因子 S_g", "陀螺比例因子S_g",
            calVec(&ImuCalibration::gyroScale), calVec(&ImuCalibration::gyroScale),
            kGyroScaleUnits, 2, 6, true, false, -1},
        {"陀螺零偏 b_g", "陀螺零偏",
            calVec(&ImuCalibration::gyroBias), topVec(&CalibReport::gyroBiasPhys),
            kGyroBiasUnits, 3, 4, false, false, -1},
        {"陀螺比例因子误差 δs_g", "陀螺比例因子误差delta_s",
            topVec(&CalibReport::gyroDeltaS), topVec(&CalibReport::gyroDeltaS),
            kDsUnits, 3, 6, true, false, -1},
    };

    inline constexpr int kCalibUiRows[] = {
        Q_ACC_SCALE, Q_ACC_BIAS, Q_ACC_DS, Q_GYRO_KEFF, Q_GYRO_SCALE, Q_GYRO_BIAS, Q_GYRO_DS
    };

    inline int gCalibQtyUnit[Q_COUNT] = {0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

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
