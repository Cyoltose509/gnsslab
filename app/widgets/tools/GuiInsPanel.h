#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>


#include "SolverTask.h"
#include "core/InsStruct.h"
#include "ins/Aligner.h"
#include "ins/ImuCalibrator.h"

namespace Tools::InsPanel {



    constexpr double SETTING_G = 9.7936174;
    constexpr double SETTING_LAT_DEG = 30.531651244;

    /// 提示框的说话口径：错误 / 注意 / 提示。画法在 GuiInsCommon.h::drawAlert，
    /// 这里只是让任务结构（ScaleNotice）也能带上档位。
    enum class AlertLevel { Error, Warn, Info };

    /// 结果区顶部横幅的内容：文案与档位在 worker 侧算好（GuiInsCommon.h::ScaleIdentifier::notice），
    /// 界面每帧只负责画，不再各自拼字符串。
    struct ScaleNotice {
        bool show = false;
        AlertLevel level = AlertLevel::Info;
        std::string text;
    };


    struct ScaleFallback {
        std::atomic<bool> asked{false};
        std::atomic<bool> accepted{false};
        double accelMpsPerLsb = 0.0;
        double gyroDegPerLsb = 0.0;
        double rateHz = 0.0;
        std::string reason;
        std::string sig;

        /// worker 侧：这一轮当量没定下来，请界面弹输入框（suggest* 只做预填，0 = 留空）
        void request(const double suggestAccel, const double suggestRate, std::string why = {}) {
            if (!(accelMpsPerLsb > 0.0)) accelMpsPerLsb = suggestAccel;
            if (!(rateHz > 0.0)) rateHz = suggestRate;
            reason = std::move(why);
            asked.store(true, std::memory_order_release);
        }

        [[nodiscard]] ImuScale scale() const {
            return {accelMpsPerLsb, gyroDegPerLsb * DEG_TO_RAD, rateHz};
        }

        void reset() {
            accepted.store(false, std::memory_order_relaxed);
            accelMpsPerLsb = gyroDegPerLsb = rateHz = 0.0;
            reason.clear();
            sig.clear();
        }
    };


    template<class T>
    struct OnlineDecimator {
        std::vector<T> buf;
        size_t stride = 1;
        size_t counter = 0;
        size_t maxPts = 4000;

        void reset() {
            buf.clear();
            stride = 1;
            counter = 0;
        }

        void push(const T &s) {
            if (counter++ % stride == 0) buf.push_back(s);
            if (buf.size() >= 2 * maxPts) {
                size_t w = 0;
                for (size_t i = 0; i < buf.size(); i += 2) buf[w++] = buf[i];
                buf.resize(w);
                stride *= 2;
            }
        }
    };

    struct InsRunState {
        std::mutex resultMutex;                // 结果区：worker 写、界面读
        std::atomic<bool> hasResult{false};    // 有可用结果
        std::atomic<long long> sampleCount{0}; // 已解析历元数（进度显示）
        std::atomic<bool> chartDirty{true};    // 有新数据 → 曲线重设横轴
        // 错误分两级：worker 中止（运行没了，结果区没意义 → 收起）
        // vs 操作失败（导出写盘失败等，结果还在 → 只提示）
        std::atomic<bool> runFailed{false};
    };

    struct SixPosCalibTask : GnssTask::SolveTask, InsRunState {
        std::vector<std::string> filePaths;
        double g = SETTING_G;
        double turnAngleDeg = 360.0;
        ScaleFallback scaleSf;

        CalibReport report;
        ScaleNotice scaleNotice; // 结果区顶部横幅（当量来源 / 缺型号自描述）
        std::atomic<bool> accelReady{false};

        std::atomic<int> segStatic{0};
        std::atomic<int> segRot{0};
        std::atomic<int> dupSkipped{0};
        std::string currentFile;



        std::vector<std::pair<std::string, std::string> > detectSeq;
        std::atomic<bool> detectDone{false};


        ImuCalibrator::CalibSegments compSegs;
        int compSel = 0;

        Eigen::Matrix3d accelMp{Eigen::Matrix3d::Identity()};
        Eigen::Vector3d accelBp{Eigen::Vector3d::Zero()};
        Eigen::Vector3d accelScaleGp{Eigen::Vector3d::Zero()};
        double accelRmsp = 0.0;

        SixPosCalibTask();

        void renderConfigPanel() override;

        void renderResult();

    private:
        void startCalib();


        void calibWorker();


        void exportCalibCsv();
        void exportDetectCsv();
        void exportCompCsv();

        std::vector<std::string> compSegLabels();

        /// 只读快照：渲染路径每帧都会问「有没有结果」，这里绝不能有副作用。
        bool peekReport(CalibReport &r);

        /// 导出用快照：没有结果时 setError，由用户的导出动作触发。
        bool snapshotReport(CalibReport &r);
    };

    struct StaticAlignTask : GnssTask::SolveTask, InsRunState {
        std::string alignPath;
        double latDeg = SETTING_LAT_DEG;
        double g = SETTING_G;
        int win = 400;
        ScaleFallback scaleSf;

        Attitude att{};
        std::vector<AttitudeSample> series;
        std::vector<AttitudeSample> plotPts;

        std::string sensorModel; // IMU 型号（导出摘要 CSV 用）
        ScaleNotice scaleNotice; // 结果区顶部横幅：当量来源，流式期间先只报记录自带的型号

        double steadyY = 0, steadyYsd = 0, steadyP = 0, steadyPsd = 0, steadyR = 0, steadyRsd = 0;


        AlignQuality alignQ;
        bool alignQValid = false;

        StaticAlignTask();

        void renderConfigPanel() override;

        void renderResult();

    private:
        void startAlign();

        void alignWorker();


        void exportAlignSeries();
        void exportAlignSummary();
        void exportAlignPerSec();

        std::vector<AttitudeSample> snapshotSeries();
    };


    std::shared_ptr<GnssTask::SolverTaskBase> MakeSixPosCalibTask();

    std::shared_ptr<GnssTask::SolverTaskBase> MakeStaticAlignTask();


    bool RenderInsResult(const std::shared_ptr<GnssTask::SolverTaskBase> &job);
}
