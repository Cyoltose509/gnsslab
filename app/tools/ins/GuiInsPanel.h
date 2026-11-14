#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>


#include "solvers/common/Task.h"
#include "core/ins/InsStruct.h"
#include "solver/ins/Aligner.h"
#include "solver/ins/Calibrator.h"
#include "solver/ins/InsVerify.h"

namespace Tools::Ins {



    constexpr double SETTING_G = 9.7936174;
    constexpr double SETTING_LAT_DEG = 30.531651244;

    struct CalibParams {
        double g = SETTING_G;
        double turnAngleDeg = 360.0;
    };

    struct AlignParams {
        double latDeg = SETTING_LAT_DEG;
        double g = SETTING_G;
        int win = 400;
    };

    /// 示例数据验证的初始条件
    struct VerifyParams {
        double latDeg = 23.1373950708;
        double lonDeg = 113.3713651222;
        double hM = 2.175;
        double vN = 0.0, vE = 0.0, vD = 0.0;
        double rollDeg = 0.0107951084511778;
        double pitchDeg = -2.14251290749072;
        double headingDeg = -75.7498049314083;
        double startTime = 91620.0;
    };

    enum class AlertLevel { Error, Warn, Info };

    struct ScaleNotice {
        bool show = false;
        AlertLevel level = AlertLevel::Info;
        std::string text;
    };


    /// 当量兜底对话框的共享状态：工作线程提议、界面线程改并回填，两条线程都读，故三个当量走原子。
    struct ScaleFallback {
        std::atomic<bool> asked{false};
        std::atomic<bool> accepted{false};
        std::atomic<double> accelMpsPerLsb{0.0};
        std::atomic<double> gyroDegPerLsb{0.0};
        std::atomic<double> rateHz{0.0};
        std::string reason;
        std::string sig;


        void request(const double suggestAccel, const double suggestRate, std::string why = {}) {
            if (!(accelMpsPerLsb.load(std::memory_order_relaxed) > 0.0))
                accelMpsPerLsb.store(suggestAccel, std::memory_order_relaxed);
            if (!(rateHz.load(std::memory_order_relaxed) > 0.0))
                rateHz.store(suggestRate, std::memory_order_relaxed);
            reason = std::move(why);
            asked.store(true, std::memory_order_release);
        }

        [[nodiscard]] ImuScale scale() const {
            return {accelMpsPerLsb.load(std::memory_order_relaxed),
                    gyroDegPerLsb.load(std::memory_order_relaxed) * DEG_TO_RAD,
                    rateHz.load(std::memory_order_relaxed)};
        }

        void reset() {
            accepted.store(false, std::memory_order_relaxed);
            accelMpsPerLsb.store(0.0, std::memory_order_relaxed);
            gyroDegPerLsb.store(0.0, std::memory_order_relaxed);
            rateHz.store(0.0, std::memory_order_relaxed);
            reason.clear();
            sig.clear();
        }
    };


    template<class T>
    struct OnlineDecimator {
        std::vector<T> buf;
        std::size_t stride = 1;
        std::size_t counter = 0;
        std::size_t maxPts = 4000;

        void reset() {
            buf.clear();
            stride = 1;
            counter = 0;
        }

        void push(const T &s) {
            if (counter++ % stride == 0) buf.push_back(s);
            if (buf.size() >= 2 * maxPts) {
                std::size_t w = 0;
                for (std::size_t i = 0; i < buf.size(); i += 2) buf[w++] = buf[i];
                buf.resize(w);
                stride *= 2;
            }
        }
    };

    struct InsRunState {
        std::mutex resultMutex;
        std::atomic<bool> hasResult{false};
        std::atomic<long long> sampleCount{0};
        std::atomic<bool> chartDirty{true};
        std::atomic<bool> runFailed{false};
    };

    struct SixPosCalibTask : Solvers::Task, InsRunState {
        std::vector<std::string> filePaths;
        CalibParams par;
        ScaleFallback scaleSf;

        CalibReport report;
        ScaleNotice scaleNotice;
        std::atomic<bool> accelReady{false};

        std::atomic<int> segStatic{0};
        std::atomic<int> segRot{0};
        std::atomic<int> dupSkipped{0};
        std::string currentFile;



        std::vector<std::pair<std::string, std::string> > detectSeq;
        std::atomic<bool> detectDone{false};


        Calibrator::CalibSegments compSegs;
        int compSel = 0;

        Eigen::Vector3d accelScaleGp{Eigen::Vector3d::Zero()};
        double accelRmsp = 0.0;

        SixPosCalibTask();

        Kind kind() const override { return Kind::Tool; }

        void renderConfigPanel() override;

        void renderResult() override;

    private:
        void startCalib();


        void calibWorker();


        void exportCalibCSV();
        void exportDetectCSV();
        void exportCompCSV();

        std::vector<std::string> compSegLabels();


        bool peekReport(CalibReport &r);


        bool snapshotReport(CalibReport &r);
    };

    struct StaticAlignTask : Solvers::Task, InsRunState {
        std::string alignPath;
        AlignParams par;
        ScaleFallback scaleSf;

        Euler att{};
        std::vector<AttitudeSample> series;
        std::vector<AttitudeSample> plotPts;

        std::string sensorModel;
        ScaleNotice scaleNotice;

        double steadyY = 0, steadyYsd = 0, steadyP = 0, steadyPsd = 0, steadyR = 0, steadyRsd = 0;


        AlignQuality alignQ;
        bool alignQValid = false;

        StaticAlignTask();

        Kind kind() const override { return Kind::Tool; }

        void renderConfigPanel() override;

        void renderResult() override;

    private:
        void startAlign();

        void alignWorker();


        void exportAlignSeries();
        void exportAlignSummary();
        void exportAlignPerSec();

        std::vector<AttitudeSample> snapshotSeries();
    };

    /// 示例数据验证：拿 IMU 增量跑 src 的机械编排，与参考轨迹逐点比对。
    struct InsVerifyTask : Solvers::Task, InsRunState {
        std::string imuPath;
        std::string refPath;
        VerifyParams par;

        ::Ins::VerifyReport report;
        bool reportValid = false;

        InsVerifyTask();

        Kind kind() const override { return Kind::Tool; }

        void renderConfigPanel() override;

        void renderResult() override;

    private:
        void startVerify();

        void verifyWorker();

        void exportSampleCSV();
        void exportSummaryCSV();
    };


    std::shared_ptr<Solvers::Task> MakeSixPosCalibTask();

    std::shared_ptr<Solvers::Task> MakeStaticAlignTask();

    std::shared_ptr<Solvers::Task> MakeInsVerifyTask();

}
