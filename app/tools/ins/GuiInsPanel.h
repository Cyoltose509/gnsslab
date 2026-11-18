#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>


#include "solvers/common/Task.h"
#include "solvers/ins/ImuScaleProbe.h"
#include "solvers/view/GuiHelpers.h"
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

    /// 标定/对准/校核共用的配置骨架：文件页 + 参数页 + 起算/取消/错误行，各页只补三件差异。
    struct InsToolTask : Solvers::Task, InsRunState {
        const char *cfgKey = "";   // 窗口标题 key，同时拼配置页 child id
        const char *startKey = ""; // 起算按钮文案 key

        void renderConfigPanel() override;

        virtual void renderFileTab() = 0;

        virtual void renderParamTab() = 0;

        virtual void startRun() = 0;

        void beginRun();

        void endRun();

        void abortRun(const std::string &msg);

        void finishRun();

        bool stoppedAndEnd();

        bool drawResultError();

        bool renderResultPrologue();
    };

    struct SixPosCalibTask : InsToolTask {
        std::vector<std::string> filePaths;
        CalibParams par;
        ImuScale scale;
        Solvers::INS::ScaleDetect scaleDet;
        std::string scaleText; // 起算那一刻的当量说明，后台线程只读它

        CalibReport report;
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

        void renderFileTab() override;

        void renderParamTab() override;

        void startRun() override;

        void renderResult() override;

    private:
        void calibWorker();

        void exportCalibCSV();
        void exportDetectCSV();
        void exportCompCSV();

        std::vector<std::string> compSegLabels();


        bool peekReport(CalibReport &r);


        bool snapshotReport(CalibReport &r);
    };

    struct StaticAlignTask : InsToolTask {
        std::string alignPath;
        AlignParams par;
        ImuScale scale;
        Solvers::INS::ScaleDetect scaleDet;
        std::string scaleText; // 起算那一刻的当量说明，后台线程只读它

        Euler att{};
        std::vector<AttitudeSample> series;
        std::vector<AttitudeSample> plotPts;

        double steadyY = 0, steadyYsd = 0, steadyP = 0, steadyPsd = 0, steadyR = 0, steadyRsd = 0;


        AlignQuality alignQ;
        bool alignQValid = false;

        StaticAlignTask();

        Kind kind() const override { return Kind::Tool; }

        void renderFileTab() override;

        void renderParamTab() override;

        void startRun() override;

        void renderResult() override;

    private:
        void alignWorker();


        void exportAlignSeries();
        void exportAlignSummary();
        void exportAlignPerSec();

        std::vector<AttitudeSample> snapshotSeries();
    };

    /// 示例数据验证：拿 IMU 增量跑 src 的机械编排，与参考轨迹逐点比对。
    struct InsVerifyTask : InsToolTask {
        std::string imuPath;
        std::string refPath;
        VerifyParams par;

        ::Ins::VerifyReport report;
        bool reportValid = false;

        InsVerifyTask();

        Kind kind() const override { return Kind::Tool; }

        void renderFileTab() override;

        void renderParamTab() override;

        void startRun() override;

        void renderResult() override;

    private:
        void verifyWorker();

        void exportSampleCSV();
        void exportSummaryCSV();
    };


    std::shared_ptr<Solvers::Task> MakeSixPosCalibTask();

    std::shared_ptr<Solvers::Task> MakeStaticAlignTask();

    std::shared_ptr<Solvers::Task> MakeInsVerifyTask();

}
