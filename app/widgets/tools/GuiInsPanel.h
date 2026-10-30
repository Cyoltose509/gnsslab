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

    struct CalibParams {
        double g = SETTING_G;
        double turnAngleDeg = 360.0;
    };

    struct AlignParams {
        double latDeg = SETTING_LAT_DEG;
        double g = SETTING_G;
        int win = 400;
    };

    enum class AlertLevel { Error, Warn, Info };

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
        std::mutex resultMutex;
        std::atomic<bool> hasResult{false};
        std::atomic<long long> sampleCount{0};
        std::atomic<bool> chartDirty{true};
        std::atomic<bool> runFailed{false};
    };

    struct SixPosCalibTask : GnssTask::SolveTask, InsRunState {
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


        bool peekReport(CalibReport &r);


        bool snapshotReport(CalibReport &r);
    };

    struct StaticAlignTask : GnssTask::SolveTask, InsRunState {
        std::string alignPath;
        AlignParams par;
        ScaleFallback scaleSf;

        Attitude att{};
        std::vector<AttitudeSample> series;
        std::vector<AttitudeSample> plotPts;

        std::string sensorModel;
        ScaleNotice scaleNotice;

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
