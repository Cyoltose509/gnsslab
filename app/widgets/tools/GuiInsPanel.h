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


    constexpr double SETTING_ER = 7.2921150e-5;       // 地球自转角速率 (rad/s)
    constexpr double SETTING_G = 9.7936174;           // 当地重力加速度 (m/s^2)
    constexpr double SETTING_LAT_DEG = 30.531651244;  // 当地纬度 (°N)

    constexpr double SETTING_KA_LSB = 0.05 / 32768.0;            // 1.52587890625e-6 (m/s)/LSB
    constexpr double SETTING_KG_DEG_LSB = 0.1 / 3600.0 / 256.0;  // 1.0850694444e-7 (°)/LSB


    struct PerFileChart {
        std::string name;  // 文件名（不含路径）
        std::string det;   // 检测到的段描述
        bool dup = false;  // 是否重复文件
        std::vector<double> t;
        std::vector<double> ax, ay, az; // 加速度计三轴原始计数
        std::vector<double> gx, gy, gz; // 陀螺仪三轴原始计数
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

    struct SixPosCalibTask : GnssTask::SolveTask {
        std::vector<std::string> filePaths; // 选中的文件
        double g = SETTING_G;               // 当地重力加速度 (m/s^2)
        double latDeg = SETTING_LAT_DEG;    // 当地纬度 (°N)
        double earthRate = SETTING_ER;      // 地球自转角速率 (rad/s)
        double turnAngleDeg = 360.0;        // 转台精确转角 (°)

        double kAccelLsb = SETTING_KA_LSB;      // (m/s)/LSB
        double kGyroLsb = SETTING_KG_DEG_LSB;   // (°)/LSB

        std::mutex resultMutex;
        CalibReport report;
        std::atomic<bool> hasResult{false};
        std::atomic<bool> accelReady{false};

        std::atomic<long long> sampleCount{0};
        std::atomic<int> segStatic{0};
        std::atomic<int> segRot{0};
        std::atomic<int> dupSkipped{0};
        std::string currentFile;


        std::vector<std::string> detectLines;

        std::vector<std::pair<std::string, std::string> > detectSeq;
        std::atomic<bool> detectDone{false};


        ImuCalibrator::CalibSegments compSegs;
        int compSel = 0;

        std::vector<PerFileChart> fileCharts;

        Eigen::Matrix3d accelMp{Eigen::Matrix3d::Identity()};
        Eigen::Vector3d accelBp{Eigen::Vector3d::Zero()};
        Eigen::Vector3d accelScaleGp{Eigen::Vector3d::Zero()};
        double accelRmsp = 0.0;
        std::atomic<bool> chartDirty{true};

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

        bool snapshotReport(CalibReport &r);
    };

    struct StaticAlignTask : GnssTask::SolveTask {
        std::string alignPath;
        double g = SETTING_G;
        double latDeg = SETTING_LAT_DEG;
        double earthRate = SETTING_ER;
        int win = 400;                  // 滑窗长度（样本）

        std::mutex resultMutex;
        Attitude att{};
        std::vector<AttitudeSample> series; // 完整逐历元序列（Done 后用于 CSV / 稳态统计）
        std::vector<AttitudeSample> plotPts; // 抽稀绘图缓冲（流式期间每帧只拷这份）
        std::atomic<bool> hasResult{false};


        std::atomic<long long> sampleCount{0};

        std::atomic<bool> chartDirty{true};


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
