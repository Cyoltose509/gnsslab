#pragma once
#include <vector>
#include "core/ins/InsMath.h"
#include "core/ins/InsStruct.h"

struct AlignQuality {
    Eigen::Vector3d axisX = Eigen::Vector3d::UnitX();
    Eigen::Vector3d axisY = Eigen::Vector3d::UnitY();
    Eigen::Vector3d axisZ = Eigen::Vector3d::UnitZ();


    double dcmDet = 1.0;
    double dcmOrthoErr = 0.0;



    double earthRateErrDeg = 0.0;


    bool hasScale = false;
    double accelNorm = 0.0;
    double gravityRef = 0.0;
    double gyroNorm = 0.0;
    double gyroRef = 0.0;


    bool latDerivedValid = false;
    double latDerivedDeg = 0.0;
};





/// 静止段样本下标区间 [begin, end)；粗对准与静置段标定共用同一段。
struct StaticSpan {
    std::size_t begin = 0;
    std::size_t end = 0;
    /// 段末之后紧跟的连续运动样本数：还很大说明段没结束，缓冲要接着读。
    std::size_t tailMove = 0;

    [[nodiscard]] std::size_t size() const { return end > begin ? end - begin : 0; }
};


class Aligner {
public:
    /// 解析粗对准能用的最短静止时长 (s)，由 INS::align 与本类共用这一个数。
    static constexpr double kMinStaticSec = 20.0;
    static constexpr double kBlockSec = 1.0;       // 分块时长，只决定统计颗粒度
    static constexpr double kRotateFrac = 5.0;     // 判「在转」门限：地球自转角速率的 5 倍（≈75 °/h）
    static constexpr double kAccelFrac = 0.05;     // 判「在动」门限：比力模长偏离静置值 5%（约 0.5 m/s²）
    static constexpr std::size_t kRefBlocks = 20;  // 参考中值回看的块数，约 20 s
    static constexpr std::size_t kStatBlock = 200; // 求均值用的块长，约 1 s @200 Hz
    static constexpr std::size_t kMoveRun = 3;     // 连续多少块超限才算真运动，单块是触碰扰动

    static Euler solve(const ImuLog &log,
                          std::size_t begin, std::size_t end,
                          const SiteParams &sp);

    /// 按模型自身采样率分块统计角速率，块均值相对邻近块与近期中值的变化超过 5 倍地球自转角速率、
    /// 且连续 kMoveRun 块才算转动；取最长的那段静止（搬运会把记录切段，长的那段才是就位后的静置）。
    static StaticSpan detectStatic(const ImuLog &log, const ImuScale &scale);

    static ATTQuat solveQuat(const ImuLog &log,
                             std::size_t begin, std::size_t end,
                             const SiteParams &sp);


    static AlignQuality diagnose(const ImuLog &log,
                                 std::size_t begin, std::size_t end,
                                 const SiteParams &sp,
                                 const ImuScale &sen = {});

    static AlignQuality diagnoseMeans(const Eigen::Vector3d &accMean, const Eigen::Vector3d &gyrMean,
                                      const SiteParams &sp,
                                      const ImuScale &sen = {});


    static std::vector<AttitudeSample> alignSeries(const ImuLog &log, int win,
                                                   const SiteParams &sp);


    struct AlignStream {
        int win = 200;
        SiteParams sp{};
        Eigen::Vector3d acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
        std::size_t count = 0;

        void init(int window, const SiteParams &site);

        void step(const ImuSample &s, AttitudeSample &out);
    };
};
