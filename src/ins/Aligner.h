#pragma once
#include <vector>
#include "core/CoordConvert.h"
#include "core/InsMath.h"
#include "core/InsStruct.h"








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





class Aligner {
public:

    static Attitude solve(const ImuLog &log,
                          size_t begin, size_t end,
                          const SiteParams &sp);

    static AttQuat solveQuat(const ImuLog &log,
                             size_t begin, size_t end,
                             const SiteParams &sp);


    static AlignQuality diagnose(const ImuLog &log,
                                 size_t begin, size_t end,
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
        size_t count = 0;

        void init(int window, const SiteParams &site);

        void step(const ImuSample &s, AttitudeSample &out);
    };
};
