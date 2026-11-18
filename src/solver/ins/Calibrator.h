#pragma once
#include <string>
#include <utility>
#include <vector>

#include "core/ins/InsMath.h"
#include "i18n/I18n.h"


struct CalibReport {
    ImuCalibration calib;
    Eigen::Vector3d accelScaleCounts = Eigen::Vector3d::Zero();
    Eigen::Vector3d accelScaleG = Eigen::Vector3d::Zero();
    Eigen::Vector3d accelScaleSigma = Eigen::Vector3d::Zero();
    double accelRms = 0.0;
    double dt = 0.005;
    Eigen::Vector3d gyroKeff = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroBiasPhys = Eigen::Vector3d::Zero();


    Eigen::Matrix3d gyroM = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d gyroCross = Eigen::Matrix3d::Zero();


    Eigen::Matrix3d accelCross = Eigen::Matrix3d::Zero();
    Eigen::Vector3d accelBiasPhys = Eigen::Vector3d::Zero();

    double nominalAccelK = 0.0;
    double nominalKeff = 0.0;
    Eigen::Vector3d accelDeltaS = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroDeltaS = Eigen::Vector3d::Zero();

    int nPos[6] = {0, 0, 0, 0, 0, 0};
    int nRot[6] = {0, 0, 0, 0, 0, 0};
    std::string errorMsg; // 空即成功；非空是缺段清单，界面直接显示
};

class Calibrator {
public:
    enum Pos { XUP, XDN, YUP, YDN, ZUP, ZDN };
    enum Rot { XFWD, XREV, YFWD, YREV, ZFWD, ZREV };

    struct CalibSegments {
        std::vector<std::pair<Pos, ImuLog> > pos;
        std::vector<std::pair<Rot, ImuLog> > rot;
    };

    struct DetectParams {
        int winSamples = 50;
        double minSegSeconds = 0.15;
        int minSegSamples = 30;
        double rotThrCounts = 30.0;
        double rotFrac = 0.4;
    };


    struct CalibSegment {
        bool isStatic = false;
        Pos pos = XUP;
        Rot rot = XFWD;
        std::size_t i0 = 0, i1 = 0;
    };

    static const char *posLabel(const Pos p) {
        static const char *k[6] = {"ins.seg.x_up", "ins.seg.x_down", "ins.seg.y_up",
                                   "ins.seg.y_down", "ins.seg.z_up", "ins.seg.z_down"};
        return I18n::trc(k[static_cast<int>(p)]);
    }

    static const char *rotLabel(const Rot r) {
        static const char *k[6] = {"ins.seg.x_cw", "ins.seg.x_ccw", "ins.seg.y_cw",
                                   "ins.seg.y_ccw", "ins.seg.z_cw", "ins.seg.z_ccw"};
        return I18n::trc(k[static_cast<int>(r)]);
    }


    static std::vector<CalibSegment> detect(const ImuLog &log);


    static Eigen::Vector3d posForce(Pos p, double g);

    static void rawMean(const ImuLog &log, Eigen::Vector3d &acc, Eigen::Vector3d &gyr);


    static void solveLinear(const std::vector<Eigen::Vector3d> &raw, const std::vector<Eigen::Vector3d> &f,
                            const std::vector<double> &w,
                            Eigen::Matrix3d &M, Eigen::Vector3d &b, double &rmsResid,
                            Eigen::Vector3d *scaleSigma = nullptr);


    static CalibReport calibrate(const CalibSegments &seg,
                                 double turnAngle,
                                 double g,
                                 double rateHz);





    static void applyNominal(CalibReport &r, const ImuScale &scale);




    struct CompMat {
        Eigen::Matrix3d accelMinv;
        Eigen::Matrix3d gyroIgN;
    };
    static CompMat compMatrices(const CalibReport &r);
    static void compensate(const CompMat &m, const CalibReport &r, const ImuSample &s,
                           Eigen::Vector3d &f, Eigen::Vector3d &w);
};
