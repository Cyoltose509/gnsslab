#pragma once

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "Ephemeris.h"
#include <Eigen/Eigen>

class SPPIFCode {
public:
    void setStationAsBase() {
        isRover = false;
    }

    void setEphemeris(const SatEphemerisMap &ephs) {
        ephMap = ephs;
    }

    void setIFCodeTypes(const std::map<char, std::pair<string, string> > &ifTypes) {
        ifCodeTypes = ifTypes;
    }

    void preprocess(ObsData &obsData);

    void solve(ObsData &obsData);

    void getResult();

    void computeSatPos(ObsData &obsData);

    void computeElevAzim();


    void computeIF(ObsData &obsData);

    void earthRotation();

    void linearize(ObsData &obsData, double rms);

    void setFrame(const FrameInfo &f) {
        frame = f;
    }

    ~SPPIFCode() = default;

    Vector3d xyz{0, 0, 0};
    Vector3d vel{0, 0, 0};
    FrameInfo frame = Frame::WGS84;
    Result result{};
    SatValueMap satElevData{};
    SatValueMap satAzimData{};
    std::set<SatID> satRejected{};
    std::map<SatID, PVT> satPVTTransTime{};
    std::map<SatID, PVT> satPVTRecTime{};
    bool oldVersion=false;
    // 继承类需要访问这个成员
protected:
    double cutOffElev = PI * 0.0555556;
    bool isRover = true;
    double sigIFCode = 0.3;

    double rClockBias{}; // 接收机钟差
    double rClockDrift{}; //接收机钟漂
    double rClockBiasBDS{};

    EquSys posEquations{};
    EquSys velEquations{};
    SolverLSQ posSolver{};
    SolverLSQ velSolver{};


    SatEphemerisMap ephMap{};

    std::map<char, std::pair<string, string> > ifCodeTypes{};
};
