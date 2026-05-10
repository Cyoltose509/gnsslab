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

    void setEphemeris(const std::map<SatID, Ephemeris *> &ephs) {
        ephMap = ephs;
    }

    void setIFCodeTypes(const std::map<char, std::pair<string, string> > &ifTypes) {
        ifCodeTypes = ifTypes;
    }

    void preprocess(ObsData &obsData);

    void solve(ObsData &obsData);

    std::map<SatID, PVT> computeSatPos(ObsData &obsData);

    void computeElevAzim();

    static void convertObsType(ObsData &obsData);

    void computeIF(ObsData &obsData);

    std::map<SatID, PVT> earthRotation();

    void linearize(ObsData &obsData);


    [[nodiscard]] const SatValueMap &getElevData() const { return satElevData; }
    [[nodiscard]] const SatValueMap &getAzimData() const { return satAzimData; }
    [[nodiscard]] const std::map<SatID, PVT> &getSatPVT() const { return satPVTRecTime; }

    [[nodiscard]] SatID getDatumSat() const {
        double maxElev(0.0);
        SatID datumSat;
        for (const auto [sat, elev]: satElevData) {
            if (elev > maxElev) {
                maxElev = elev;
                datumSat = sat;
            }
        }
        return datumSat;
    }

    void setFrame(const ReferenceFrame &f) {
        frame = f;
    }

    ~SPPIFCode() = default;

    Vector3d xyz{0, 0, 0};
    Vector3d vel{0, 0, 0};
    ReferenceFrame frame = Frame::WGS84;
    Result result{};
    SatValueMap satElevData{};
    SatValueMap satAzimData{};
    std::set<SatID> satRejected{};
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

    std::map<SatID, PVT> satPVTTransTime{};
    std::map<SatID, PVT> satPVTRecTime{};




    std::map<SatID, Ephemeris *> ephMap{};

    std::map<char, std::pair<string, string> > ifCodeTypes{};
};
