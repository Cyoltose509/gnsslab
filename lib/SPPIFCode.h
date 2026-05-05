#pragma once

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "Ephemeris.h"
#include <Eigen/Eigen>

class SPPIFCode {
public:
    SPPIFCode()
        : cutOffElev(10), isRover(true), sigIFCode(1.0), result(), rcvrClk(0.0) {
    }

    void setStationAsBase() {
        isRover = false;
    }

    void setEphemeris(const std::map<SatID, Ephemeris *> &ephs) {
        ephMap = ephs;
    };

    void setIFCodeTypes(const std::map<char, std::pair<string, string> > &ifTypes) {
        ifCodeTypes = ifTypes;
    };

    void solve(ObsData &obsData);

    std::map<SatID, PVT> computeSatPos(ObsData &obsData);

    void computeElevAzim();

    static void convertObsType(ObsData &obsData);

    void computeIF(ObsData &obsData) const;

    std::map<SatID, PVT> earthRotation();

    EquSys linearize(ObsData &obsData);

    EquSys getEquSys() {
        return equSys;
    };

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
    };

    SatValueMap getSatElevData() {
        return satElevData;
    }

    Vector3d getXYZ() {
        return xyz;
    }

    ~SPPIFCode() = default;

    // 继承类需要访问这个成员
protected:
    double cutOffElev;

    bool isRover;
    double sigIFCode;


    EquSys equSys;
    Result result;

    Vector3d xyz;
    Vector3d dxyz;
    double rcvrClk; // Receiver clock bias in meters

    std::map<SatID, PVT> satPVTTransTime{};
    std::map<SatID, PVT> satPVTRecTime{};

    SatValueMap satElevData;
    SatValueMap satAzimData;
    SatValueMap satTropData;

    SolverLSQ solver;

    std::map<SatID, Ephemeris *> ephMap;

    std::map<char, std::pair<string, string> > ifCodeTypes{};
};
