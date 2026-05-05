

#pragma once

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "RinexNavStore.h"
#include <Eigen/Eigen>

class SPPIFCode {
public:
    SPPIFCode()
    : cutOffElev(10), isRover(true), sigIFCode(1.0)
    {}

    void setStationAsBase()
    {
        isRover = false;
    }

    void setRinexNavStore(RinexNavStore* pStore)
    {
        pEphStore = pStore;
    };

    void setIFCodeTypes(const std::map<char, std::pair<string, string>>& ifTypes)
    {
        ifCodeTypes = ifTypes;
    };

    void solve(ObsData &obsData);

    std::map<SatID,PVT> computeSatPos(ObsData &obsData);
    PVT computeAtTransmitTime(const CommonTime& tr,
                              const double& pr,
                              const SatID& sat);

    void computeElevAzim(Vector3d& xyz,
                          std::map<SatID,PVT> & satXvtTransTime,
                          SatValueMap& tempElevData,
                          SatValueMap& tempAzimData);

    static void convertObsType(ObsData &obsData);
    void computeIF(ObsData &obsData);
    std::map<SatID,PVT> earthRotation(Vector3d& xyz,
                                      std::map<SatID,PVT> & satXvtTransTime);
    EquSys linearize(Vector3d& xyz,
                     std::map<SatID,PVT>& satXvtRecTime,
                     SatValueMap& satElevData,
                     ObsData& obsData);

    EquSys getEquSys()
    {
        return equSys;
    };

    [[nodiscard]] SatID getDatumSat() const {
        double maxElev(0.0);
        SatID datumSat;
        for(auto se: satElevData)
        {
            if(se.second>maxElev)
            {
                maxElev = se.second;
                datumSat = se.first;
            }
        }
        return datumSat;
    };

    SatValueMap getSatElevData()
    {
        return satElevData;
    }

    Vector3d getXYZ()
    {
        return xyz;
    }

    Result getResult();

    ~SPPIFCode()= default;

    // 继承类需要访问这个成员
protected:
    double cutOffElev;

    bool isRover;
    double sigIFCode;


    EquSys equSys;
    Result result;

    Vector3d xyz;
    Vector3d dxyz;

    std::map<SatID,PVT> satXvtTransTime{};
    std::map<SatID,PVT> satXvtRecTime{};

    SatValueMap  satElevData;
    SatValueMap  satAzimData;
    SatValueMap  satTropData;

    SolverLSQ  solverLsq;

    RinexNavStore* pEphStore;

    std::map<char, std::pair<string, string>> ifCodeTypes{};

    SatID datumSat;

};


