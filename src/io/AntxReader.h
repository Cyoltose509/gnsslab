#pragma once
#include "GnssStruct.h"
#include "FreqCombo.h"
#include "Geodesy.h"
#include <map>
#include <string>
#include <vector>

class AntxReader {
public:
    bool read(const std::string &path);

    bool empty() const { return satPCO.empty() && rcvPCO.empty(); }

    /// 卫星某频点 PCO
    Eigen::Vector3d getSatPCO(const SatID &sat, const std::string &band) const;


    Eigen::Vector3d getRcvPCOENU(const std::string &rcvType) const;

    void applySatPCO(const SatID &sat, Eigen::Vector3d &pTx, const Eigen::Vector3d &vTx,
                    const CommonTime &epoch, const FreqCombo &def) const;

private:
    std::map<SatID, std::map<std::string, Eigen::Vector3d>> satPCO;
    std::map<std::string, Eigen::Vector3d> rcvPCO;
};
