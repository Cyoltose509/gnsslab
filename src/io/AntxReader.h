#pragma once
#include "GnssStruct.h"
#include "FreqCombo.h"
#include "Geodesy.h"
#include <map>
#include <string>
#include <vector>


struct PcvGrid {
    double zen1 = 0.0, zen2 = 90.0, dzen = 5.0, dazi = 0.0;
    std::vector<double> noazi;
    std::vector<std::vector<double> > azi;

    [[nodiscard]] bool valid() const { return !noazi.empty(); }

    [[nodiscard]] double value(double zenDeg, double azDeg) const;
};

class AntxReader {
public:
    bool read(const std::string &path);

    bool empty() const { return satPCO.empty() && rcvPCO.empty(); }


    Eigen::Vector3d getSatPCO(const SatID &sat, const std::string &band) const;


    Eigen::Vector3d getRcvPCOENU(const std::string &rcvType) const;

    void applySatPCO(const SatID &sat, Eigen::Vector3d &pTx, const CommonTime &epoch,
                     const FreqCombo &def) const;


    double rcvPcvIF(const std::string &rcvType, char sys, const FreqCombo &def,
                    double zenDeg, double azDeg) const;


    double satPcvIF(const SatID &sat, const FreqCombo &def, double nadirDeg) const;

private:
    std::map<SatID, std::map<std::string, Eigen::Vector3d> > satPCO;
    std::map<std::string, Eigen::Vector3d> rcvPCO;
    std::map<SatID, std::map<std::string, PcvGrid> > satPCV;
    std::map<std::string, std::map<std::string, PcvGrid> > rcvPCV;
};
