#pragma once
#include "GnssStruct.h"
#include "FreqCombo.h"
#include "Geodesy.h"
#include <map>
#include <string>
#include <vector>

/// ATX 相位中心变化(PCV)网格
struct PcvGrid {
    double zen1 = 0.0, zen2 = 90.0, dzen = 5.0, dazi = 0.0;
    std::vector<double> noazi; // 与方位无关，按 zen1..zen2 步长 dzen
    std::vector<std::vector<double> > azi; // azi[方位块][天顶距索引]

    [[nodiscard]] bool valid() const { return !noazi.empty(); }

    [[nodiscard]] double value(double zenDeg, double azDeg) const;
};

class AntxReader {
public:
    bool read(const std::string &path);

    bool empty() const { return satPCO.empty() && rcvPCO.empty(); }

    /// 卫星某频点 PCO
    Eigen::Vector3d getSatPCO(const SatID &sat, const std::string &band) const;


    Eigen::Vector3d getRcvPCOENU(const std::string &rcvType) const;

    void applySatPCO(const SatID &sat, Eigen::Vector3d &pTx, const CommonTime &epoch,
                     const FreqCombo &def) const;

    /// 接收机 PCV(m)：sys 决定 IF 双频点；zenDeg 天顶距(度)，azDeg 方位角(度)
    double rcvPcvIF(const std::string &rcvType, char sys, const FreqCombo &def,
                    double zenDeg, double azDeg) const;

    /// 卫星 PCV(m)：自变量为星下点角(度)
    double satPcvIF(const SatID &sat, const FreqCombo &def, double nadirDeg) const;

private:
    std::map<SatID, std::map<std::string, Eigen::Vector3d> > satPCO;
    std::map<std::string, Eigen::Vector3d> rcvPCO;
    std::map<SatID, std::map<std::string, PcvGrid> > satPCV;
    std::map<std::string, std::map<std::string, PcvGrid> > rcvPCV;
};
