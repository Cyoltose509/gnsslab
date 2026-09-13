#pragma once
#include "GnssStruct.h"
#include "FreqCombo.h"
#include "Geodesy.h"
#include <map>
#include <string>
#include <vector>
#include <algorithm>

/// ATX 相位中心变化(PCV)网格：NOAZI 行 + 可选按方位角分块的行，单位 m
struct PcvGrid {
    double zen1 = 0.0, zen2 = 90.0, dzen = 5.0, dazi = 0.0;
    std::vector<double> noazi;                     // 与方位无关，按 zen1..zen2 步长 dzen
    std::vector<std::vector<double>> azi;          // azi[方位块][天顶距索引]

     [[nodiscard]]bool valid() const { return !noazi.empty(); }

    [[nodiscard]] double value(const double zenDeg, const double azDeg) const {
        if (noazi.empty()) return 0.0;
        const int n = static_cast<int>(noazi.size());
        const double dz = dzen > 1e-9 ? dzen : 1.0;
        double zf = (zenDeg - zen1) / dz;
        if (zf < 0.0) zf = 0.0;
        if (zf > n - 1) zf = static_cast<double>(n - 1);
        const int i0 = static_cast<int>(std::floor(zf));
        const int i1 = std::min(i0 + 1, n - 1);
        const double t = zf - i0;
        if (azi.empty() || dazi <= 1e-9) return noazi[i0] + t * (noazi[i1] - noazi[i0]);

        const int m = static_cast<int>(azi.size());
        double af = std::fmod(azDeg / dazi, static_cast<double>(m));
        if (af < 0.0) af += m;
        const int j0 = static_cast<int>(std::floor(af));
        const int j1 = (j0 + 1) % m;
        const double u = af - j0;
        const auto &r0 = azi[j0], &r1 = azi[j1];
        const double v0 = r0[i0] + t * (r0[i1] - r0[i0]);
        const double v1 = r1[i0] + t * (r1[i1] - r1[i0]);
        return v0 + u * (v1 - v0);
    }
};

class AntxReader {
public:
    bool read(const std::string &path);

    bool empty() const { return satPCO.empty() && rcvPCO.empty(); }

    /// 卫星某频点 PCO
    Vector3d getSatPCO(const SatID &sat, const std::string &band) const;


    Vector3d getRcvPCOENU(const std::string &rcvType) const;

    void applySatPCO(const SatID &sat, Vector3d &pTx, const CommonTime &epoch,
                    const FreqCombo &def) const;

    /// 接收机 PCV(m)：sys 决定 IF 双频点；zenDeg 天顶距(度)，azDeg 方位角(度)
    double rcvPcvIF(const std::string &rcvType, char sys, const FreqCombo &def,
                    double zenDeg, double azDeg) const;

    /// 卫星 PCV(m)：自变量为星下点角(度)
    double satPcvIF(const SatID &sat, const FreqCombo &def, double nadirDeg) const;

private:
    std::map<SatID, std::map<std::string, Vector3d>> satPCO;
    std::map<std::string, Vector3d> rcvPCO;
    std::map<SatID, std::map<std::string, PcvGrid>> satPCV;
    std::map<std::string, std::map<std::string, PcvGrid>> rcvPCV;
};
