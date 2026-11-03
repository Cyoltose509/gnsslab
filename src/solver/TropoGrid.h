#pragma once

#include "product/GPT3Reader.h"
#include "Troposphere.h"
#include "product/VMF3Reader.h"

#include <string>
#include <vector>

class TropoGrid {
public:
    struct Site {
        bool valid = false;
        Troposphere::VMF3::Coef coef; // ah/aw/bh/bw/ch/cw
        double pHpa = 0.0; // 格网气压(hPa
        bool hasVMF3Zhd = false; // 格网含 zhd/zwd 且格点高可用
        double zhdGrid = 0.0; // 格点地形高处的天顶干延迟(m)
        double zwdGrid = 0.0; // 格点地形高处的天顶湿延迟(m)
        double hGridM = 0.0; // 该格点的椭球高(m)
    };

    bool loadVMF3(const std::vector<std::string> &paths) { return vmf3.loadFiles(paths); }

    bool loadGPT3(const std::string &path) { return gpt3.load(path); }

    void clear() {
        vmf3.clear();
        gpt3.clear();
    }

    [[nodiscard]] bool hasVMF3() const { return !vmf3.empty(); }
    [[nodiscard]] bool hasGPT3() const { return !gpt3.empty(); }

    [[nodiscard]] bool readyFor(const Troposphere::Map m) const {
        if (m == Troposphere::Map::VMF3) return !vmf3.empty();
        if (m == Troposphere::Map::GPT3) return !gpt3.empty();
        return true;
    }

    /// 解析出该历元该测站的系数
    bool site(const Troposphere::Map m, const double mjd, const double latRad, const double lonRad, const double hgtM,
              const double doy, Site &out) const {
        out = Site{};
        if (m == Troposphere::Map::VMF3) {
            double ah = 0.0, aw = 0.0;
            if (!vmf3.coefficients(mjd, latRad, lonRad, ah, aw)) return false;
            out.coef = Troposphere::VMF3::makeCoef(ah, aw, latRad, lonRad, doy);
            out.valid = true;

            if (double hg = 0.0, zg = 0.0, wg = 0.0; gpt3.gridHeight(latRad, lonRad, hg) &&
                                                     vmf3.zenithDelays(mjd, latRad, lonRad, zg, wg)) {
                out.hasVMF3Zhd = true;
                out.zhdGrid = zg;
                out.zwdGrid = wg;
                out.hGridM = hg;
            }
            return true;
        }
        if (m == Troposphere::Map::GPT3) {
            GPT3Values g;
            if (!gpt3.at(mjd, latRad, lonRad, hgtM, g)) return false;
            out.coef = Troposphere::VMF3::makeCoef(g.ah, g.aw, latRad, lonRad, doy);
            out.pHpa = g.pHpa;
            out.valid = true;
            return true;
        }
        return false;
    }

private:
    VMF3Reader vmf3;
    GPT3Reader gpt3;
};

/// 干/湿映射统一入口：
inline double tropoMappingWith(const TropoGrid::Site &s, const Troposphere::Map m, const double elevRad, const double latRad,
                               const double lonRad, const double hgtM, const double doy, const double mjd,
                               double *mapfw) {
    if (s.valid) {
        if (mapfw) *mapfw = Troposphere::VMF3::mapW(s.coef, elevRad);
        return Troposphere::VMF3::mapH(s.coef, elevRad, hgtM);
    }
    return Troposphere::mapping(m, elevRad, latRad, lonRad, hgtM, doy, mjd, mapfw);
}

/// 天顶干延迟(m)
inline double tropoZhdWith(const TropoGrid::Site &s, const double latRad, const double hgtM) {
    return Troposphere::Saastamoinen::zhd(latRad, hgtM, s.valid ? s.pHpa : 0.0);
}
