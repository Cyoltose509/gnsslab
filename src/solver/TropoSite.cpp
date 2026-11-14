#include "TropoSite.h"

#include "core/Const.h"
#include "core/geom/CoordConvert.h"
#include "TimeStruct.h"


void TropoSite::reset() {
    valid = false;
    latRad = lonRad = 0.0;
    hgt = 0.0;
    doy = mjd = 0.0;
    zhd = 0.0;
    zwd = 0.0;
    gradN = gradE = 0.0;
    grid = {};
    gmf = {};
}

void TropoSite::update(const TropoGrid &table, const Troposphere::Map map, const Eigen::Vector3d &xyzEcef,
                       const CommonTime &epoch, const GeodeticFrame &frame) {
    reset();
    if (!(xyzEcef.norm() > 1000.0)) return;

    const auto blh = Coord::XYZToBLH(xyzEcef, frame);
    latRad = blh.B();
    lonRad = blh.L();
    hgt = blh.H();
    doy = static_cast<double>(epoch.toYDS().doy);
    const MJD m = epoch.toMJD();
    mjd = static_cast<double>(m.mjd);
    valid = true;

    table.site(map, mjd, latRad, lonRad, hgt, doy, grid);
    // 格网产品没配（或没覆盖到）时映射走 GMF 球谐：系数只与站坐标/时间有关，
    // 逐卫星重算 55 项 Legendre 纯属浪费，这里按历元算一次留给 mapping() 用。
    if (!grid.valid && (map == Troposphere::Map::GMF || map == Troposphere::Map::VMF3 ||
                        map == Troposphere::Map::GPT3))
        gmf = Troposphere::GMF::coef(latRad, lonRad, mjd);

    if (grid.valid && grid.hasVMF3Zhd) {
        const double p = Troposphere::Saastamoinen::stdPressure(hgt);
        const double p0 = Troposphere::Saastamoinen::zhdToPressure(grid.zhdGrid, latRad, grid.hGridM);
        const double ratio = p0 > 0.0 ? p / p0 : 1.0;
        zhd = grid.zhdGrid * ratio;
        zwd = grid.zwdGrid * ratio;
    } else {
        if (grid.valid && grid.pHpa > 0.0)
            zhd = Troposphere::Saastamoinen::zhd(latRad, hgt, grid.pHpa);
        else if (map == Troposphere::Map::Hopfield)
            zhd = Troposphere::Hopfield::zhd(hgt);
        else
            zhd = Troposphere::Saastamoinen::zhd(latRad, hgt); // 标准大气
        zwd = zwdPrior(map);
    }
}

double TropoSite::latDeg() const { return latRad * RAD_TO_DEG; }

double TropoSite::lonDeg() const { return lonRad * RAD_TO_DEG; }

double TropoSite::zwdPrior(const Troposphere::Map map) const {
    if (map == Troposphere::Map::Hopfield) return Troposphere::Hopfield::zwd(hgt);
    return Troposphere::zwdPrior(latDeg(), hgt, doy);
}

double TropoSite::mapping(const Troposphere::Map map, const double elevRad, double *mW) const {
    if (!grid.valid && gmf.ready &&
        (map == Troposphere::Map::GMF || map == Troposphere::Map::VMF3 ||
         map == Troposphere::Map::GPT3)) {
        double mH = 0.0;
        Troposphere::GMF::mapping(gmf, hgt, elevRad, &mH, mW);
        return mH;
    }
    return tropoMappingWith(grid, map, elevRad, latRad, lonRad, hgt, doy, mjd, mW);
}

double TropoSite::dryDelay(const Troposphere::Map map, const double elevRad, double *mW,
                           const double azimRad, const bool withGradient) const {
    if (!valid) return 0.0;
    const double mH = mapping(map, elevRad, mW);
    double d = zhd * mH;
    if (withGradient && (gradN != 0.0 || gradE != 0.0)) {
        if (const double sinEl = std::sin(elevRad); sinEl > 1e-3)
            d += (gradN * std::cos(azimRad) + gradE * std::sin(azimRad)) / sinEl;
    }
    return d;
}
