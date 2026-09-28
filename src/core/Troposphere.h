#pragma once

#include "Const.h"

class Troposphere {
public:
    static constexpr double MAP_EPS = 3.0e-3;
    static constexpr double COEF[9][5] = {
        {1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3},
        {2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3},
        {62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3},
        {0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5},
        {0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5},
        {0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5},
        {5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4},
        {1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3},
        {4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2}
    };
    static constexpr double AHT[3] = {2.53E-5, 5.49E-3, 1.14E-3}; // 高程改正系数
    static double tropoHopfieldDry(const double H) {
        constexpr double T0 = 288.16, P0 = 1013.25, H0 = 0.0;
        const double T = T0 - 0.0065 * (H - H0);
        if (T < 200.0) return 0.0;
        const double P = P0 * pow(1 - 0.0000226 * (H - H0), 5.225);
        constexpr double hd = 40136.0 + 148.72 * (T0 - 273.16);
        const double Kd = 155.2e-7 * (P / T) * (hd - H);
        return Kd > 0.0 ? Kd : 0.0;
    }

    static double tropoHopfieldWet(const double H) {
        constexpr double T0 = 288.16, RH0 = 0.5, H0 = 0.0;
        const double T = T0 - 0.0065 * (H - H0);
        if (T < 200.0) return 0.0;
        const double RH = RH0 * exp(-0.0006396 * (H - H0));
        const double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
        constexpr double hw = 11000.0;
        const double Kw = 155.2e-7 * (4810.0 * e / (T * T)) * (hw - H);
        return Kw > 0.0 ? Kw : 0.0;
    }

    // 干/湿映射函数
    static double tropoMapDry(const double E) { return 1.0 / sin(sqrt(E * E + 1.90386e-3)); }
    static double tropoMapWet(const double E) { return 1.0 / sin(sqrt(E * E + 6.85389e-4)); }

    // 总天顶对流层延迟（Hopfield 模型）
    static double tropoHopfield(const double H, const double E) {
        const double tropo = tropoHopfieldDry(H) * tropoMapDry(E)
                             + tropoHopfieldWet(H) * tropoMapWet(E);
        if (!isfinite(tropo) || tropo < 0.0 || tropo > 100.0)
            return 0.0;
        return tropo;
    }

    // Saastamoinen 天顶延迟
    static double tropoSaastamoinen(const double latRad, const double hgtM, const double humi) {
        if (hgtM < -100.0 || hgtM > 1.0e4) return 0.0;
        constexpr double temp0 = 15.0; // 海平面温度(℃)
        const double hgt = hgtM < 0.0 ? 0.0 : hgtM;
        const double pres = 1013.25 * pow(1.0 - 2.2557e-5 * hgt, 5.2568);
        const double temp = temp0 - 6.5e-3 * hgt + 273.16; // K
        const double e = 6.108 * humi * exp((17.15 * temp - 4684.0) / (temp - 38.45)); // 水汽压(hPa)
        const double trph = 0.0022768 * pres /
                            (1.0 - 0.00266 * cos(2.0 * latRad) - 0.00028 * hgt / 1.0e3);
        const double trpw = 0.002277 * (1255.0 / temp + 0.05) * e;
        return trph + trpw;
    }

    // Saastamoinen 天顶干延迟 ZHD(m)
    static double tropoSaastamoinenZHD(const double latRad, const double hgtM) {
        return tropoSaastamoinen(latRad, hgtM, 0.0);
    }

    static double tropoNMF(const double elRad, const double latDeg, const double hgtM,
                           const double doy, double *mapfw) {
        if (elRad <= 0.0) {
            if (mapfw) *mapfw = 0.0;
            return 0.0;
        }
        auto interpc = [](const double c[5], const double lat) -> double {
            const int i = static_cast<int>(lat / 15.0);
            if (i < 1) return c[0];
            if (i > 4) return c[4];
            return c[i - 1] * (1.0 - lat / 15.0 + i) + c[i] * (lat / 15.0 - i);
        };
        auto mapf = [](const double el, const double a, const double b, const double c) -> double {
            const double sinel = sin(el);
            return (1.0 + a / (1.0 + b / (1.0 + c))) / (sinel + a / (sinel + b / (sinel + c)));
        };
        const double y = (doy - 28.0) / 365.25 + (latDeg < 0.0 ? 0.5 : 0.0);
        const double cosy = cos(2.0 * PI * y);
        const double lat = fabs(latDeg);
        double ah[3] = {0.0}, aw[3] = {0.0};
        for (int i = 0; i < 3; i++) {
            ah[i] = interpc(COEF[i], lat) - interpc(COEF[i + 3], lat) * cosy;
            aw[i] = interpc(COEF[i + 6], lat);
        }
        const double dm = (1.0 / sin(elRad) - mapf(elRad, AHT[0], AHT[1], AHT[2])) * hgtM / 1.0E3;
        if (mapfw) *mapfw = mapf(elRad, aw[0], aw[1], aw[2]);
        return mapf(elRad, ah[0], ah[1], ah[2]) + dm;
    }
};
