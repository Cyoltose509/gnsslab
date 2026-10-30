#pragma once

#include "Const.h"
#include <string>


enum class TropoMap {
    Hopfield,
    Nmf,
    Gmf
};

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

    // 天顶湿延迟先验(m)
    static double tropoZwdPrior(const double latDeg, const double hgtM, const double doy) {
        if (hgtM < -100.0 || hgtM > 1.0e4) return 0.0;
        const double hgt = hgtM < 0.0 ? 0.0 : hgtM;
        const double alat = std::fabs(latDeg);
        const double peak = latDeg >= 0.0 ? 200.0 : 17.0; // 北半球 7 月中旬最热，南半球反相
        const double tC = 30.0 - 0.30 * alat + 0.27 * alat * cos(2.0 * PI * (doy - peak) / 365.25);
        const double temp = tC - 6.5e-3 * hgt + 273.16; // K
        const double e = 6.108 * 0.8 * exp((17.15 * temp - 4684.0) / (temp - 38.45)); // 水汽压(hPa)
        return 0.002277 * (1255.0 / temp + 0.05) * e;
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

    // GMF (Global Mapping Function, Boehm et al. 2006)
    static void tropoGMF(const double latRad, const double lonRad, const double hgtM,
                         const double mjd, const double elevRad, double *mh, double *mw) {
        constexpr double PI2 = 2.0 * PI;
        const double DOY = mjd - 44239.0 + 1.0 - 28.0; // = mjd - 44266，参考日 1980-01-28
        const double T = std::sin(latRad);
        constexpr int NMAX = 9;
        double dfac[21] = {};
        dfac[1] = 1.0;
        for (int i = 1; i <= 2 * NMAX + 1; ++i) dfac[i + 1] = dfac[i] * i;
        double P[10][10] = {{}};
        for (int n = 0; n <= NMAX; ++n) {
            for (int m = 0; m <= n; ++m) {
                const int IR = (n - m) / 2;
                double sum1 = 0.0;
                for (int k = 0; k <= IR; ++k) {
                    const double num = dfac[2 * n - 2 * k + 1];
                    const double den = dfac[k + 1] * dfac[n - k + 1] * dfac[n - m - 2 * k + 1];
                    sum1 += (k % 2 == 0 ? 1.0 : -1.0) * num / den * std::pow(T, n - m - 2 * k);
                }
                P[n][m] = std::pow(0.5, n) * std::sqrt(std::pow(1.0 - T * T, m)) * sum1;
            }
        }
        static constexpr double AH_MEAN[55] = {
            1.2517e+02, 8.503e-01, 6.936e-02, -6.760e+00, 1.771e-01, 1.130e-02, 5.963e-01, 1.808e-02, 2.801e-03, -1.414e-03,
            -1.212e+00, 9.300e-02, 3.683e-03, 1.095e-03, 4.671e-05, 3.959e-01, -3.867e-02, 5.413e-03, -5.289e-04, 3.229e-04,
            2.067e-05, 3.000e-01, 2.031e-02, 5.900e-03, 4.573e-04, -7.619e-05, 2.327e-06, 3.845e-06, 1.182e-01, 1.158e-02,
            5.445e-03, 6.219e-05, 4.204e-06, -2.093e-06, 1.540e-07, -4.280e-08, -4.751e-01, -3.490e-02, 1.758e-03, 4.019e-04,
            -2.799e-06, -1.287e-06, 5.468e-07, 7.580e-08, -6.300e-09, -1.160e-01, 8.301e-03, 8.771e-04, 9.955e-05, -1.718e-06,
            -2.012e-06, 1.170e-08, 1.790e-08, -1.300e-09, 1.000e-10
        };
        static constexpr double BH_MEAN[55] = {
            0.0, 0.0, 3.249e-02, 0.0, 3.324e-02, 1.850e-02, 0.0, -1.115e-01, 2.519e-02, 4.923e-03,
            0.0, 2.737e-02, 1.595e-02, -7.332e-04, 1.933e-04, 0.0, -4.796e-02, 6.381e-03, -1.599e-04, -3.685e-04,
            1.815e-05, 0.0, 7.033e-02, 2.426e-03, -1.111e-03, -1.357e-04, -7.828e-06, 2.547e-06, 0.0, 5.779e-03,
            3.133e-03, -5.312e-04, -2.028e-05, 2.323e-07, -9.100e-08, -1.650e-08, 0.0, 3.688e-02, -8.638e-04, -8.514e-05,
            -2.828e-05, 5.403e-07, 4.390e-07, 1.350e-08, 1.800e-09, 0.0, -2.736e-02, -2.977e-04, 8.113e-05, 2.329e-07,
            8.451e-07, 4.490e-08, -8.100e-09, -1.500e-09, 2.000e-10
        };
        static constexpr double AH_AMP[55] = {
            -2.738e-01, -2.837e+00, 1.298e-02, -3.588e-01, 2.413e-02, 3.427e-02, -7.624e-01, 7.272e-02, 2.160e-02, -3.385e-03,
            4.424e-01, 3.722e-02, 2.195e-02, -1.503e-03, 2.426e-04, 3.013e-01, 5.762e-02, 1.019e-02, -4.476e-04, 6.790e-05,
            3.227e-05, 3.123e-01, -3.535e-02, 4.840e-03, 3.025e-06, -4.363e-05, 2.854e-07, -1.286e-06, -6.725e-01, -3.730e-02,
            8.964e-04, 1.399e-04, -3.990e-06, 7.431e-06, -2.796e-07, -1.601e-07, 4.068e-02, -1.352e-02, 7.282e-04, 9.594e-05,
            2.070e-06, -9.620e-08, -2.742e-07, -6.370e-08, -6.300e-09, 8.625e-02, -5.971e-03, 4.705e-04, 2.335e-05, 4.226e-06,
            2.475e-07, -8.850e-08, -3.600e-08, -2.900e-09, 0.0
        };
        static constexpr double BH_AMP[55] = {
            0.0, 0.0, -1.136e-01, 0.0, -1.868e-01, -1.399e-02, 0.0, -1.043e-01, 1.175e-02, -2.240e-03,
            0.0, -3.222e-02, 1.333e-02, -2.647e-03, -2.316e-05, 0.0, 5.339e-02, 1.107e-02, -3.116e-03, -1.079e-04,
            -1.299e-05, 0.0, 4.861e-03, 8.891e-03, -6.448e-04, -1.279e-05, 6.358e-06, -1.417e-07, 0.0, 3.041e-02,
            1.150e-03, -8.743e-04, -2.781e-05, 6.367e-07, -1.140e-08, -4.200e-08, 0.0, -2.982e-02, -3.000e-03, 1.394e-05,
            -3.290e-05, -1.705e-07, 7.440e-08, 2.720e-08, -6.600e-09, 0.0, 1.236e-02, -9.981e-04, -3.792e-05, -1.355e-05,
            1.162e-06, -1.789e-07, 1.470e-08, -2.400e-09, -4.000e-10
        };
        static constexpr double AW_MEAN[55] = {
            5.640e+01, 1.555e+00, -1.011e+00, -3.975e+00, 3.171e-02, 1.065e-01, 6.175e-01, 1.376e-01, 4.229e-02, 3.028e-03,
            1.688e+00, -1.692e-01, 5.478e-02, 2.473e-02, 6.059e-04, 2.278e+00, 6.614e-03, -3.505e-04, -6.697e-03, 8.402e-04,
            7.033e-04, -3.236e+00, 2.184e-01, -4.611e-02, -1.613e-02, -1.604e-03, 5.420e-05, 7.922e-05, -2.711e-01, -4.406e-01,
            -3.376e-02, -2.801e-03, -4.090e-04, -2.056e-05, 6.894e-06, 2.317e-06, 1.941e+00, -2.562e-01, 1.598e-02, 5.449e-03,
            3.544e-04, 1.148e-05, 7.503e-06, -5.667e-07, -3.660e-08, 8.683e-01, -5.931e-02, -1.864e-03, -1.277e-04, 2.029e-04,
            1.269e-05, 1.629e-06, 9.660e-08, -1.015e-07, -5.000e-10
        };
        static constexpr double BW_MEAN[55] = {
            0.0, 0.0, 2.592e-01, 0.0, 2.974e-02, -5.471e-01, 0.0, -5.926e-01, -1.030e-01, -1.567e-02,
            0.0, 1.710e-01, 9.025e-02, 2.689e-02, 2.243e-03, 0.0, 3.439e-01, 2.402e-02, 5.410e-03, 1.601e-03,
            9.669e-05, 0.0, 9.502e-02, -3.063e-02, -1.055e-03, -1.067e-04, -1.130e-04, 2.124e-05, 0.0, -3.129e-01,
            8.463e-03, 2.253e-04, 7.413e-05, -9.376e-05, -1.606e-06, 2.060e-06, 0.0, 2.739e-01, 1.167e-03, -2.246e-05,
            -1.287e-04, -2.438e-05, -7.561e-07, 1.158e-06, 4.950e-08, 0.0, -1.344e-01, 5.342e-03, 3.775e-04, -6.756e-05,
            -1.686e-06, -1.184e-06, 2.768e-07, 2.730e-08, 5.700e-09
        };
        static constexpr double AW_AMP[55] = {
            1.023e-01, -2.695e+00, 3.417e-01, -1.405e-01, 3.175e-01, 2.116e-01, 3.536e+00, -1.505e-01, -1.660e-02, 2.967e-02,
            3.819e-01, -1.695e-01, -7.444e-02, 7.409e-03, -6.262e-03, -1.836e+00, -1.759e-02, -6.256e-02, -2.371e-03, 7.947e-04,
            1.501e-04, -8.603e-01, -1.360e-01, -3.629e-02, -3.706e-03, -2.976e-04, 1.857e-05, 3.021e-05, 2.248e+00, -1.178e-01,
            1.255e-02, 1.134e-03, -2.161e-04, -5.817e-06, 8.836e-07, -1.769e-07, 7.313e-01, -1.188e-01, 1.145e-02, 1.011e-03,
            1.083e-04, 2.570e-06, -2.140e-06, -5.710e-08, 2.000e-08, -1.632e+00, -6.948e-03, -3.893e-03, 8.592e-04, 7.577e-05,
            4.539e-06, -3.852e-07, -2.213e-07, -1.370e-08, 5.800e-09
        };
        static constexpr double BW_AMP[55] = {
            0.0, 0.0, -8.865e-02, 0.0, -4.309e-01, 6.340e-02, 0.0, 1.162e-01, 6.176e-02, -4.234e-03,
            0.0, 2.530e-01, 4.017e-02, -6.204e-03, 4.977e-03, 0.0, -1.737e-01, -5.638e-03, 1.488e-04, 4.857e-04,
            -1.809e-04, 0.0, -1.514e-01, -1.685e-02, 5.333e-03, -7.611e-05, 2.394e-05, 8.195e-06, 0.0, 9.326e-02,
            -1.275e-02, -3.071e-04, 5.374e-05, -3.391e-05, -7.436e-06, 6.747e-07, 0.0, -8.637e-02, -3.807e-03, -6.833e-04,
            -3.861e-05, -2.268e-05, 1.454e-06, 3.860e-07, -1.068e-07, 0.0, -2.658e-02, -1.947e-03, 7.131e-04, -3.506e-05,
            1.885e-07, 5.792e-07, 3.990e-08, 2.000e-08, -5.700e-09
        };
        double AP[55] = {}, BP[55] = {};
        int idx = 0;
        for (int n = 0; n <= NMAX; ++n)
            for (int m = 0; m <= n; ++m) {
                AP[idx] = P[n][m] * std::cos(static_cast<double>(m) * lonRad);
                BP[idx] = P[n][m] * std::sin(static_cast<double>(m) * lonRad);
                ++idx;
            }
        double AHM = 0, AHA = 0, AWM = 0, AWA = 0;
        for (int i = 0; i < 55; ++i) {
            AHM += (AH_MEAN[i] * AP[i] + BH_MEAN[i] * BP[i]) * 1e-5;
            AHA += (AH_AMP[i]  * AP[i] + BH_AMP[i]  * BP[i]) * 1e-5;
            AWM += (AW_MEAN[i] * AP[i] + BW_MEAN[i] * BP[i]) * 1e-5;
            AWA += (AW_AMP[i]  * AP[i] + BW_AMP[i]  * BP[i]) * 1e-5;
        }
        const double cdt = std::cos(DOY / 365.25 * PI2);
        const double AH = AHM + AHA * cdt;
        const double AW = AWM + AWA * cdt;

        constexpr double BH = 0.0029;
        constexpr double C0H = 0.062;
        double PHH, C11H, C10H;
        if (latRad < 0.0) { PHH = PI; C11H = 0.007; C10H = 0.002; }
        else              { PHH = 0.0; C11H = 0.005; C10H = 0.001; }
        const double CH = C0H + ((std::cos(DOY / 365.25 * PI2 + PHH) + 1.0) * C11H / 2.0 + C10H) *
                                 (1.0 - std::cos(latRad));

        const double SINE = std::sin(elevRad);
        double beta = BH / (SINE + CH);
        double gamma = AH / (SINE + beta);
        double topcon = 1.0 + AH / (1.0 + BH / (1.0 + CH));
        double GMFH = topcon / (SINE + gamma);
        constexpr double A_HT = 2.53e-5, B_HT = 5.49e-3, C_HT = 1.14e-3;
        const double hsKm = hgtM / 1000.0;
        beta = B_HT / (SINE + C_HT);
        gamma = A_HT / (SINE + beta);
        topcon = 1.0 + A_HT / (1.0 + B_HT / (1.0 + C_HT));
        GMFH += (1.0 / SINE - topcon / (SINE + gamma)) * hsKm;
        constexpr double BW = 0.00146, CW = 0.04391;
        beta = BW / (SINE + CW);
        gamma = AW / (SINE + beta);
        topcon = 1.0 + AW / (1.0 + BW / (1.0 + CW));
        const double GMFW = topcon / (SINE + gamma);
        if (mh) *mh = GMFH;
        if (mw) *mw = GMFW;
    }

    static const char *tropoMapName(const TropoMap m) {
        switch (m) {
            case TropoMap::Nmf: return "nmf";
            case TropoMap::Gmf: return "gmf";
            case TropoMap::Hopfield: break;
        }
        return "hopfield";
    }

    static TropoMap tropoMapFromName(const std::string &s) {
        if (s == "nmf") return TropoMap::Nmf;
        if (s == "gmf") return TropoMap::Gmf;
        return TropoMap::Hopfield; // "hopfield"/"simple"/未识别一律回落简单映射
    }

    /// 干/湿映射函数统一入口：返回干映射，湿映射经 mapfw 输出。
    /// 三个解算器都只经这里取映射，避免「PPP 用 GMF、运动学重锚定却写死 NMF」这类分叉。
    static double tropoMapping(const TropoMap m, const double elevRad, const double latRad,
                               const double lonRad, const double hgtM, const double doy,
                               const double mjd, double *mapfw = nullptr) {
        if (m == TropoMap::Nmf) return tropoNMF(elevRad, latRad * RAD_TO_DEG, hgtM, doy, mapfw);
        if (m == TropoMap::Gmf) {
            double mh = 0.0, mw = 0.0;
            tropoGMF(latRad, lonRad, hgtM, mjd, elevRad, &mh, &mw);
            if (mapfw) *mapfw = mw;
            return mh;
        }
        if (mapfw) *mapfw = tropoMapWet(elevRad);
        return tropoMapDry(elevRad);
    }
};
