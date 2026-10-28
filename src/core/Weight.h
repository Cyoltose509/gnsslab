#pragma once

#include <cmath>

class Weight {
public:
    double a = 0.004;
    double b = 0.003;
    double minSinElev = 1e-3;

    Weight() = default;
    Weight(double a_, double b_, double minSinElev_ = 1e-3)
        : a(a_), b(b_), minSinElev(minSinElev_) {}

    double variance(double elev) const {
        const double s = (std::max)(std::sin(elev), minSinElev);
        return a * a + b * b / (s * s);
    }


    double weight(double elev) const {
        return 1.0 / variance(elev);
    }
};
