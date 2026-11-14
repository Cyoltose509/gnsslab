#pragma once


class Weight {
public:
    double a = 0.004;
    double b = 0.003;
    double minSinElev = 1e-3;

    Weight() = default;
    Weight(const double a_, const double b_, const double minSinElev_ = 1e-3)
        : a(a_), b(b_), minSinElev(minSinElev_) {}

    [[nodiscard]] double variance(const double elev) const {
        const double s = (std::max)(std::sin(elev), minSinElev);
        return a * a + b * b / (s * s);
    }


    [[nodiscard]] double weight(const double elev) const {
        return 1.0 / variance(elev);
    }
};
