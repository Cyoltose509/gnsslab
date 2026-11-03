#pragma once

struct EpochDecimator {
    explicit EpochDecimator(const double iv) : interval(iv > 0.0 ? iv : 0.0) {}

    [[nodiscard]] bool accept(const double sow) const {
        if (interval <= 0.0) return true;
        return std::fmod(sow + kDttol, interval) <= kDttol * 2.0;
    }

private:
    static constexpr double kDttol = 0.025; // RTKLIB rtklib.h DTTOL
    double interval;
};
