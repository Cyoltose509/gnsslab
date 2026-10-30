#pragma once
#include "GnssStruct.h"
#include "MathUtils.h"
#include <vector>
#include <map>
#include <string>

#include "Ephemeris.h"


class Sp3OrbitReader {
public:
    static constexpr int kSp3FieldW = 14;
    static constexpr int kSp3IdLen = 4;

    static SatID parseSatId(const std::string &s) {
        if (s.size() < 3) return {};
        return {s[0], std::stoi(s.substr(1, 2))};
    }


    static bool isValid(const double v) { return std::fabs(v) < 999999.0; }


    static bool getField(const std::string &line, const int i, double &out) {
        const size_t pos = kSp3IdLen + kSp3FieldW * static_cast<size_t>(i);
        if (pos >= line.size()) return false;
        const std::string f = line.substr(pos, kSp3FieldW);
        const size_t a = f.find_first_not_of(' ');
        if (a == std::string::npos) return false;
        try { out = std::stod(f.substr(a)); } catch (...) { return false; }
        return !std::isnan(out);
    }
    struct Record {
        CommonTime t;
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        double clock = 0.0;
        double clockRate = 0.0;
        bool hasVel = false;
        bool hasClock = false;
        bool hasClockRate = false;
    };


    bool read(const std::string &path);

    [[nodiscard]] bool empty() const { return data.empty(); }


    [[nodiscard]] bool hasClock() const { return anyClock; }


    [[nodiscard]] bool contains(const SatID &sat) const {
        const auto it = data.find(sat);
        return it != data.end() && !it->second.empty();
    }


    [[nodiscard]] bool covers(const SatID &sat, const CommonTime &t) const {
        const auto it = data.find(sat);
        if (it == data.end() || it->second.empty()) return false;

        constexpr double tol = 900.0;
        return t - it->second.front().t >= -tol && it->second.back().t - t >= -tol;
    }


    [[nodiscard]] PVT getPVT(const SatID &sat, const CommonTime &t) const;


    [[nodiscard]] std::pair<CommonTime, CommonTime> timeSpan(const SatID &sat) const;

    [[nodiscard]] int satCount() const { return static_cast<int>(data.size()); }


    [[nodiscard]] const std::map<SatID, std::vector<Record> > &getData() const { return data; }

private:
    std::map<SatID, std::vector<Record> > data;
    std::map<SatID, std::vector<double> > relTimes;
    bool anyClock = false;
};
