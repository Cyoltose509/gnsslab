#pragma once

#include <iomanip>
#include <ostream>
#include <set>
#include <sstream>
#include <string>

#include "Exception.h"

/// 卫星标识（系统 + PRN）。GLONASS 的 FDMA 通道号 chan 只是解算时携带的附加信息，
/// 不参与身份比较——同一颗星在不同历元（头里通道号可能缺失）必须落到同一个键上。
struct SatID {
    char system{};
    int id;
    int chan{0};

    SatID() : id(-1) {
    }

    SatID(const char sys, const int satID) : system(sys), id(satID) {
    }

    SatID(const char sys, const int satID, const int channel) : system(sys), id(satID), chan(channel) {
    }

    explicit SatID(const std::string &substr) {
        if (substr.length() != 3)
            throw StringException("SatID 由 3 个字符组成（系统 + 两位 PRN），收到 \"" + substr + "\"");
        system = substr[0];
        id = std::stoi(substr.substr(1, 2));
    }

    bool operator==(const SatID &other) const {
        return this->system == other.system && this->id == other.id;
    }

    bool operator!=(const SatID &other) const {
        return !(*this == other);
    }

    bool operator<(const SatID &other) const {
        if (this->system != other.system)
            return this->system < other.system;
        return this->id < other.id;
    }

    [[nodiscard]] std::string toString() const {
        std::stringstream sstream;
        sstream << system
                << std::setw(2) << std::setfill('0') << id;
        return sstream.str();
    }
};

typedef std::set<SatID> SatIDSet;

inline std::ostream &operator<<(std::ostream &os, const SatID &sat_id) {
    os << sat_id.system << std::setw(2) << std::setfill('0') << sat_id.id;
    return os;
}
