#pragma once

#include <utility>
#include <memory>
#include <vector>
#include <set>
#include <map>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#include "TimeConvert.h"
#include "CoordStruct.h"
#include "Ephemeris.h"
#include "TimeStruct.h"

struct SatID {
    char system{};
    int id;

    //
    // todo:
    // 增加这个字段，实现北斗2代和北斗3代的区分，
    // 在后面星间差分观测值构建时，需要考虑北斗2和北斗3接收机钟差不同
    // 引起的模型差异
    // int generation;

    // 构造函数
    SatID() : id(-1) {
    }

    SatID(const char sys, const int satID) : system(sys), id(satID) {
    }

    // 从字符串构造函数
    explicit SatID(const string &satStr) {
        if (!satStr.empty()) {
            system = satStr[0];
            id = stoi(satStr.substr(1));
        } else {
            system = '?';
        }
    }

    // Overload the equality operator as a member function
    bool operator==(const SatID &other) const {
        return this->system == other.system && this->id == other.id;
    }

    // Overload the not equal operator as a member function
    bool operator!=(const SatID &other) const {
        return !(*this == other);
    }

    // Overload the less-than operator as a member function
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

    [[nodiscard]] const ReferenceFrame &getFrame() const {
        switch (system) {
            case 'G':
                return Frame::GPS;
            case 'C':
                return Frame::WGS84;
            default:
                return Frame::WGS84;
        }
    }
};

typedef std::set<SatID> SatIDSet;

inline ostream &operator<<(ostream &os, const SatID &sat_id) {
    os << sat_id.system << std::setw(2) << std::setfill('0') << sat_id.id;
    return os;
}


//---------------
// 观测值管理
//---------------
//---------------
// struct used in rinex reader
//---------------
struct RinexHeader {
    string station;
    double version; //< RINEX 3 version/type
    XYZ antennaPosition; //< APPROX POSITION XYZ
    std::map<string, std::vector<string> > mapObsTypes; //< SYS / # / OBS TYPES
    RinexHeader() : version(0) {
    }
};


typedef std::map<std::string, double> TypeValueMap;

inline std::ostream &operator<<(std::ostream &os, const TypeValueMap &typeValueMap) {
    for (const auto &[type, value]: typeValueMap) {
        os << "  Type: " << type
                << ", Value: " << std::fixed << std::setprecision(6) << value << "\n";
    }
    return os;
}

typedef std::map<SatID, double> SatValueMap;

inline std::ostream &operator<<(std::ostream &os, const SatValueMap &satValueMap) {
    for (const auto &[sat, value]: satValueMap) {
        os << "  sat: " << sat
                << ", Value: " << std::fixed << std::setprecision(6) << value << "\n";
    }
    return os;
}


typedef std::map<SatID, TypeValueMap> SatTypeValueMap;
typedef std::map<SatID, Ephemeris *> SatEphemerisMap;

inline std::ostream &operator<<(std::ostream &os, const SatTypeValueMap &satTypeValueMap) {
    for (const auto &[sat, map]: satTypeValueMap) {
        os << "Satellite ID: " << sat << "\n";
        os << map;
    }
    return os;
}

struct ObsData {
    //接收机天线位置
    Eigen::Vector3d antennaPosition{0, 0, 0};
    //接收站名
    string station;
    //接收机接收信号的时刻
    CommonTime epoch;
    //接收机接收信号的时刻的周数和秒数
    WeekSecond weekSecond;
    SatTypeValueMap satTypeValueData;
    SatEphemerisMap satEphemerisData;
};

inline std::ostream &operator<<(std::ostream &os, const ObsData &data) {
    os << "Epoch: " << CommonTime2CivilTime(data.epoch) << "\n"; // 使用 CommonTime 的输出函数

    if (!data.satTypeValueData.empty()) {
        os << "Satellite Data:\n";
        os << data.satTypeValueData; // 使用 SatTypeValueMap 的输出函数
    } else {
        os << "No satellite data available.\n";
    }

    return os;
}

typedef std::map<SatID, std::map<CommonTime, double> > SatEpochValueMap;

//-------------------
// 星历相关数据结构
//-------------------


// Output operator for pvt
inline std::ostream &operator<<(std::ostream &os, PVT &pvt)
    noexcept {
    os << setprecision(10) << "p:" << pvt.p.transpose() << endl;
    os << "v:" << pvt.v.transpose() << endl;
    os << "clk bias:" << pvt.clockBias << endl;
    os << "clk drift:" << pvt.clockDrift << endl;
    os << "relcorr:" << pvt.relativityCorrection << endl;
    for (const auto &[id, tgd]: pvt.typeTGDData)
        os << id << "tgd:" << tgd << endl;
    return os;
}

//---------------
// 参数估计模块数据结构
//---------------
class Parameter {
public:
    enum ParameterName {
        // 显式指定底层类型为int
        Unknown = 0,
        dX,
        dY,
        dZ,
        cdt,
        cdt2,
        ifb,
        iono,
        ambiguity,
        dVX,
        dVY,
        dVZ,
        cdtr_dot,
        count
    };

    Parameter() = default;

    Parameter(const ParameterName _name) : paraName(_name) { //NOLINT
    }

    [[nodiscard]] std::string toString() const {
        // 直接返回对应的字符串，假设调用方保证传入的颜色有效
        return paraNameStrings[static_cast<int>(paraName)];
    }

    static ParameterName toParameterName(const std::string &nameStr) {
        for (int i = Unknown; i < static_cast<int>(count); ++i) {
            if (paraNameStrings[i] == nameStr) return static_cast<ParameterName>(i);
        }
        return Unknown;
    }

    // Overload the equality operator as a member function
    bool operator==(const Parameter &other) const {
        return this->paraName == other.paraName;
    }

    // Overload the not-equal operator as a member function
    bool operator!=(const Parameter &other) const {
        return !(*this == other);
    }

    // Overload the less-than operator as a member function
    bool operator<(const Parameter &other) const {
        return this->paraName < other.paraName;
    }

private:
    ParameterName paraName;
    inline static const std::string paraNameStrings[static_cast<int>(count)] = {
        "Unknown", "dX", "dY", "dZ", "cdt", "ifb", "iono", "ambiguity", "dVX", "dVY", "dVZ", "cdtr_dot"
    };
};

// 全局的 operator<< 函数
inline std::ostream &operator<<(std::ostream &os, const Parameter &v) {
    os << v.toString();
    return os;
}


class ObsID {
public:
    std::string satSys;
    std::string obsType;

    // 默认构造函数
    ObsID() = default;

    // 使用两个字符串参数的构造函数
    ObsID(std::string sys, std::string type)
        : satSys(std::move(sys)), obsType(std::move(type)) {
    }

    // 重载相等运算符
    bool operator==(const ObsID &other) const {
        return this->satSys == other.satSys && this->obsType == other.obsType;
    }

    // 重载小于运算符（用于排序）
    bool operator<(const ObsID &other) const {
        if (this->satSys != other.satSys) {
            return this->satSys < other.satSys;
        }
        return this->obsType < other.obsType;
    }

    [[nodiscard]] std::string toString() const {
        std::stringstream ss;
        ss << satSys << obsType;
        return ss.str();
    }
};

// 定义全局重载的 << 运算符
inline std::ostream &operator<<(std::ostream &os, const ObsID &obsid) {
    os << obsid.satSys << " " << obsid.obsType;
    return os;
}

class Variable {
public:
    // 默认构造函数
    Variable() = default;

    //  (dx, dy, dz, cdt)
    Variable(std::string _station, const Parameter _paraName)
        : station(std::move(_station)), paraName(_paraName) {
    }

    // isb, ion, ambiguity
    Variable(std::string _station, const SatID _sat, const Parameter _paraName, ObsID _obsID)
        : station(std::move(_station)), sat(_sat), obsID(std::move(_obsID)), paraName(_paraName) {
    }

    Variable &operator=(const Variable &right) {
        if (this != &right) {
            this->station = right.station;
            this->sat = right.sat;
            this->obsID = right.obsID;
            this->paraName = right.paraName;
        }
        return *this;
    }

    bool operator<(const Variable &right) const;

    bool operator==(const Variable &right) const;

    bool operator!=(const Variable &right) const;

    // Getter方法

    [[nodiscard]] std::string getStation() const { return station; }
    [[nodiscard]] Parameter getParaType() const { return paraName; }
    [[nodiscard]] SatID getSat() const { return sat; } // 假设 SatID 是 string 类型
    [[nodiscard]] ObsID getObsID() const { return obsID; }
    // 添加 toString 方法
    [[nodiscard]] std::string toString() const {
        std::stringstream ss;
        ss << "Variable{"
                << "station=" << station << ", "
                << "sat=" << sat << ", "
                << "obsID=" << obsID << ", "
                << "paraName=" << paraName
                << "}";
        return ss.str();
    }

    std::string station;
    SatID sat; // 如果 SatID 不是 string，请根据实际情况调整
    ObsID obsID;
    Parameter paraName{};
};

// 全局的 operator<< 函数
inline std::ostream &operator<<(std::ostream &os, const Variable &v) {
    os << v.toString();
    return os;
}

typedef std::set<Variable> VariableSet;
typedef std::map<Variable, double> VariableDataMap;
typedef std::map<Variable, int> VariableIntMap;

//===========
// EquationData
//===========
class EquID {
public:
    SatID sat; // 卫星标识（假设 SatID 已定义）
    std::string_view obsType; // 观测类型
    //    std::string station; // 站点标识

    // 默认构造函数
    EquID() = default;

    // 带参数的构造函数（假设 SatID 可从字符串构造）
    EquID(const SatID sat, const std::string_view type)
        : sat(sat), obsType(type) {
    }

    // 重载相等运算符
    bool operator==(const EquID &other) const {
        return this->sat == other.sat &&
               this->obsType == other.obsType;
    }

    // 重载小于运算符（用于排序）
    bool operator<(const EquID &other) const {
        if (this->sat != other.sat) {
            return this->sat < other.sat;
        }
        return this->obsType < other.obsType;
    }

    [[nodiscard]] std::string toString() const {
        std::stringstream ss;
        ss << "obs{"
                << "sat=" << sat << ", "
                << "obsType=" << obsType
                << "}";
        return ss.str();
    }
};

// 全局的 operator<< 函数
inline std::ostream &operator<<(std::ostream &os, const EquID &equID) {
    os << equID.toString();
    return os;
}

struct EquData {
    double prefit;
    std::map<Variable, double> varCoeffData;
    double weight;
};

// 所有观测方程数据，包括未知参数和每个方程的数据
struct EquSys {
    // 每个观测方程的未知参数和系数及先验残差
    std::string_view station;
    std::map<EquID, EquData> obsEquData;
    // 整个方程系统的所有未知参数
    VariableSet varSet;

    void reset() {
        obsEquData.clear();
        varSet.clear();
        station = "";
    }
};

struct Result {
    Eigen::Vector3d xyz, xyzFixed;
    Eigen::Vector3d blh, blhFixed;
    Eigen::Vector3d vel;
    Eigen::Vector3d sigmaXYZ;
    Eigen::Vector3d sigmaVel;
    double sigmaP;
    double sigmaV;
    double pdop, gdop, hdop, vdop, tdop;
    int numSats;
    double ratio;

    void reset() {
        xyz.setZero();
        xyzFixed.setZero();
        blh.setZero();
        blhFixed.setZero();
        vel.setZero();
        pdop = gdop = 0;
        sigmaP = sigmaV = 0;
        numSats = 0;
        ratio = 0;
    }
};
