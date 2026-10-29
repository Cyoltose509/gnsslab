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

#include "Exception.h"
#include "TimeConvert.h"
#include "CoordStruct.h"
#include "CoordConvert.h"
#include "TimeStruct.h"

struct SatID {
    char system{};
    int id;
    /// GLONASS FDMA 频率通道号 k（RINEX "GLONASS SLOT / FRQ #" 给出）；
    /// 仅 GLONASS 使用，其它系统恒为 0。不参与 operator==/</toString 身份（卫星身份由 system+id 决定）。
    int chan{0};

    SatID() : id(-1) {
    }

    SatID(const char sys, const int satID) : system(sys), id(satID) {
    }

    SatID(const char sys, const int satID, const int channel) : system(sys), id(satID), chan(channel) {
    }

    explicit SatID(const string &substr) {
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

inline ostream &operator<<(ostream &os, const SatID &sat_id) {
    os << sat_id.system << std::setw(2) << std::setfill('0') << sat_id.id;
    return os;
}


struct RinexHeader {
    string station;
    double version;
    string antType;
    XYZ antennaPosition;
    Eigen::Vector3d antDeltaENU{0, 0, 0};
    std::map<char, std::vector<string> > mapObsTypes;
    // RINEX "SYS / PHASE SHIFT"：各相位观测的半周/整数周偏置
    std::map<std::string, double> phaseShift;

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
    //接收机天线型号（RINEX "ANT # / TYPE"），用于 ATX 查 PCO/PCV
    string antType;
    //接收站名
    string station;
    //接收机接收信号的时刻
    CommonTime epoch;
    //接收机接收信号的时刻的周数和秒数
    WeekSecond weekSecond;
    SatTypeValueMap satTypeValueData;
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
        cdt3,
        cdt4,
        ifb,
        iono,
        ambiguity,
        dVX,
        dVY,
        dVZ,
        cdtr_dot,
        ztd,
        dOX, dOY, dOZ, // 接收机天线/参考点常值 ECEF 偏移
        codeBias, // 每星伪距(IF code)系统偏置估计：吸收多路径/未建模硬件延迟，使紧码权不再污染位置
        count
    };

    Parameter() = default;

    Parameter(const ParameterName _name) : paraName(_name) { //NOLINT
    }

    [[nodiscard]] std::string toString() const {
        // 直接返回对应的字符串，假设调用方保证传入的颜色有效
        return paraNameStrings[static_cast<int>(paraName)];
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
        "Unknown", "dX", "dY", "dZ", "cdt", "cdt2", "cdt3", "cdt4", "ifb", "iono", "ambiguity",
        "dVX", "dVY", "dVZ", "cdtr_dot", "ztd", "dOX", "dOY", "dOZ", "codeBias"
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

// 模糊度状态变量构造：RTK 与 PPP 共用同一模式(仅 station / 观测类型标签不同)。
// sys 取自 sat.system；obsType 为 "L1"/"L2"(RTK 双频) 或 "IF"(PPP 无电离层组合)。
inline Variable makeAmbiguityVar(const std::string &station, const SatID &sat, const std::string &obsType) {
    return {station, sat, Parameter::ambiguity, ObsID(std::string(1, sat.system), obsType)};
}

inline Variable makeCodeBiasVar(const std::string &station, const SatID &sat, const std::string &obsType) {
    return {station, SatID(sat.system, -1), Parameter::codeBias, ObsID(std::string(1, sat.system), obsType)};
}

// 逐 GLONASS 星 / 频点 IFCB（Inter-Frequency Code Bias）状态量构造：用真实 PRN 作键（非 -1 通配符），
// 因为 GLONASS FDMA 频分多址致每星频率不同，其频间码偏是逐星独立、无法用逐系统(per-system)标量装下。
// 仅在 GLONASS 的码(伪距)观测方程接入，系数 1.0（相位 IF 组合已吸收频间项，IFCB 主要体现于码）。
inline Variable makeIfbVar(const std::string &station, const SatID &sat, const std::string &obsType) {
    return {station, sat, Parameter::ifb, ObsID(std::string(1, sat.system), obsType)};
}

typedef std::set<Variable> VariableSet;
typedef std::map<Variable, double> VariableDataMap;
typedef std::map<Variable, int> VariableIntMap;


class EquID {
public:
    SatID sat; // 卫星标识（假设 SatID 已定义）
    std::string obsType; // 观测类型（拥有式；勿用 string_view，否则临时串会悬垂）
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
    double varBudget = 0.0; // 完整误差方差预算(对流层+轨道/钟差)，仅供后验抗差 sigma 使用，不改滤波权
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
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzFixed = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzFloat = Eigen::Vector3d::Zero();
    Eigen::Vector3d blh = Eigen::Vector3d::Zero();
    Eigen::Vector3d blhFixed = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d sigmaXYZ = Eigen::Vector3d::Zero();
    Eigen::Vector3d sigmaVel = Eigen::Vector3d::Zero();
    double sigmaP = 0;
    double sigmaPFloat = -1; // 浮点解 σ
    double sigmaPFixed = -1; // 固定解 σ
    double sigmaV = 0;
    double pdop = 0, gdop = 0, hdop = 0, vdop = 0, tdop = 0;
    int numSats = 0;
    double ratio = 0;
    std::map<SatID, double> postRes; // 伪距(码)后验残差, m
    std::map<SatID, double> postResPhase; // 相位后验残差, m (SPP 无相位, 恒空)
    unsigned char stat = 0; // 解状态: 0=无效, 1=Fixed, 2=Float

    void reset() {
        xyz.setZero();
        xyzFixed.setZero();
        xyzFloat.setZero();
        blh.setZero();
        blhFixed.setZero();
        vel.setZero();
        sigmaXYZ.setZero();
        sigmaVel.setZero();
        pdop = gdop = hdop = vdop = tdop = 0;
        sigmaP = sigmaV = 0;
        sigmaPFloat = -1;
        sigmaPFixed = -1;
        numSats = 0;
        ratio = 0;
        postRes.clear();
        postResPhase.clear();
        stat = 0;
    }
};

// 由位置/钟差协方差算 DOP 族（SPP/RTK/PPP 三处共用，集中在 ENU 投影这一易错步）：
//   pdop = √tr(Qpos)；gdop = √(pdop² + σ²_clk)；tdop = σ_clk（clockVar ≤ 0 时记 0）
//   hdop/vdop = ENU 投影协方差 C_enu = C_e^enu·Qpos·C_e^enuᵀ 的水平/天向迹
// clockVar 传 0.0 表示不估计接收机钟差（RTK），此时 tdop = 0。
inline void computePositionDops(Result &result, const Eigen::Matrix3d &Qpos, double clockVar, const Eigen::Vector3d &blh) {
    result.pdop = std::sqrt(Qpos(0, 0) + Qpos(1, 1) + Qpos(2, 2));
    result.gdop = std::sqrt(result.pdop * result.pdop + clockVar);
    result.tdop = clockVar > 0.0 ? std::sqrt(clockVar) : 0.0;
    const Eigen::Matrix3d R = ECEFToENUDCM(blh[0], blh[1]);
    const Eigen::Matrix3d C_enu = R * Qpos * R.transpose();
    result.hdop = std::sqrt(C_enu(0, 0) + C_enu(1, 1));
    result.vdop = std::sqrt(C_enu(2, 2));
}