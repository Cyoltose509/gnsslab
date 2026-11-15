#pragma once

#include <map>
#include <ostream>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include <Eigen/Core>

#include "core/SatID.h"

class Parameter {
public:
    enum ParameterName {
        Unknown = 0,
        dX,
        dY,
        dZ,
        dVX,
        dVY,
        dVZ,
        cdt,
        cdt2,
        cdt3,
        cdt4,
        ifb,
        iono,
        ambiguity,
        ztd,
        dOX, dOY, dOZ,
        codeBias,
        count
    };

    Parameter() = default;

    Parameter(const ParameterName _name) : paraName(_name) { //NOLINT
    }

    [[nodiscard]] std::string toString() const {
        return paraNameStrings[static_cast<int>(paraName)];
    }

    /// 参数种类枚举值：方差表按它下标查，避免各处再写一遍类型词表
    [[nodiscard]] ParameterName name() const { return paraName; }

    bool operator==(const Parameter &other) const {
        return this->paraName == other.paraName;
    }


    bool operator!=(const Parameter &other) const {
        return !(*this == other);
    }

    /// 接收机钟差类：cdt 是参考系统钟，cdt2~4 是其余系统的钟（各系统各一个，同时充当 ISB 载体）
    [[nodiscard]] bool isClock() const {
        return paraName >= cdt && paraName <= cdt4;
    }


    bool operator<(const Parameter &other) const {
        return this->paraName < other.paraName;
    }

private:
    ParameterName paraName;
    inline static const std::string paraNameStrings[static_cast<int>(count)] = {
        "Unknown", "dX", "dY", "dZ", "dVX", "dVY", "dVZ", "cdt", "cdt2", "cdt3", "cdt4", "ifb",
        "iono", "ambiguity", "ztd", "dOX", "dOY", "dOZ", "codeBias"
    };
};


inline std::ostream &operator<<(std::ostream &os, const Parameter &v) {
    os << v.toString();
    return os;
}


/// 观测类型标识：系统 + 观测码（如 "G" + "C1C"）。
class ObsID {
public:
    std::string satSys;
    std::string obsType;


    ObsID() = default;


    ObsID(std::string sys, std::string type)
        : satSys(std::move(sys)), obsType(std::move(type)) {
    }


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

/// 未知参数的键：站 + 卫星 + 观测类型 + 参数种类，四元组全序即方程系统的变量编号顺序。
class Variable {
public:
    // 默认构造函数
    Variable() = default;

    //  (dx, dy, dz, cdt)
    Variable(std::string _station, const Parameter p)
        : station(std::move(_station)), par(p) {
    }

    // isb, ion, ambiguity
    Variable(std::string _station, const SatID _sat, const Parameter p, ObsID id)
        : station(std::move(_station)), sat(_sat), obsID(std::move(id)), par(p) {
    }

    Variable &operator=(const Variable &right) {
        if (this != &right) {
            this->station = right.station;
            this->sat = right.sat;
            this->obsID = right.obsID;
            this->par = right.par;
        }
        return *this;
    }

    bool operator<(const Variable &right) const;

    bool operator==(const Variable &right) const;

    bool operator!=(const Variable &right) const;


    [[nodiscard]] std::string toString() const {
        std::stringstream ss;
        ss << "Variable{"
                << "station=" << station << ", "
                << "sat=" << sat << ", "
                << "obsID=" << obsID << ", "
                << "paraName=" << par
                << "}";
        return ss.str();
    }

    std::string station;
    SatID sat;
    ObsID obsID;
    Parameter par{};
};

inline std::ostream &operator<<(std::ostream &os, const Variable &v) {
    os << v.toString();
    return os;
}

// 模糊度状态变量构造
inline Variable makeAmbiguityVar(const std::string &station, const SatID &sat, const std::string &obsType) {
    return {station, sat, Parameter::ambiguity, ObsID(std::string(1, sat.system), obsType)};
}

inline Variable makeCodeBiasVar(const std::string &station, const SatID &sat, const std::string &obsType) {
    return {station, SatID(sat.system, -1), Parameter::codeBias, ObsID(std::string(1, sat.system), obsType)};
}

// 逐 GLONASS 星 / 频点 IFCB（Inter-Frequency Code Bias）状态量构造
inline Variable makeIfbVar(const std::string &station, const SatID &sat, const std::string &obsType) {
    return {station, sat, Parameter::ifb, ObsID(std::string(1, sat.system), obsType)};
}

typedef std::set<Variable> VariableSet;
typedef std::map<Variable, double> VariableDataMap;
typedef std::map<Variable, int> VariableIntMap;


/// 单条观测方程的键：卫星 + 观测类型（站由 EquSys::station 承担）。
class EquID {
public:
    SatID sat;
    std::string obsType; // 观测类型（拥有式；勿用 string_view，否则临时串会悬垂）

    EquID() = default;

    EquID(const SatID sat, const std::string_view type)
        : sat(sat), obsType(type) {
    }

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
    double prefit = 0.0;
    std::map<Variable, double> varCoeffData;
    double weight = 1.0;
    double varBudget = 0.0; // 完整误差方差预算(对流层+轨道/钟差)，仅供后验抗差 sigma 使用，不改滤波权
};

// 所有观测方程数据，包括未知参数和每个方程的数据
struct EquSys {
    // 每个观测方程的未知参数和系数及先验残差
    std::string_view station;
    std::map<EquID, EquData> obsEquData;
    // 整个方程系统的所有未知参数
    VariableSet varSet;

    /// 按观测方程的写入顺序取权阵对角
    [[nodiscard]] Eigen::VectorXd weightVector() const;

    void reset() {
        obsEquData.clear();
        varSet.clear();
        station = "";
    }
};
