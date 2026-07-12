#pragma once

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "Ephemeris.h"
#include "EphemerisTable.h"
#include <Eigen/Eigen>

typedef std::map<char, std::pair<string, string> > IFCodeTypes;

class SPPIFCode {
public:
    void setStationAsBase() {
        isRover = false;
    }

    /// 文件模式：传入星历表（数据由 EphemerisTable 拥有，SPPIFCode 不持所有权）
    void setEphemerisTable(EphemerisTable *table) { ephTable = table; }

    /// 实时模式：从 ObsData 拷贝星历指针（兼容旧路径）
    void setEphemeris(const SatEphemerisMap &ephs) {
        ephMap = ephs;
    }

    void setIFCodeTypes(const IFCodeTypes &ifTypes) {
        ifCodeTypes = ifTypes;
        for (const auto &[sys, pair]: ifCodeTypes) {
            ifTypeNames[sys] = "CC" + pair.first.substr(1, 1) + pair.second.substr(1, 1);
        }
    }

    /// 自动检测 IF 码类型：根据可用观测类型选择最佳双频组合
    /// GPS: C1? + C2?    BDS: C2?/C1? + C6? (fallback C7?+C6?)
    static IFCodeTypes autoDetectIFTypes(const std::map<char, std::vector<string>> &availableTypes);

    void preprocess(ObsData &obsData);

    void solve(ObsData &obsData);

    void getResult();

    void computeSatPos(ObsData &obsData);

    void computeElevAzim();

    void computeIF(ObsData &obsData);

    void earthRotation();

    void linearize(ObsData &obsData, int iter);

    void setFrame(const FrameInfo &f) {
        frame = f;
    }

    ~SPPIFCode() = default;

    Vector3d xyz{0, 0, 0};
    Vector3d vel{0, 0, 0};
    FrameInfo frame = Frame::WGS84;
    Result result{};
    SatValueMap satElevData{};
    SatValueMap satAzimData{};
    std::set<SatID> satRejected{};
    std::map<SatID, PVT> satPVTTransTime{};
    std::map<SatID, PVT> satPVTRecTime{};

protected:
    double cutOffElev = PI * 0.0555556;
    bool isRover = true;
    double sigIFCode = 0.3;

    double rClockDrift{}; // 接收机钟漂

    /// 各系统钟差 [m]：key=系统标识(G=GPS, C=BDS, R=GLONASS, E=Galileo)
    std::map<char, double> rClockBias{};

    /// 系统→钟差参数映射（未来加 GLONASS：{'R', Parameter::cdt3}）
    static inline std::map<char, Parameter> sysCdtParam = {
        {'G', Parameter::cdt},
        {'C', Parameter::cdt2},
        {'R', Parameter::cdt3},  // GLONASS 预留
    };

    EquSys posEquations{};
    EquSys velEquations{};
    SolverLSQ posSolver{};
    SolverLSQ velSolver{};

    SatEphemerisMap ephMap{}; // 实时模式：指向 ObsData 中的星历（后向兼容）
    EphemerisTable *ephTable = nullptr; // 文件模式：指向外部星历表

    std::map<char, std::pair<string, string> > ifCodeTypes{};
    std::map<char, std::string> ifTypeNames{}; // 预计算的 IF 类型名

    /// 当前历元有卫星的系统集合（用于避免 query 未参解算系统的钟差参数）
    std::set<char> activeSystems;

private:
    /// 解算完成后计算每颗卫星的 Baarda w 统计量
    void buildResidualMap();

    /// 构建单颗卫星的位置观测方程
    void buildPosEquation(const SatID &sat, const PVT &pvt, const Vector3d &los,
                          double rho, double trop,
                          const string &obsType, double obsVal, double weight,
                          const Variable &dx, const Variable &dy,
                          const Variable &dz, const Variable &cdtVar);

    /// 构建单颗卫星的速度观测方程
    void buildVelEquation(const SatID &sat, const TypeValueMap &codeList,
                          const PVT &pvt, const Vector3d &los,
                          double elev, double posWeight,
                          const Variable &dvx, const Variable &dvy,
                          const Variable &dvz, const Variable &dcdt);

    /// 上一轮解算的标准化残差 |v|/σ₀ (SatID → 值)，用于粗差探测
    std::map<SatID, double> lastWStats_;
};
