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
            const string c2 = pair.second.empty() ? "" : pair.second.substr(1, 1);
            ifTypeNames[sys] = "CC" + pair.first.substr(1, 1) + c2;
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

    double rClockDrift{};

    /// GPS 与 BDS 各自钟差 [m]
    std::map<char, double> rClockBias{};

    /// 系统→钟差参数映射（仅 GPS + BDS）
    static inline std::map<char, Parameter> sysCdtParam = {
        {'G', Parameter::cdt},
        {'C', Parameter::cdt2},
    };

    EquSys posEquations{};
    EquSys velEquations{};
    SolverLSQ posSolver{};
    SolverLSQ velSolver{};

    SatEphemerisMap ephMap{};
    EphemerisTable *ephTable = nullptr;

    std::map<char, std::pair<string, string> > ifCodeTypes{};
    std::map<char, std::string> ifTypeNames{};

    /// 当前历元有卫星的系统集合
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
    /// 上一轮解算的原始残差绝对值 (SatID → |v|)，用于整星座一致性校验
    std::map<SatID, double> lastResidMag_;
};
