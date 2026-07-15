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

    /// 根据可用观测类型自动检测并设置 IF 码组合；
    /// 若检测结果为空（无可用的双频组合），则回退到 defaultTypes。
    /// GPS: C1? + C2?    BDS: C2?/C1? + C6? (fallback C7?+C6?)
    void setIFCodeTypesAuto(const std::map<char, std::vector<string>> &availableTypes,
                            const IFCodeTypes &defaultTypes);

    /// 读取当前已设置的 IF 码组合（含自动检测/默认兜底结果）
    const IFCodeTypes &getIFCodeTypes() const { return ifCodeTypes; }

    virtual void preprocess(ObsData &obsData);

    virtual void solve(ObsData &obsData);

    virtual void getResult();

    virtual void computeSatPos(ObsData &obsData);

    void computeElevAzim();

    void computeIF(ObsData &obsData);

    void earthRotation();

    virtual void linearize(ObsData &obsData, int iter);

    void setFrame(const FrameInfo &f) {
        frame = f;
    }

    /// IF-code 解算的结果位置和钟差，供 UC 模式热启动
    double getClockBias(char sys) const {
        auto it = rClockBias.find(sys);
        return it != rClockBias.end() ? it->second : 0.0;
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

    /// 上一轮解算的标准化残差和原始残差，用于粗差探测
    std::map<SatID, double> lastWStats;
    std::map<SatID, double> lastResidMag_;

    /// 解算完成后计算每颗卫星的 Baarda w 统计量
    virtual void buildResidualMap();

    void buildVelEquation(const SatID &sat, const TypeValueMap &codeList,
                          const PVT &pvt, const Vector3d &los,
                          double elev, double posWeight,
                          const Variable &dvx, const Variable &dvy,
                          const Variable &dvz, const Variable &dcdt);

private:
    /// 构建单颗卫星的位置观测方程（IF 组合）
    void buildPosEquation(const SatID &sat, const PVT &pvt, const Vector3d &los,
                          double rho, double trop,
                          const string &obsType, double obsVal, double weight,
                          const Variable &dx, const Variable &dy,
                          const Variable &dz, const Variable &cdtVar);

    /// 纯检测：根据可用观测类型选择最佳双频组合（不修改实例状态）
    static IFCodeTypes detectIFTypes(const std::map<char, std::vector<string>> &availableTypes);
};
