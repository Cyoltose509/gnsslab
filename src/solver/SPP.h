#pragma once

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "Ephemeris.h"
#include "EphemerisTable.h"
#include "RinexNavStore.h"   // loadBrdc：广播星历加载（与 PPP 一致，类内加载）
#include "FreqCombo.h"
#include <Eigen/Eigen>
#include <set>

// SPP 的 IF 组合类型
using IFCodeTypes = std::map<char, FreqCombo>;

class SPP {
public:
    void setEphemeris(const SatEphemerisMap &ephs) {
        ephMap = ephs;
    }

    bool loadBrdc(const std::string &path);

    void detectIFCombinations(const ObsData &obsData, const IFCodeTypes &defaultTypes = {});

    void setCutoffElevDeg(const double deg) { cutOffElev = deg * PI / 180.0; }

    virtual void preprocess(ObsData &obsData);

    virtual void solve(ObsData &obsData);

    virtual void readback(const ObsData &obsData);

    virtual void computeSatPos(ObsData &obsData);

    void computeElevAzim();

    void earthRotation();

    virtual void linearize(ObsData &obsData, int iter);

    /// IF-code 解算的结果位置和钟差，供 UC 模式热启动
    double getClockBias(const char sys) const {
        const auto it = rClockBias.find(sys);
        return it != rClockBias.end() ? it->second : 0.0;
    }

    virtual ~SPP() = default;

    Vector3d xyz{0, 0, 0};
    Vector3d vel{0, 0, 0};
    FrameInfo frame = Frame::WGS84;
    Result result{};
    SatValueMap satElevData{};
    SatValueMap satAzimData{};
    std::set<SatID> satRejected{};
    std::map<SatID, PVT> satPVTTransTime{};
    std::map<SatID, PVT> satPVTRecTime{};

    std::set<char> enabledSystems; // 参与解算的星座集合

    EphemerisTable ephTable; // 星历表（值存储，类内拥有）；文件模式由 loadBrdc 填充，流式模式每历元覆盖

protected:
    double cutOffElev = PI * 0.0555556;
    bool isRover = true;
    bool mRequirePhaseForIF = false;
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

    IFCodeTypes ifCodeTypes{}; // 系统 → FreqCombo（码名 + 频率/系数/波长，及 MW/GF 组合）

    /// 当前历元有卫星的系统集合
    std::set<char> activeSystems;

    /// 上一轮解算的标准化残差和原始残差，用于粗差探测
    std::map<SatID, double> lastWStats;
    std::map<SatID, double> lastResidMag;

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
};
