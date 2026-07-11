#pragma once

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "Ephemeris.h"
#include "EphemerisTable.h"
#include <Eigen/Eigen>

class SPPIFCode {
public:
    void setStationAsBase() {
        isRover = false;
    }

    /// 文件模式：传入星历表（数据由 EphemerisTable 拥有，SPPIFCode 不持所有权）
    void setEphemerisTable(EphemerisTable *table) { ephTable_ = table; }

    /// 实时模式：从 ObsData 拷贝星历指针（兼容旧路径）
    void setEphemeris(const SatEphemerisMap &ephs) {
        ephMap = ephs;
    }

    void setIFCodeTypes(const std::map<char, std::pair<string, string> > &ifTypes) {
        ifCodeTypes = ifTypes;
    }

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
    bool oldVersion=false;

protected:
    double cutOffElev = PI * 0.0555556;
    bool isRover = true;
    double sigIFCode = 0.3;

    double rClockBias{};   // 接收机钟差 (GPS)
    double rClockDrift{};  // 接收机钟漂
    double rClockBiasBDS{};// 接收机钟差 (BDS)

    EquSys posEquations{};
    EquSys velEquations{};
    SolverLSQ posSolver{};
    SolverLSQ velSolver{};

    SatEphemerisMap ephMap{};           // 实时模式：指向 ObsData 中的星历（后向兼容）
    EphemerisTable *ephTable_ = nullptr;  // 文件模式：指向外部星历表

    std::map<char, std::pair<string, string> > ifCodeTypes{};

private:
    /// 解算完成后计算每颗卫星的 Baarda w 统计量
    void buildResidualMap();

    /// 构建单颗卫星的位置观测方程
    void buildPosEquation(const SatID &sat, const PVT &pvt, const Vector3d &los,
                          double rho, double trop,
                          const string &obsType, double obsVal, double weight,
                          const class Variable &dx, const class Variable &dy,
                          const class Variable &dz, const class Variable &cdt,
                          const class Variable &cdt2);

    /// 构建单颗卫星的速度观测方程
    void buildVelEquation(const SatID &sat, const TypeValueMap &codeList,
                          const PVT &pvt, const Vector3d &los,
                          double elev, double posWeight,
                          const class Variable &dvx, const class Variable &dvy,
                          const class Variable &dvz, const class Variable &dcdt);

    /// 上一轮解算的标准化残差 |v|/σ₀ (SatID → 值)，用于粗差探测
    std::map<SatID, double> lastWStats_;
};
