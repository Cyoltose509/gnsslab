#pragma once

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "Ephemeris.h"
#include <Eigen/Eigen>

class SPPIFCode {
public:
    void setStationAsBase() {
        isRover = false;
    }

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

    /// 构建全部观测方程（单次遍历卫星 → 逐颗调用子函数）
    /// @param iter 当前迭代轮数（从 0 开始），前 2 轮跳过 w 检验以避免收敛前误伤
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

    SatEphemerisMap ephMap{};

    std::map<char, std::pair<string, string> > ifCodeTypes{};

private:
    /// 解算完成后计算每颗卫星的 Baarda w 统计量
    void buildResidualMap();

    /// 构建单颗卫星的位置观测方程
    void buildPosEquation(const SatID &sat, const TypeValueMap &codeList,
                          const PVT &pvt, const Vector3d &los,
                          double rho, double elev, double trop,
                          const std::string &obsType, double obsVal, double weight);

    /// 构建单颗卫星的速度观测方程
    void buildVelEquation(const SatID &sat, const TypeValueMap &codeList,
                          const PVT &pvt, const Vector3d &los,
                          double elev, double posWeight);

    /// 上一轮解算的标准化残差 |v|/σ₀ (SatID → 值)，用于粗差探测
    std::map<SatID, double> lastWStats_;
};
