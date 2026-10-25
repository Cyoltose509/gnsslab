#pragma once



#include "GnssStruct.h"
#include "SolverKalman.h"

class GnssEKF {
public:
    GnssEKF() : firstTime(true) {
    }

    virtual void solve(EquSys &equSys, VariableDataMap &csData);

    void timeUpdate(const VariableSet &varSet, VariableDataMap *csData = nullptr);

    void measUpdate(EquSys &equSys);

    void createIndex(const VariableSet &varSet);

    static double getSolution(const Parameter &type,
                              const VariableSet &currentUnkSet,
                              const Eigen::VectorXd &stateVec);

    static bool tryGetSolution(const Parameter &type, const VariableSet &currentUnkSet,
                               const Eigen::VectorXd &stateVec, double &out);

    [[nodiscard]] double getSolution(const Variable &var) const;

    void forceValue(const Variable &var, const double v) { mForceValue[var] = v; }

    /// 保存“量测更新前”的先验(xhatminus/Pminus)。
    /// 供 rtkpost 式迭代抗差使用：检出粗差后必须回到**同一**先验重解，
    /// 否则会把好的观测重复计入（状态被更新两次）。
    void savePrior() {
        savedXhatminus = kalmanFilter.xhatminus;
        savedPminus = kalmanFilter.Pminus;
        hasSaved = true;
    }

    void configure(const double posNoise, const bool hasTropo,
                   const double ambNoise = 1.0E-8,
                   const double clockNoise = 0.0) {
        posProcNoise = posNoise;
        useTropo = hasTropo;
        ambProcNoise = ambNoise;
        clockProcNoise = clockNoise;
    }

    void reset() {
        firstTime = true;
        solution = Eigen::VectorXd();
        covMatrix = Eigen::MatrixXd();
        oldUnkSet.clear();
    }

    /// 首历元引导：仅当尚处于 firstTime（无任何后验）时，把位置先验设为近似点。
    void setBootstrapPosition(const Eigen::Vector3d &p, const double posVar = 0.0) {
        if (currentUnkSet.empty()) return; // timeUpdate 尚未建索引时无效
        for (const auto &var: currentUnkSet) {
            const auto pt = var.getParaType();
            const int idx = currentIndexData[var];
            if (pt == Parameter::dX) {
                xhat(idx) = p[0];
                kalmanFilter.xhat(idx) = p[0];
                kalmanFilter.xhatminus(idx) = p[0];
            } else if (pt == Parameter::dY) {
                xhat(idx) = p[1];
                kalmanFilter.xhat(idx) = p[1];
                kalmanFilter.xhatminus(idx) = p[1];
            } else if (pt == Parameter::dZ) {
                xhat(idx) = p[2];
                kalmanFilter.xhat(idx) = p[2];
                kalmanFilter.xhatminus(idx) = p[2];
            }
        }
        if (posVar > 0.0) {
            for (const auto &var: currentUnkSet) {
                if (const auto pt = var.getParaType(); pt == Parameter::dX || pt == Parameter::dY || pt == Parameter::dZ) {
                    const int idx = currentIndexData[var];
                    kalmanFilter.Pminus(idx, idx) = posVar;
                    kalmanFilter.P(idx, idx) = posVar;
                    P(idx, idx) = posVar;
                }
            }
        }
    }

    /// 逐分量首历元位置先验方差(m^2)
    void setBootstrapPosition(const Eigen::Vector3d &p, const Eigen::Vector3d &posVar) {
        if (currentUnkSet.empty()) return; // timeUpdate 尚未建索引时无效
        for (const auto &var: currentUnkSet) {
            const auto pt = var.getParaType();
            const int idx = currentIndexData[var];
            if (pt == Parameter::dX) {
                xhat(idx) = p[0];
                kalmanFilter.xhat(idx) = p[0];
                kalmanFilter.xhatminus(idx) = p[0];
            } else if (pt == Parameter::dY) {
                xhat(idx) = p[1];
                kalmanFilter.xhat(idx) = p[1];
                kalmanFilter.xhatminus(idx) = p[1];
            } else if (pt == Parameter::dZ) {
                xhat(idx) = p[2];
                kalmanFilter.xhat(idx) = p[2];
                kalmanFilter.xhatminus(idx) = p[2];
            }
        }
        for (const auto &var: currentUnkSet) {
            const auto pt = var.getParaType();
            if (pt != Parameter::dX && pt != Parameter::dY && pt != Parameter::dZ) continue;
            const int idx = currentIndexData[var];
            const double pv = pt == Parameter::dX ? posVar[0] : pt == Parameter::dY ? posVar[1] : posVar[2];
            kalmanFilter.Pminus(idx, idx) = pv;
            kalmanFilter.P(idx, idx) = pv;
            P(idx, idx) = pv;
        }
    }


    void shiftState(const Variable &var, const double delta) {
        if (currentUnkSet.empty()) return;
        if (const auto it = currentIndexData.find(var); it != currentIndexData.end()) {
            const int idx = it->second;
            kalmanFilter.xhat(idx) += delta;
            kalmanFilter.xhatminus(idx) += delta;
        }
    }

    /// 首历元钟差/ZTD 引导
    void setBootstrapStates(const std::map<Variable, double> &values,
                            const std::map<Variable, double> *vars = nullptr) {
        if (currentUnkSet.empty()) return;
        for (const auto &[var, value]: values) {
            if (const auto it = currentIndexData.find(var); it != currentIndexData.end()) {
                const int idx = it->second;
                kalmanFilter.xhat(idx) = value;
                kalmanFilter.xhatminus(idx) = value;
                double v = -1.0;
                if (vars) {
                    if (const auto vit = vars->find(var); vit != vars->end()) v = vit->second;
                }
                if (v > 0.0) {
                    kalmanFilter.Pminus(idx, idx) = v;
                    kalmanFilter.P(idx, idx) = v;
                    P(idx, idx) = v;
                }
            }
        }
    }

    /// 模糊度引导
    void setBootstrapAmbiguity(const std::string &station, const std::map<SatID, double> &amb,
                               const VariableDataMap *csData = nullptr) {
        if (currentUnkSet.empty()) return;
        for (const auto &[sat, value]: amb) {
            const Variable vamb = makeAmbiguityVar(station, sat, "IF");
            if (csData) {
                if (const auto itc = csData->find(vamb); itc == csData->end() || itc->second < 0.5) continue; // 仅新弧段/周跳重置
            }
            if (auto it = currentIndexData.find(vamb); it != currentIndexData.end()) {
                const int idx = it->second;
                kalmanFilter.xhat(idx) = value;
                kalmanFilter.xhatminus(idx) = value;
            }
        }
    }

    void setPosProcNoise(const double v) { posProcNoise = v; }

    virtual ~GnssEKF() = default;

    VariableSet currentUnkSet;
    VariableIntMap currentIndexData;
    std::vector<Variable> idxToVar_; // 与 currentIndexData 同步：idx -> var（timeUpdate 按整数索引遍历，免反查）

private:
    [[nodiscard]] double initVariance(const Parameter &p) const;

    bool firstTime;
    bool useTropo = true;
    double posProcNoise = 0.0;

public:
    double ambProcNoise = 1.0E-8;
    double ambInitVar = 100.0;
    double clockProcNoise = 0.0;
    double ztdProcNoise = 1.0E-6;
    double posVarFloor = 0.0;
    Eigen::Vector3d posVarFloorVec = Eigen::Vector3d(0.0, 0.0, 0.0);
    double ztdInitVar = 0.04;
    double ztdInitValue = 0.0;
    double clockInitVar = 1.0E4;
    Eigen::VectorXd solution; // 后验状态向量 x
    Eigen::MatrixXd covMatrix; // 后验协方差 P
    Eigen::VectorXd postfitResidual;
    Eigen::Vector3d dxyz;

private:
    Eigen::VectorXd xhat;
    Eigen::MatrixXd P;
    VariableSet oldUnkSet;
    VariableIntMap oldIndexData;

    SolverKalman kalmanFilter;

    Eigen::VectorXd savedXhatminus;
    Eigen::MatrixXd savedPminus;
    bool hasSaved = false;

    std::map<Variable, double> mForceValue;

public:

    double codeBiasProcNoise = 1.0E-8;
    double codeBiasInitVar = 1.0;
    double isbProcNoise = 0.0;
    double isbInitVar = 0.0;

    double gloIfcbProcNoise = 1.0E-8;
    double gloIfcbInitVar = 1.0;
};
