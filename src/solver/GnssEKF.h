#pragma once


#include <array>

#include "core/equ/EquStruct.h"
#include "SolverKalman.h"

class GnssEKF {
public:
    GnssEKF() : firstTime(true) {
    }

    void timeUpdate(const VariableSet &varSet, VariableDataMap *csData = nullptr);

    void measUpdate(EquSys &equSys, const Eigen::MatrixXd *fullWeight = nullptr);

    void createIndex(const VariableSet &varSet);

    void reparameterize(const VariableSet &newOrder, const Eigen::MatrixXd &T);

    [[nodiscard]] int indexOf(const Variable &v) const {
        const auto it = currentIndexData.find(v);
        return it == currentIndexData.end() ? -1 : it->second;
    }

    void updateVelocity(const Eigen::Vector3d &vel, const Eigen::Vector3d &sigma);

    [[nodiscard]] bool posteriorVelocity(const std::string &station, Eigen::Vector3d &vel,
                                         Eigen::Vector3d &sigma) const;

    static double getSolution(const Parameter &type,
                              const VariableSet &currentUnkSet,
                              const Eigen::VectorXd &stateVec);

    static bool tryGetSolution(const Parameter &type, const VariableSet &currentUnkSet,
                               const Eigen::VectorXd &stateVec, double &out);

    [[nodiscard]] double getSolution(const Variable &var) const;

    [[nodiscard]] double priorOf(const Variable &var) const;

    [[nodiscard]] Eigen::VectorXd priorInnovationStd(const EquSys &equSys) const;

    void forceValue(const Variable &var, const double v) { mForceValue[var] = v; }

    void savePrior() {
        savedXminus = kalmanFilter.Xminus;
        savedDp = kalmanFilter.Dp;
        hasSaved = true;
    }

    void configure(const double posSigma, const bool hasTropo, const double ambSigma = 1.0E-8, const double clockSigma = 0.0) {
        posProcSigma = posSigma;
        useTropo = hasTropo;
        ambProcSigma = ambSigma;
        clockProcSigma = clockSigma;
    }

    void reset() {
        firstTime = true;
        motionDt = 0.0;
        velocityCoupled = false;
        solution = Eigen::VectorXd();
        Q = Eigen::MatrixXd();
        postfitResidual = Eigen::VectorXd();
        currentUnkSet.clear();
        currentIndexData.clear();
        posIdx = {-1, -1, -1};
        velIdx = {-1, -1, -1};
        Xhat = Eigen::VectorXd();
        Dx = Eigen::MatrixXd();
        oldUnkSet.clear();
        oldIndexData.clear();
        kalmanFilter = SolverKalman{};
        savedXminus = Eigen::VectorXd();
        savedDp = Eigen::MatrixXd();
        hasSaved = false;
        mForceValue.clear();
    }

    /// 首历元引导：仅当尚处于 firstTime（无任何后验）时，把位置先验设为近似点。
    void setBootstrapPosition(const Eigen::Vector3d &p, const double posVar = 0.0) {
        if (currentUnkSet.empty()) return; // timeUpdate 尚未建索引时无效
        for (int k = 0; k < 3; ++k) {
            const int idx = posIdx[k];
            if (idx < 0) continue;
            Xhat(idx) = p[k];
            kalmanFilter.Xhat(idx) = p[k];
            kalmanFilter.Xminus(idx) = p[k];
            if (posVar > 0.0) {
                kalmanFilter.Dp(idx, idx) = posVar;
                kalmanFilter.Dx(idx, idx) = posVar;
                Dx(idx, idx) = posVar;
            }
        }
    }

    /// 逐分量首历元位置先验方差(m^2)
    void setBootstrapPosition(const Eigen::Vector3d &p, const Eigen::Vector3d &posVar) {
        if (currentUnkSet.empty()) return; // timeUpdate 尚未建索引时无效
        for (int k = 0; k < 3; ++k) {
            const int idx = posIdx[k];
            if (idx < 0) continue;
            Xhat(idx) = p[k];
            kalmanFilter.Xhat(idx) = p[k];
            kalmanFilter.Xminus(idx) = p[k];
            kalmanFilter.Dp(idx, idx) = posVar[k];
            kalmanFilter.Dx(idx, idx) = posVar[k];
            Dx(idx, idx) = posVar[k];
        }
    }


    void shiftState(const Variable &var, const double delta) {
        if (currentUnkSet.empty()) return;
        const int idx = indexOf(var);
        if (idx < 0) return;
        kalmanFilter.Xhat(idx) += delta;
        kalmanFilter.Xminus(idx) += delta;
    }

    /// 首历元钟差/ZTD 引导
    void setBootstrapStates(const std::map<Variable, double> &values,
                            const std::map<Variable, double> *vars = nullptr) {
        if (currentUnkSet.empty()) return;
        for (const auto &[var, value]: values) {
            if (const int idx = indexOf(var); idx >= 0) {
                kalmanFilter.Xhat(idx) = value;
                kalmanFilter.Xminus(idx) = value;
                double v = -1.0;
                if (vars) {
                    if (const auto vit = vars->find(var); vit != vars->end()) v = vit->second;
                }
                if (v > 0.0) {
                    kalmanFilter.Dp(idx, idx) = v;
                    kalmanFilter.Dx(idx, idx) = v;
                    Dx(idx, idx) = v;
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
            if (const int idx = indexOf(vamb); idx >= 0) {
                kalmanFilter.Xhat(idx) = value;
                kalmanFilter.Xminus(idx) = value;
            }
        }
    }

    virtual ~GnssEKF() = default;

    VariableSet currentUnkSet;
    VariableIntMap currentIndexData;

private:
    struct VarianceTable {
        std::array<double, Parameter::count> init;
        std::array<double, Parameter::count> proc;
    };

    /// 按当前 σ 配置一次列出全部参数类型的初始方差与过程噪声方差，漏配用哨兵显式失败
    [[nodiscard]] VarianceTable varianceTables() const;

    std::array<int, 3> posIdx = {-1, -1, -1};
    std::array<int, 3> velIdx = {-1, -1, -1};

    bool firstTime;
    bool useTropo = true;
    double posProcSigma = 0.0;

public:
    // 以下全部为 σ 口径：使用点平方后进 Dx/Dw 阵（原为方差口径，2026-11 统一转 σ）。
    double ambProcSigma = 1.0E-4;
    double ambInitSigma = 10.0;
    double clockProcSigma = 0.0;
    double ztdProcSigma = 1.0E-3;
    double posFloorSigma = 0.0;
    Eigen::Vector3d posFloorSigmaVec = Eigen::Vector3d(0.0, 0.0, 0.0);
    double ztdInitSigma = 0.2;
    double ztdInitValue = 0.0;
    double clockInitSigma = 100.0;
    double accProcSigma = 0.0; // 加速度过程 σ (m/s²)
    double velInitSigma = 10.0; // 速度状态初始 σ (m/s)
    double motionDt = 0.0; // 与上一历元的间隔 (s)，0 = 本历元不预测（首历元/断档）
    bool velocityCoupled = false; // 本历元位置是否被速度预测耦合（＝常速度模型生效），速度量测经此影响位置
    Eigen::VectorXd solution; // 后验状态向量 x
    Eigen::MatrixXd Q; // 后验协方差
    Eigen::VectorXd postfitResidual;

private:
    Eigen::VectorXd Xhat;
    Eigen::MatrixXd Dx;
    VariableSet oldUnkSet;
    VariableIntMap oldIndexData;

    SolverKalman kalmanFilter;

    Eigen::VectorXd savedXminus;
    Eigen::MatrixXd savedDp;
    bool hasSaved = false;

    std::map<Variable, double> mForceValue;

public:
    double codeBiasProcSigma = 1.0E-4;
    double codeBiasInitSigma = 1.0;
    double isbProcSigma = 0.0;
    double isbInitSigma = 0.0;

    double gloIfcbProcSigma = 1.0E-4;
    double gloIfcbInitSigma = 1.0;
};
