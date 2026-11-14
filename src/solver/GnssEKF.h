#pragma once



#include "core/equ/EquStruct.h"
#include "SolverKalman.h"

class GnssEKF {
public:
    GnssEKF() : firstTime(true) {
    }

    virtual void solve(EquSys &equSys, VariableDataMap &csData);

    void timeUpdate(const VariableSet &varSet, VariableDataMap *csData = nullptr);

    /// 量测更新。`fullWeight` 给非空时按满阵权（双差站间相关）解，否则用方程标量权取对角。
    void measUpdate(EquSys &equSys, const Eigen::MatrixXd *fullWeight = nullptr);

    void createIndex(const VariableSet &varSet);

    /// 状态线性重参数化：x ← T·x，P ← T·P·Tᵀ，变量顺序同时切到 newOrder。
    /// T 的列按重参数化前的 currentUnkSet 排，行按 newOrder 排；换模糊度基准（参考星切换）用它。
    void reparameterize(const VariableSet &newOrder, const Eigen::MatrixXd &T);

    /// 状态下标，变量不在本历元状态集里返回 -1。
    [[nodiscard]] int indexOf(const Variable &v) const {
        const auto it = currentIndexData.find(v);
        return it == currentIndexData.end() ? -1 : it->second;
    }

    /// 多普勒速度量测更新：只挂速度三态，位置经协方差耦合微调。
    /// 仅在状态集含 dVX/dVY/dVZ 时生效（SPP 滤波档），PPP 无速度态直接返回。
    void updateVelocity(const Eigen::Vector3d &vel, const Eigen::Vector3d &sigma);

    /// 后验速度，状态集无速度态时返回 false。
    [[nodiscard]] bool posteriorVelocity(const std::string &station, Eigen::Vector3d &vel,
                                         Eigen::Vector3d &sigma) const;

    static double getSolution(const Parameter &type,
                              const VariableSet &currentUnkSet,
                              const Eigen::VectorXd &stateVec);

    static bool tryGetSolution(const Parameter &type, const VariableSet &currentUnkSet,
                               const Eigen::VectorXd &stateVec, double &out);

    [[nodiscard]] double getSolution(const Variable &var) const;

    /// 本历元先验 x⁻（时间更新后、量测更新前）。线性化点与 prefit 的口径要按它对齐。
    [[nodiscard]] double priorOf(const Variable &var) const;

    /// 每条方程先验新息 σ = sqrt(diag(H·P⁻·Hᵀ))，按 equSys.obsEquData 顺序；不含观测噪声。
    /// 抗差门限必须按它定：运动时先验位置本身差几十厘米，新息大是预测误差，不是粗差。
    [[nodiscard]] Eigen::VectorXd priorInnovationStd(const EquSys &equSys) const;

    void forceValue(const Variable &var, const double v) { mForceValue[var] = v; }

    /// 保存“量测更新前”的先验(xhatminus/Pminus)。
    /// 供 rtkpost 式迭代抗差使用：检出粗差后必须回到**同一**先验重解，
    /// 否则会把好的观测重复计入（状态被更新两次）。
    void savePrior() {
        savedXhatminus = kalmanFilter.xhatminus;
        savedPminus = kalmanFilter.Pminus;
        hasSaved = true;
    }

    void configure(const double posSigma, const bool hasTropo,
                   const double ambSigma = 1.0E-8,
                   const double clockSigma = 0.0) {
        posProcSigma = posSigma;
        useTropo = hasTropo;
        ambProcSigma = ambSigma;
        clockProcSigma = clockSigma;
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


    void shiftState(const Variable &var, const double delta) {        if (currentUnkSet.empty()) return;
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

    virtual ~GnssEKF() = default;

    VariableSet currentUnkSet;
    VariableIntMap currentIndexData;
    std::vector<Variable> idxToVar_; // 与 currentIndexData 同步：idx -> var（timeUpdate 按整数索引遍历，免反查）

private:
    [[nodiscard]] double initVariance(const Parameter &p) const;

    bool firstTime;
    bool useTropo = true;
    double posProcSigma = 0.0;

public:
    // 以下全部为 σ 口径：使用点平方后进 P/Q 阵（原为方差口径，2026-11 统一转 σ）。
    double ambProcSigma = 1.0E-4;
    double ambInitSigma = 10.0;
    double clockProcSigma = 0.0;
    double ztdProcSigma = 1.0E-3;
    double posFloorSigma = 0.0;
    Eigen::Vector3d posFloorSigmaVec = Eigen::Vector3d(0.0, 0.0, 0.0);
    double ztdInitSigma = 0.2;
    double ztdInitValue = 0.0;
    double clockInitSigma = 100.0;
    /// 常速度预测：accProcSigma>0 且状态含速度三态时，位置按 p+=v·dt 预测、速度增量按 a·dt；
    /// 此时位置过程噪声只由加速度项给（标准 CV，Q 的位置-速度块含 ½a²Δt³ 交叉项），
    /// posProcSigma 只在无速度态时生效。
    double accProcSigma = 0.0;  // 加速度过程 σ (m/s²)
    double velInitSigma = 10.0; // 速度状态初始 σ (m/s)
    double motionDt = 0.0;      // 与上一历元的间隔 (s)，0 = 本历元不预测（首历元/断档）
    bool velocityCoupled = false; // 本历元位置是否被速度预测耦合（＝常速度模型生效），速度量测经此影响位置
    Eigen::VectorXd solution; // 后验状态向量 x
    Eigen::MatrixXd covMatrix; // 后验协方差 P
    Eigen::VectorXd postfitResidual;

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

    double codeBiasProcSigma = 1.0E-4;
    double codeBiasInitSigma = 1.0;
    double isbProcSigma = 0.0;
    double isbInitSigma = 0.0;

    double gloIfcbProcSigma = 1.0E-4;
    double gloIfcbInitSigma = 1.0;
};
