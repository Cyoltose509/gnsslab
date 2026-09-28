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

    void forceValue(const Variable &var, double v) { mForceValue[var] = v; }

    /// 保存“量测更新前”的先验(xhatminus/Pminus)。
    /// 供 rtkpost 式迭代抗差使用：检出粗差后必须回到**同一**先验重解，
    /// 否则会把好的观测重复计入（状态被更新两次）。
    void savePrior() {
        savedXhatminus = kalmanFilter.xhatminus;
        savedPminus = kalmanFilter.Pminus;
        hasSaved = true;
    }

    /// 恢复到 savePrior() 时刻的先验，随后可再次 measUpdate() 重解。
    bool restorePrior() {
        if (!hasSaved) return false;
        kalmanFilter.xhatminus = savedXhatminus;
        kalmanFilter.Pminus = savedPminus;
        kalmanFilter.xhat = savedXhatminus;
        kalmanFilter.P = savedPminus;
        solution = kalmanFilter.xhat;
        covMatrix = kalmanFilter.P;
        return true;
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

    /// 模糊度引导：把状态强制置为码减相位初值。
    /// 若给出 csData，则**只**引导本历元被标记重置(>=0.5，见 resetAmbiguity 写入的 1.0/2.0)的模糊度；
    /// 已收敛的模糊度必须交给滤波自行演化——否则每历元把它们全部写回初值，
    /// measUpdate refined 出的估计会被覆盖，模糊度永远累积不了信息（PPP 收敛被拖到小时级）。
    void setBootstrapAmbiguity(const std::string &station, const std::map<SatID, double> &amb,
                               const VariableDataMap *csData = nullptr) {
        if (currentUnkSet.empty()) return;
        for (const auto &[sat, value]: amb) {
            const Variable vamb = makeAmbiguityVar(station, sat, "IF");
            if (csData) {
                const auto itc = csData->find(vamb);
                if (itc == csData->end() || itc->second < 0.5) continue; // 仅新弧段/周跳重置
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

private:
    [[nodiscard]] double initVariance(const Parameter &p) const;

    bool firstTime;
    bool useTropo = true;
    double posProcNoise = 0.0;
public:
    // 以下成员原由透传 getter/setter 暴露，按项目约定改为公有成员。
    // 声明顺序与旧布局完全一致，仅翻转访问符，避免改变 kalmanFilter 前成员偏移导致 ABI 不匹配。
    double ambProcNoise = 1.0E-8;
    double clockProcNoise = 0.0;
    double ztdProcNoise = 1.0E-6;
    double ztdInitVar = 0.04;
    double ztdInitValue = 0.0;
    double clockInitVar = 1.0E4;
    Eigen::VectorXd solution;        // 后验状态向量 x
    Eigen::MatrixXd covMatrix;       // 后验协方差 P
    Eigen::VectorXd postfitResidual;
    Eigen::Vector3d dxyz;
private:
    Eigen::VectorXd xhat;
    Eigen::MatrixXd P;
    VariableSet oldUnkSet;
    VariableIntMap oldIndexData;

    SolverKalman kalmanFilter;

    // 迭代抗差用：量测更新前的先验快照。
    // 必须放**类末尾**：部分重链时 lib/gnss.lib 里的 GnssEKF.cpp 仍是旧布局，
    // 新增成员若插在 kalmanFilter 之前会改变其偏移，导致 ABI 不匹配。
    Eigen::VectorXd savedXhatminus;
    Eigen::MatrixXd savedPminus;
    bool hasSaved = false;

    // 周跳软重置用：slip 重置时 PPP 把"跳变后正确模糊度(prefit)"钉进此表，timeUpdate 取用。
    // 放类末尾(同 saved*)，避免改变 kalmanFilter 前成员偏移导致 ABI 不匹配。
    std::map<Variable, double> mForceValue;
};
