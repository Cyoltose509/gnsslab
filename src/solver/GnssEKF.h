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

    [[nodiscard]] Eigen::VectorXd getState() const { return solution; }
    [[nodiscard]] Eigen::MatrixXd getCovMatrix() { return covMatrix; }

    Eigen::VectorXd getState(const std::map<int, Variable> &idxToVar) const;

    Eigen::MatrixXd getCovMatrix(const std::map<int, Variable> &idxToVar) const;

    [[nodiscard]] Eigen::VectorXd getPredState(const std::map<int, Variable> &idxToVar) const;

    [[nodiscard]] Eigen::MatrixXd getPredCov(const std::map<int, Variable> &idxToVar) const;

    [[nodiscard]] double getSolution(const Variable &var) const;

    [[nodiscard]] Eigen::Vector3d getdxyz() const { return dxyz; }
    [[nodiscard]] Eigen::VectorXd getPostfitResidual() const { return postfitResidual; }


    void configure(const double posNoise, const bool hasTropo,
                   const double ambNoise = 1.0E-8,
                   const double clockNoise = 0.0) {
        posProcNoise = posNoise;
        useTropo = hasTropo;
        ambProcNoise = ambNoise;
        clockProcNoise = clockNoise;
    }

    /// 单独设置模糊度过程噪声
    void setAmbProcNoise(const double ambNoise) { this->ambProcNoise = ambNoise; }

    /// 单独设置接收机钟差过程噪声
    void setClockProcNoise(const double clockNoise) { this->clockProcNoise = clockNoise; }
    /// 单独设置接收机钟差初始方差(m²)
    void setClockInitVar(const double v) { this->clockInitVar = v; }

    /// 单独设置对流层 ZTD 过程噪声
    void setZtdProcNoise(const double q) { this->ztdProcNoise = q; }
    void setZtdInitVar(const double v) { this->ztdInitVar = v; }
    /// 设置 ZTD 初始均值(m)
    void setZtdInitValue(const double v) { this->ztdInitValue = v; }

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

    /// 首历元引导
    void setBootstrapAmbiguity(const std::string &station, const std::map<SatID, double> &amb) {
        if (currentUnkSet.empty()) return;
        for (const auto &[sat, value]: amb) {
            const Variable vamb(station, sat, Parameter::ambiguity,
                                ObsID(std::string(1, sat.system), "IF"));
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
    double ambProcNoise = 1.0E-8;
    double clockProcNoise = 0.0;
    double ztdProcNoise = 1.0E-6;
    double ztdInitVar = 0.04;
    double ztdInitValue = 0.0;
    double clockInitVar = 1.0E4;

    Eigen::VectorXd solution, xhat;
    Eigen::MatrixXd covMatrix, P;
    Eigen::VectorXd postfitResidual;
    Eigen::Vector3d dxyz;

    VariableSet oldUnkSet;
    VariableIntMap oldIndexData;

    SolverKalman kalmanFilter;
};
