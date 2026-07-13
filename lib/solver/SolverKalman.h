// SolverKalman 把 EquSys 方程系统转为 Kalman Filter 矩阵形式求解。
// 可作为 SolverLSQ 的 Kalman 替代品。
#ifndef GNSSLAB_SOLVERKALMAN_H
#define GNSSLAB_SOLVERKALMAN_H

#include <Eigen/Eigen>
#include "GnssStruct.h"
#include "KalmanFilter.h"

class SolverKalman {
public:
    SolverKalman() : firstTime(true) {}

    /// 完整一步：TimeUpdate + MeasUpdate（适用每历元行一次）
    virtual void solve(EquSys &equSys, VariableDataMap &csData);

    /// 仅时间更新（预测到当前历元），不依赖观测方程
    void timeUpdate(const VariableSet &varSet, VariableDataMap *csData = nullptr);

    /// 仅测量更新（校正），使用 equSys 中的观测方程
    void measUpdate(EquSys &equSys);

    void createIndex(const VariableSet &varSet);

    double getSolution(const Parameter &type,
                       VariableSet &currentUnkSet,
                       const Eigen::VectorXd &stateVec);

    Eigen::VectorXd getState() { return solution; }
    Eigen::MatrixXd getCovMatrix() { return covMatrix; }
    Eigen::Vector3d getdxyz() const { return dxyz; }
    Eigen::VectorXd getPostfitResidual() const { return postfitResidual; }

    /// 强制下次 timeUpdate 重新初始化状态（抛弃已累积的偏置）。
    /// 当检测到位置突跳/回退后调用，避免滤波器“卡死”在陈旧状态上。
    void reset() { firstTime = true; solution = Eigen::VectorXd(); covMatrix = Eigen::MatrixXd(); }

    virtual ~SolverKalman() {}

    VariableSet currentUnkSet;
    VariableIntMap currentIndexData;

private:
    bool firstTime;

    Eigen::VectorXd solution, xhat;
    Eigen::MatrixXd covMatrix, P;
    Eigen::VectorXd postfitResidual;
    Eigen::Vector3d dxyz;

    VariableSet oldUnkSet;
    VariableIntMap oldIndexData;

    KalmanFilter kalmanFilter;
};

#endif
