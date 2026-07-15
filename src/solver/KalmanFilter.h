/**
 * @file KalmanFilter.h
 * Standard linear Kalman filter: TimeUpdate (predict) + MeasUpdate (correct).
 * Based on Welch & Bishop: http://www.cs.unc.edu/~welch/kalman/kalmanIntro.html
 */
#ifndef KalmanFilter_H
#define KalmanFilter_H

#include "Exception.h"
#include <Eigen/Eigen>

class KalmanFilter {
public:
    KalmanFilter() {}

    KalmanFilter(const Eigen::VectorXd &initialState,
                 const Eigen::MatrixXd &initialErrorCovariance)
            : xhat(initialState), P(initialErrorCovariance) {
        xhatminus = Eigen::VectorXd::Zero(initialState.size());
        Pminus = Eigen::MatrixXd::Zero(initialErrorCovariance.rows(), initialErrorCovariance.cols());
    }

    virtual void Reset(const Eigen::VectorXd &initialState,
                       const Eigen::MatrixXd &initialErrorCovariance);

    virtual int Compute(const Eigen::MatrixXd &phiMatrix,
                        const Eigen::MatrixXd &qMatrix,
                        const Eigen::VectorXd &mVector,
                        const Eigen::MatrixXd &hMatrix,
                        const Eigen::MatrixXd &wMatrix) ;

    virtual int TimeUpdate(const Eigen::MatrixXd &phiMatrix,
                           const Eigen::MatrixXd &qMatrix)  {
        return Predict(phiMatrix, xhat, qMatrix);
    }

    virtual int MeasUpdate(const Eigen::VectorXd &mVector,
                           const Eigen::MatrixXd &hMatrix,
                           const Eigen::MatrixXd &wMatrix)  {
        return Correct(mVector, hMatrix, wMatrix);
    }

    virtual int MeasUpdate(const Eigen::VectorXd &mVector,
                           const Eigen::MatrixXd &hMatrix,
                           const Eigen::MatrixXd &wMatrix,
                           const Eigen::VectorXd &mVectorAug,
                           const Eigen::MatrixXd &hMatrixAug,
                           const Eigen::MatrixXd &wMatrixAug)  {
        int numMeas = static_cast<int>(mVector.size());
        int numUnks = static_cast<int>(hMatrix.cols());
        int numAug = static_cast<int>(mVectorAug.size());
        int numMeasExt = numMeas + numAug;

        Eigen::VectorXd mVectorExt = Eigen::VectorXd::Zero(numMeasExt);
        mVectorExt.head(numMeas) = mVector;
        mVectorExt.tail(numAug) = mVectorAug;

        Eigen::MatrixXd hMatrixExt = Eigen::MatrixXd::Zero(numMeasExt, numUnks);
        hMatrixExt.block(0, 0, numMeas, numUnks) = hMatrix;
        hMatrixExt.block(numMeas, 0, numAug, numUnks) = hMatrixAug;

        Eigen::MatrixXd wMatrixExt = Eigen::MatrixXd::Zero(numMeasExt, numMeasExt);
        wMatrixExt.block(0, 0, numMeas, numMeas) = wMatrix;
        wMatrixExt.block(numMeas, numMeas, numAug, numAug) = wMatrixAug;

        return Correct(mVectorExt, hMatrixExt, wMatrixExt);
    }

    virtual ~KalmanFilter() {}

    Eigen::VectorXd xhat;
    Eigen::MatrixXd P;
    Eigen::VectorXd xhatminus;
    Eigen::MatrixXd Pminus;
    Eigen::VectorXd postfitResidual;

private:
    virtual int Predict(const Eigen::MatrixXd &phiMatrix,
                        const Eigen::VectorXd &previousState,
                        const Eigen::MatrixXd &qMatrix) ;

    virtual int Predict(const Eigen::MatrixXd &phiMatrix,
                        const Eigen::VectorXd &previousState,
                        const Eigen::MatrixXd &controlMatrix,
                        const Eigen::VectorXd &controlInput,
                        const Eigen::MatrixXd &qMatrix) ;

    virtual int Correct(const Eigen::VectorXd &mVector,
                        const Eigen::MatrixXd &hMatrix,
                        const Eigen::MatrixXd &wMatrix) ;
};

#endif
