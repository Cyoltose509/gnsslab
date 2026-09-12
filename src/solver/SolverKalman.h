#pragma once


#include <Eigen/Eigen>

class SolverKalman {
public:
    SolverKalman() = default;

    SolverKalman(const Eigen::VectorXd &initialState,
                 const Eigen::MatrixXd &initialErrorCovariance)
            : xhat(initialState), P(initialErrorCovariance) {
        xhatminus = Eigen::VectorXd::Zero(initialState.size());
        Pminus = Eigen::MatrixXd::Zero(initialErrorCovariance.rows(), initialErrorCovariance.cols());
    }

    virtual void reset(const Eigen::VectorXd &initialState,
                 const Eigen::MatrixXd &initialErrorCovariance);

    virtual int compute(const Eigen::MatrixXd &phiMatrix,
                        const Eigen::MatrixXd &qMatrix,
                        const Eigen::VectorXd &mVector,
                        const Eigen::MatrixXd &hMatrix,
                        const Eigen::MatrixXd &wMatrix) ;

    virtual int timeUpdate(const Eigen::MatrixXd &phiMatrix,
                           const Eigen::MatrixXd &qMatrix)  {
        return predict(phiMatrix, xhat, qMatrix);
    }

    virtual int measUpdate(const Eigen::VectorXd &mVector,
                           const Eigen::MatrixXd &hMatrix,
                           const Eigen::MatrixXd &wMatrix)  {
        return correct(mVector, hMatrix, wMatrix);
    }

    virtual int measUpdate(const Eigen::VectorXd &mVector,
                           const Eigen::MatrixXd &hMatrix,
                           const Eigen::MatrixXd &wMatrix,
                           const Eigen::VectorXd &mVectorAug,
                           const Eigen::MatrixXd &hMatrixAug,
                           const Eigen::MatrixXd &wMatrixAug)  {
        const int numMeas = static_cast<int>(mVector.size());
        const int numUnks = static_cast<int>(hMatrix.cols());
        const int numAug = static_cast<int>(mVectorAug.size());
        const int numMeasExt = numMeas + numAug;

        Eigen::VectorXd mVectorExt = Eigen::VectorXd::Zero(numMeasExt);
        mVectorExt.head(numMeas) = mVector;
        mVectorExt.tail(numAug) = mVectorAug;

        Eigen::MatrixXd hMatrixExt = Eigen::MatrixXd::Zero(numMeasExt, numUnks);
        hMatrixExt.block(0, 0, numMeas, numUnks) = hMatrix;
        hMatrixExt.block(numMeas, 0, numAug, numUnks) = hMatrixAug;

        Eigen::MatrixXd wMatrixExt = Eigen::MatrixXd::Zero(numMeasExt, numMeasExt);
        wMatrixExt.block(0, 0, numMeas, numMeas) = wMatrix;
        wMatrixExt.block(numMeas, numMeas, numAug, numAug) = wMatrixAug;

        return correct(mVectorExt, hMatrixExt, wMatrixExt);
    }

    virtual ~SolverKalman() = default;

    Eigen::VectorXd xhat;
    Eigen::MatrixXd P;
    Eigen::VectorXd xhatminus;
    Eigen::MatrixXd Pminus;
    Eigen::VectorXd postfitResidual;

private:
    virtual int predict(const Eigen::MatrixXd &phiMatrix,
                        const Eigen::VectorXd &previousState,
                        const Eigen::MatrixXd &qMatrix) ;

    virtual int predict(const Eigen::MatrixXd &phiMatrix,
                        const Eigen::VectorXd &previousState,
                        const Eigen::MatrixXd &controlMatrix,
                        const Eigen::VectorXd &controlInput,
                        const Eigen::MatrixXd &qMatrix) ;

    virtual int correct(const Eigen::VectorXd &mVector,
                        const Eigen::MatrixXd &hMatrix,
                        const Eigen::MatrixXd &wMatrix) ;
};


