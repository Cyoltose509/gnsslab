#pragma once

#include <vector>

#include <Eigen/Core>

#include "core/SatID.h"

/// 逐星后验残差
struct SatResidual {
    std::vector<SatID> sats;
    std::vector<double> values;

    void set(const SatID &id, const double res) {
        for (std::size_t i = 0; i < sats.size(); ++i) {
            if (sats[i] == id) {
                values[i] = res;
                return;
            }
        }
        sats.push_back(id);
        values.push_back(res);
    }

    void release() {
        std::vector<SatID>().swap(sats);
        std::vector<double>().swap(values);
    }

    [[nodiscard]] std::size_t size() const { return sats.size(); }

    [[nodiscard]] bool empty() const { return sats.empty(); }
};


struct Result {
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzFixed = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzFloat = Eigen::Vector3d::Zero();
    Eigen::Vector3d blh = Eigen::Vector3d::Zero();
    Eigen::Vector3d blhFixed = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d sigmaXYZ = Eigen::Vector3d::Zero();
    Eigen::Vector3d sigmaVel = Eigen::Vector3d::Zero();
    double sigmaP = 0;
    double sigmaPFloat = -1; // 浮点解 σ
    double sigmaPFixed = -1; // 固定解 σ
    double sigmaV = 0;
    double pdop = 0, gdop = 0, hdop = 0, vdop = 0, tdop = 0;
    int numSats = 0;
    double ratio = 0;
    SatResidual postRes;      // 伪距(码)后验残差, m
    SatResidual postResPhase; // 相位后验残差, m (SPP 无相位, 恒空)
    unsigned char stat = 0; // 解状态: 0=无效, 1=Fixed, 2=Float

    void reset() {
        xyz.setZero();
        xyzFixed.setZero();
        xyzFloat.setZero();
        blh.setZero();
        blhFixed.setZero();
        vel.setZero();
        sigmaXYZ.setZero();
        sigmaVel.setZero();
        pdop = gdop = hdop = vdop = tdop = 0;
        sigmaP = sigmaV = 0;
        sigmaPFloat = -1;
        sigmaPFixed = -1;
        numSats = 0;
        ratio = 0;
        postRes.release();
        postResPhase.release();
        stat = 0;
    }
};
