/**
 * SPPCodePhase: IF 组合 (消电离层) 伪距+载波相位 SPP。
 * 每颗卫星使用 C_IF=α·C1+β·C2 和 L_IF=α·L1+β·L2 两个观测量，
 * 估计位置、钟差、IF 浮点模糊度。
 */
#pragma once

#include "SPPIFCode.h"
#include "SolverKalman.h"
#include <map>
#include <string>

class SPPCodePhase : public SPPIFCode {
public:
    SPPCodePhase() {}

    void solve(ObsData &obsData) override;
    void linearize(ObsData &obsData, int iter) override;
    void getResult() override;
    void preprocess(ObsData &obsData) override { satRejected.clear(); setEphemeris(obsData.satEphemerisData); }
    void computeSatPos(ObsData &obsData) override;

    void enableKalman(bool on) { useKalman_ = on; }
    static void stripObsType(ObsData &obsData);
    static void normalizePhase(ObsData &obsData);

protected:
    double sigCode = 0.3;
    double sigPhase = 0.03;  // 降低载波相位权重，避免单历元浮点模糊度引起的病态矩阵

private:
    bool useKalman_ = false;
    SolverKalman solverKalman_;
    void solveLSQ(ObsData &obsData);
    void solveKalman(ObsData &obsData);
    void computeDOP();

    /// 上一历元成功解算的位置，用于冷启动（无先验坐标时）的初值传递
    XYZ lastGoodXyz_{0, 0, 0};

    /// 用干净的 IF-code 单历元解作为可靠参考（位置+钟差），供初值热启动与粗差回退
    /// 传入未经 strip/normalize 的原始 obsData 副本；成功返回 true
    bool computeCodeReference(const ObsData &rawObs, XYZ &outXyz, std::map<char, double> &outCdt);

    /// 浮点解偏离 IF-code 参考超过此阈值(m)判为发散(周跳等)，回退到 IF-code
    static constexpr double FALLBACK_THRESH = 8.0;
};
