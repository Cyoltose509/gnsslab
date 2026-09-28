#pragma once

#include <vector>
#include <set>
#include "FreqCombo.h"

class CycleSlip {
public:
    enum class Verdict { None, Slip, Outlier, Gap };

    void configure(const FreqCombo &fcIn);

    void reset();

    Verdict feed(double L1, double L2, double C1, double C2, bool valid,
                 double osbP1 = 0, double osbP2 = 0, double osbL1 = 0, double osbL2 = 0);

    static void detectMW(const std::vector<double> &MW, const std::vector<char> &ok,
                         std::vector<char> &slip, std::vector<char> &outlier);

    static void detectGF(const std::vector<double> &LGF, const std::vector<double> &PGF,
                         const std::vector<char> &ok, double lam1, double lam2,
                         std::vector<char> &slip, std::vector<char> &artifact);

    void setGfThreshold(double thr) { gfThres = thr; }
    double gfThreshold() const { return gfThres; }

    bool gfWideLaneSlip(double L1, double L2, double thr = 0.12);

    bool lastMwSlip() const { return mLastMwSlip; }
    bool lastGfSlip() const { return mLastGfSlip; }
    bool lastGfArtifact() const { return mLastGfArtifact; }

private:
    FreqCombo fc;
    double lam1 = 0, lam2 = 0;
    bool mLastMwSlip = false;
    bool mLastGfSlip = false;
    bool mLastGfArtifact = false; // 本次 feed 的 GF 跳变是否为伪影(码 GF 同跳)

    double mwMean = 0, mwVar = 0;
    int mwCnt = 0;

    double gfPrev = 0; // 上一历元 GF(米)
    bool gfHasPrev = false;
    double gfThres = 0.0;

    double mGfWlPrev = 0;
    bool mGfWlHasPrev = false; // 宽巷即时 GF 检测的上一历元值

    std::vector<double> gfLbuf; // 相位 GF = L2-L1 (米)，当前弧段逐历元
    std::vector<double> gfPbuf; // 伪距 GF = P2-P1 (米)，用于多项式拟合消去电离层漂移
    std::vector<char> gfObuf; // 观测有效标志
    std::set<int> gfSlipSet; // 弧段内相对历元号被判为真实周跳的集合
    std::set<int> gfArtifactSet; // 弧段内相对历元号被判为伪影(码 GF 同跳)的集合

    bool pendActive = false;
    double pendMw = 0, pendDev = 0;
    double mwMeanB = 0, mwVarB = 0;
    int mwCntB = 0;
};

/// 历元级 GF 共模/伪影抑制簿记
class CycleSlipEpoch {
public:
    /// 记录一颗星本历元的 GF 判定：isArtifact=true 记伪影
    void record(const SatID &sat, const bool isArtifact = false) {
        (isArtifact ? gfArtifact : gfSats).insert(sat);
    }

    /// 统一裁决。返回需重置的真实孤立周跳星
    SatIDSet decide(const int kCommonMode, SatIDSet &excludeOut) {
        excludeOut.clear();
        if (const int total = static_cast<int>(gfSats.size() + gfArtifact.size()); kCommonMode > 0 && total >= kCommonMode) {
            excludeOut = gfSats;
            excludeOut.insert(gfArtifact.begin(), gfArtifact.end());
            return SatIDSet();
        }
        // 孤立事件：伪影星排除
        excludeOut = gfArtifact;
        return gfSats;
    }

    void clear() {
        gfSats.clear();
        gfArtifact.clear();
    }

private:
    SatIDSet gfSats;
    SatIDSet gfArtifact;
};
