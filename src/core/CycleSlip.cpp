#include "CycleSlip.h"
#include "Const.h"
#include "MathUtils.h"

void CycleSlip::configure(const FreqCombo &fcIn) {
    fc = fcIn;
    lam1 = fc.f1 > 0 ? C_MPS / fc.f1 : fc.lambda1;
    lam2 = fc.f2 > 0 ? C_MPS / fc.f2 : fc.lambda2;
}

void CycleSlip::reset() {
    mwMean = 0; mwVar = 0; mwCnt = 0;
    pendActive = false;
    gfPrev = 0; gfHasPrev = false; // GF 弧段断开，下一历元重新起算
    mGfWlPrev = 0; mGfWlHasPrev = false; // 宽巷即时 GF 检测断开
}

CycleSlip::Verdict CycleSlip::feed(const double L1, const double L2, const double C1, const double C2, const bool valid,
                                  const  double osbP1, const double osbP2, const double osbL1, const double osbL2) {
    if (!valid) {
        // 弧段结束：在完整弧段上跑一次 detectGF(去电离层漂移的相位 GF 跳变检测)，再清空缓冲
        if (gfThres > 0.0 && gfLbuf.size() >= 6) {
            std::vector<char> slip(gfLbuf.size(), 0), artifact(gfLbuf.size(), 0);
            detectGF(gfLbuf, gfPbuf, gfObuf, lam1, lam2, slip, artifact);
            for (size_t k = 0; k < slip.size(); ++k) {
                if (slip[k]) gfSlipSet.insert(static_cast<int>(k));
                if (artifact[k]) gfArtifactSet.insert(static_cast<int>(k));
            }
        }
        gfLbuf.clear(); gfPbuf.clear(); gfObuf.clear(); gfSlipSet.clear(); gfArtifactSet.clear();
        reset();
        return Verdict::Gap;
    }
    if (lam1 <= 0 || lam2 <= 0) return Verdict::None;

    const double mw = fc.MW_meter(C1, C2, L1, L2, osbP1, osbP2, osbL1, osbL2); // MW (米)

    // 弧段缓冲 GF(几何无关)组合：相位 GF=L2-L1 消几何与钟差，伪距 GF=P2-P1 含电离层漂移。
    // detectGF 用 rtkpost 式「相邻历元相位 GF 阈值」(|LGF(k)-LGF(k-1)|>thr)抓跳变(电离层漂移仅~0.05m/历元，
    //   远小于单周跳0.054m，故3*(lam2-lam1)阈值即可)，并同查伪距 GF 一致性(|ΔPGF|)区分真实周跳(只动相位)
    //   与接收机/格式伪影(相位+码同跳，须排除而非重置)，细节见 detectGF。
    const double gfL = fc.GF_phase(L1, L2); // = L2 - L1 (米)
    const double gfP = fc.GF_code(C1, C2);  // = P2 - P1 (米)
    const int arcIdx = static_cast<int>(gfLbuf.size());
    gfLbuf.push_back(gfL);
    gfPbuf.push_back(gfP);
    gfObuf.push_back(1);

    if (gfThres > 0.0 && gfLbuf.size() >= 6) {
        std::vector<char> slip(gfLbuf.size(), 0), artifact(gfLbuf.size(), 0);
        detectGF(gfLbuf, gfPbuf, gfObuf, lam1, lam2, slip, artifact);
        for (size_t k = 0; k < slip.size(); ++k) {
            if (slip[k]) gfSlipSet.insert(static_cast<int>(k));
            if (artifact[k]) gfArtifactSet.insert(static_cast<int>(k));
        }
    }

    auto v = Verdict::None;
    mLastMwSlip = false;
    mLastGfSlip = false;

    if (pendActive) {
        const double meanK = (mwCntB * mwMeanB + pendMw) / (mwCntB + 1.0);
        const double varK  = (mwCntB * mwVarB + pendDev * pendDev) / (mwCntB + 1.0);
        const double devNext = mw - meanK;
        const bool slip = varK > 0.0 && std::fabs(devNext) >= 4.0 * std::sqrt(varK)
                         && std::fabs(mw - pendMw) <= 1.0;
        if (slip) {
            mwMean = mw; mwVar = 0; mwCnt = 0;
            pendActive = false;
            mLastMwSlip = true;
            v = Verdict::Slip;
        } else {
            mwMean = meanK; mwVar = varK; mwCnt++;
            pendActive = false;
        }
    }

    {
        if (const double dev = mw - mwMean; mwCnt >= 5 && mwVar > 0.0 && std::fabs(dev) >= 4.0 * std::sqrt(mwVar)) {
            pendActive = true;
            pendMw = mw;
            pendDev = dev;
            mwMeanB = mwMean;
            mwVarB  = mwVar;
            mwCntB  = mwCnt;
        } else {
            mwMean = (mwCnt * mwMean + mw) / (mwCnt + 1.0);
            mwVar  = (mwCnt * mwVar + dev * dev) / (mwCnt + 1.0);
            mwCnt++;
        }
    }

    // GF 判定的周跳：detectGF 在跳变后一历元才标记(需 r[k+1])，故查 arcIdx-1。
    // 命中即重置 MW 统计并置 Slip，使 PPP 重置该星模糊度(rtkpost 同机理的 udbias)。
    if (arcIdx >= 1 && gfThres > 0.0 && gfSlipSet.count(arcIdx - 1)) {
        mwMean = mw; mwVar = 0; mwCnt = 0;
        pendActive = false;
        mLastGfSlip = true;
        mLastGfArtifact = gfArtifactSet.count(arcIdx - 1) > 0;
        v = Verdict::Slip;
    }

    return v;
}

void CycleSlip::detectMW(const std::vector<double> &MW, const std::vector<char> &ok,
                         std::vector<char> &slip, std::vector<char> &outlier) {
    const int n = static_cast<int>(MW.size());
    slip.assign(n, 0);
    outlier.assign(n, 0);
    if (n < 2) return;
    double mean = MW[0], var = 0.0;
    int cnt = 1;
    for (int k = 1; k < n; k++) {
        if (!ok[k]) {
            mean = MW[k];
            var = 0.0;
            cnt = 1;
            continue;
        }
        const double dev = MW[k] - mean;
        if (var > 0.0 && std::fabs(dev) >= 4.0 * std::sqrt(var) && k + 1 < n && ok[k + 1]) {
            const double meanK = (cnt * mean + MW[k]) / (cnt + 1);
            const double varK = (cnt * var + dev * dev) / (cnt + 1);
            if (const double devNext = MW[k + 1] - meanK; !(varK > 0.0 && std::fabs(devNext) >= 4.0 * std::sqrt(varK)) || std::fabs(MW[k + 1] - MW[k]) > 1.0) {
                outlier[k] = 1; // ti+1 不超限，或两者均超限但 |ΔMW|>1m → 粗差
            } else {
                slip[k] = 1; // 两者均超限且 ΔMW≤1m → 周跳，从 k 起新弧段
                mean = MW[k];
                var = 0.0;
                cnt = 1;
                continue;
            }
        }
        mean = (cnt * mean + MW[k]) / (cnt + 1);
        var = (cnt * var + dev * dev) / (cnt + 1);
        cnt++;
    }
}

void CycleSlip::detectGF(const std::vector<double> &LGF, const std::vector<double> &PGF,
                         const std::vector<char> &ok, const double lam1, const double lam2,
                         std::vector<char> &slip, std::vector<char> &artifact) {
    const int n = static_cast<int>(LGF.size());
    slip.assign(n, 0); artifact.assign(n, 0);
    if (n < 2) return;
    // 几何无关相位组合 LGF = L2 - L1 (米)。一个共同周跳使两频率同跳 N 周 →
    //   LGF 阶跃 (lam2 - lam1)*N 米；该组合已消去几何/钟差/对流层，纯净(只剩模糊度+电离层)。
    // 电离层漂移每历元仅 ~0.05m(本数据)，远小于单周跳 0.054m。故 rtkpost 式相邻历元阈值即可：
    //   |LGF(k)-LGF(k-1)| > thr  → 相位跳变。阈值默认 3*(lam2-lam1)≈0.16m(>电离层漂移、抓≥3周)，
    //   避免逐历元电离层漂移(0.05m)误触发；env PPP_GF_THR(米)可覆盖做扫描。
    // 真实周跳 vs 接收机/格式伪影：真实周跳只动相位、伪距(PGF)不变；而 .rnx 解析伪影/钟跳会
    // 同时污染相位与伪距(码 GF 也跳)。故对相位跳变同时查同历元 |ΔPGF|：
    //   |ΔPGF| ≤ PPP_GF_CODE_THR(默认 0.10m，真实周跳=0、码噪通常<0.1m) → 真实周跳(slip)；
    //   |ΔPGF| >  阈值 → 伪影(artifact)：该星相位被污染、非真实周跳，PPP 应排除其方程而非重置
    //   (重置会把伪影阶跃灌进模糊度→尾部 E 常偏；rtkpost 对干净 .26o 不报此类跳)。
    const double step = lam2 - lam1;                       // 单周跳阶跃(米)
    const double thr  = 3.0 * step;                         // 相位跳变阈值(默认 3×单周跳)
    for (int k = 1; k < n; ++k) {
        if (!ok[k] || !ok[k - 1]) continue;
        if (std::fabs(LGF[k] - LGF[k - 1]) > thr) {
            // 相位跳变：判码 GF 同历元是否也跳 → 区分真实周跳/伪影
            if (std::fabs(PGF[k] - PGF[k - 1]) > 0.10) artifact[k] = 1;
            else                                              slip[k] = 1;
        }
    }
}

bool CycleSlip::gfWideLaneSlip(double L1, double L2, double thr) {
    const double gf = fc.GF_phase(L1, L2); // L2 - L1 (米)
    bool slip = false;
    if (mGfWlHasPrev) slip = std::fabs(gf - mGfWlPrev) > thr;
    mGfWlPrev = gf; mGfWlHasPrev = true;
    return slip;
}
