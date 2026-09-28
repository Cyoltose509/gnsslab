#include "FreqCombo.h"
#include <algorithm>
#include "Const.h"   // getFreq, C_MPS

FreqCombo::FreqCombo(std::string c1, std::string c2)
    : code1(std::move(c1)), code2(std::move(c2)) {
}

FreqCombo FreqCombo::fromFreq(const double f1, const double f2) {
    FreqCombo d;
    d.f1 = f1;
    d.f2 = f2;
    d.lambda1 = C_MPS / f1;
    d.lambda2 = C_MPS / f2;
    d.lambdaW = C_MPS / (f1 - f2);
    d.lambdaN = C_MPS / (f1 + f2);
    const double f1s = f1 * f1, f2s = f2 * f2;
    d.c1 = f1s / (f1s - f2s);
    d.c2 = -f2s / (f1s - f2s);
    return d;
}

FreqCombo FreqCombo::detect(char sys, const std::vector<std::string> &codes, bool requirePhase) {
    FreqCombo invalid;

    // 在 codes 中按 (kind, band) 找一个码
    auto findCode = [&](const char kind, const char band) -> std::string {
        for (const auto &c: codes)
            if (c.size() >= 2 && c[0] == kind && c[1] == band) return c;
        return "";
    };

    std::string code1, code2;
    if (sys == 'G') {
        for (char b: {'1', '2'}) if (code1.empty()) code1 = findCode('C', b);
        if (code2.empty())
            for (char b: {'2', '5', '7'}) if (code2.empty()) code2 = findCode('C', b);
    } else if (sys == 'C') {
        for (char b: {'2', '1'}) if (code1.empty()) code1 = findCode('C', b);
        for (char b: {'6', '7'}) if (code2.empty()) code2 = findCode('C', b);
    } else {
        // 通用兜底
        std::vector<std::string> avail;
        for (const auto &c: codes) if (c.size() >= 2 && c[0] == 'C') avail.push_back(c);
        std::sort(avail.begin(), avail.end(), [&](const std::string &a, const std::string &b) {
            return getFreq(sys, a) < getFreq(sys, b);
        });
        if (avail.size() >= 2) {
            code1 = avail[0];
            code2 = avail[1];
        }
    }
    if (code1.empty() || code2.empty() || code1 == code2) return invalid;

    std::string phase1 = findCode('L', code1[1]);
    std::string phase2 = findCode('L', code2[1]);
    if (requirePhase && (phase1.empty() || phase2.empty())) return invalid;
    if (phase1.empty()) phase1 = "L" + std::string(1, code1[1]);
    if (phase2.empty()) phase2 = "L" + std::string(1, code2[1]);

    const double f1 = getFreq(sys, code1);
    const double f2 = getFreq(sys, code2);
    if (f1 == 0.0 || f2 == 0.0) return invalid;

    const double f1s = f1 * f1, f2s = f2 * f2;
    FreqCombo d;
    d.code1 = code1;
    d.code2 = code2;
    d.phase1 = phase1;
    d.phase2 = phase2;
    d.f1 = f1;
    d.f2 = f2;
    d.c1 = f1s / (f1s - f2s);
    d.c2 = -f2s / (f1s - f2s);
    d.lambda1 = C_MPS / f1;
    d.lambda2 = C_MPS / f2;
    d.lambdaW = C_MPS / (f1 - f2);
    d.lambdaN = C_MPS / (f1 + f2);
    return d;
}


// 从整段 ObsData 收集某系统的观测码
FreqCombo FreqCombo::detect(const char sys, const ObsData &obs, const bool requirePhase) {
    std::vector<std::string> codes;
    for (const auto &[sat, tv]: obs.satTypeValueData)
        if (sat.system == sys)
            for (const auto &[c, _]: tv) codes.push_back(c);
    return detect(sys, codes, requirePhase);
}

bool FreqCombo::available(const TypeValueMap &tv, const bool requirePhase) const {
    const auto ok = [&](const std::string &c) {
        const auto it = tv.find(c);
        return it != tv.end() && std::abs(it->second) > 1e-6;
    };
    if (!ok(code1) || !ok(code2)) return false;
    if (requirePhase && (!ok(phase1) || !ok(phase2))) return false;
    return true;
}

double FreqCombo::combineCode(const double P1, const double P2) const { return c1 * P1 + c2 * P2; }
double FreqCombo::combinePhase(const double L1, const double L2) const { return c1 * L1 + c2 * L2; }

double FreqCombo::combineCodeFromObs(const std::map<std::string, double> &obs) const {
    return combineCode(obs.at(code1), obs.at(code2));
}

double FreqCombo::MW_cycle(const double P1, const double P2, const double L1, const double L2,
                           const double osbP1, const double osbP2, const double osbL1, const double osbL2) const {
    const double oP1 = P1 - osbP1, oP2 = P2 - osbP2;
    const double oL1 = L1 - osbL1, oL2 = L2 - osbL2;
    const double lam1 = lambda1, lam2 = lambda2;
    const double r = f1 / f2 * (f1 / f2);
    const double a1 = 1.0 / lam1; // = f1/c
    const double a2 = -1.0 / lam2; // = -f2/c
    const double K = -(a1 + a2);
    const double Gc = a1 + r * a2;
    const double b2 = (Gc - K) / (r - 1.0);
    const double b1 = K - b2;
    return a1 * oL1 + a2 * oL2 + b1 * oP1 + b2 * oP2;
}

double FreqCombo::MW_meter(const double P1, const double P2, const double L1, const double L2,
                           const double osbP1, const double osbP2, const double osbL1, const double osbL2) const {
    return MW_cycle(P1, P2, L1, L2, osbP1, osbP2, osbL1, osbL2) * lambdaW;
}

double FreqCombo::GF_phase(const double L1, const double L2) { return L2 - L1; }
double FreqCombo::GF_code(const double P1, const double P2) { return P2 - P1; }
