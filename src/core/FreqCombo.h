#pragma once

#include <string>
#include <set>
#include <map>
#include <vector>
#include "GnssStruct.h"   // SatID（detect 等参数类型）

class FreqCombo {
public:
    std::string code1, code2;    // 伪距码
    std::string phase1, phase2;  // 相位码
    double f1 = 0, f2 = 0;       // Hz
    double c1 = 0, c2 = 0;       // IF 系数
    double lambda1 = 0, lambda2 = 0;
    double lambdaW = 0;          // 宽巷波长
    double lambdaN = 0;          // 窄巷波长

    FreqCombo() = default;

    FreqCombo(std::string c1, std::string c2);

    static FreqCombo fromFreq(double f1, double f2);

    static FreqCombo detect(char sys, const std::set<std::string> &codes, bool requirePhase = true);

    static FreqCombo detect(char sys, const std::vector<std::string> &codes, bool requirePhase = true);

    [[nodiscard]] double combineCode(double P1, double P2) const;
    [[nodiscard]] double combinePhase(double L1, double L2) const;

    [[nodiscard]] double combineCodeFromObs(const std::map<std::string, double> &obs) const;

    [[nodiscard]] double MW_cycle(double P1, double P2, double L1, double L2,
                                 double osbP1 = 0.0, double osbP2 = 0.0,
                                 double osbL1 = 0.0, double osbL2 = 0.0) const;

    [[nodiscard]] double MW_meter(double P1, double P2, double L1, double L2,
                                 double osbP1 = 0.0, double osbP2 = 0.0,
                                 double osbL1 = 0.0, double osbL2 = 0.0) const;

    [[nodiscard]] static double GF_phase(double L1, double L2);
    [[nodiscard]] static double GF_code(double P1, double P2);
    [[nodiscard]] bool available(const TypeValueMap &tv, bool requirePhase = true) const;
};
