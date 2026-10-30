#pragma once

#include <string>
#include <map>
#include <vector>
#include "GnssStruct.h"

class FreqCombo {
public:
    std::string code1, code2;
    std::string phase1, phase2;
    double f1 = 0, f2 = 0;
    double c1 = 0, c2 = 0;
    double lambda1 = 0, lambda2 = 0;
    double lambdaW = 0;
    double lambdaN = 0;

    FreqCombo() = default;

    FreqCombo(std::string c1, std::string c2);

    static FreqCombo fromFreq(double f1, double f2);


    static FreqCombo detect(char sys, const std::vector<std::string> &codes, bool requirePhase = true);


    static FreqCombo detect(char sys, const std::vector<std::string> &codes, int channel, bool requirePhase);


    static FreqCombo detect(char sys, const TypeValueMap &tv, int channel, bool requirePhase);

    static FreqCombo detect(char sys, const ObsData &obs, bool requirePhase = true);

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
