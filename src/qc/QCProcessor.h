#pragma once

#include <vector>
#include <map>
#include <string>
#include "GnssStruct.h"

namespace QC {
    struct QCObsEpoch {
        double sow = 0;
        std::vector<SatID> satIds;
        std::vector<TypeValueMap> allObs;
        std::vector<double> elevations;
        std::vector<double> azimuths;
        double pdop = 0, hdop = 0, vdop = 0, gdop = 0;
        int nsat = 0;
    };

    inline QCObsEpoch makeQCObsEpoch(const ObsData &obs, double sow) {
        QCObsEpoch qe;
        qe.sow = sow;
        for (auto &[sat, tv]: obs.satTypeValueData) {
            qe.satIds.push_back(sat);
            qe.allObs.push_back(tv);
        }
        qe.nsat = static_cast<int>(qe.satIds.size());
        return qe;
    }

    struct SatQC {
        SatID sat;
        char band1 = 0;
        char band2 = 0;
        int totalEpochs = 0;
        int validDual = 0;

        double completeness = 0;
        double di_f1 = 0;
        double di_f2 = 0;

        int slips = 0;
        int outliers = 0;
        int clockJumps = 0;
        double slipRatio = 0;

        double mp1Mean = 0, mp1Rms = 0;
        double mp2Mean = 0, mp2Rms = 0;

        double sigRho1 = 0;
        double sigRho2 = 0;
        double sigPh1 = 0;
        double sigPh2 = 0;

        double ionoRateStd = 0;
        double ionoRateStd1 = 0;
        double ionoRateStd2 = 0;
        int ionoJumps = 0;
        int ionoJumps1 = 0;
        int ionoJumps2 = 0;


        std::vector<int> epIdx;
        std::vector<double> t;
        std::vector<double> mp1, mp2;
        std::vector<double> mp1Resid, mp2Resid;
        std::vector<double> ionoRate;
        std::vector<double> ionoRate2;
        std::vector<char> slipFlag;
        std::vector<char> outlierFlag;
        std::vector<char> clockJumpFlag;


        std::vector<double> cnr;
        std::vector<double> cnr2;
        double cnrMean = 0, cnrMin = 0, cnrMax = 0;
        double cnrMean2 = 0, cnrMin2 = 0, cnrMax2 = 0;
    };


    struct ComprehensiveEval {
        std::vector<std::string> indicators;
        std::vector<double> weights;
        std::map<SatID, std::vector<double> > z;
        std::map<SatID, double> E;
        std::map<char, double> sysE;
        double overallE = 0;
    };

    struct QualityReport {
        int totalEpochs = 0;
        int totalInputEpochs = 0;
        double interval = 0;
        std::vector<SatID> satOrder;
        std::map<SatID, SatQC> sats;

        std::map<char, double> sysCompleteness;
        std::map<char, double> sysDI_f1, sysDI_f2;
        std::map<char, double> sysSlipRatio;
        std::map<char, double> sysMp1, sysMp2;
        std::map<char, double> sysSigRho, sysSigPhase;
        std::map<char, int> sysSlips, sysIonoJumps, sysOutliers, sysClockJumps;
        std::map<char, double> sysIonoRate;
        std::map<char, double> sysIonoRate1;
        std::map<char, double> sysIonoRate2;
        std::map<char, int> sysIonoJumps1;
        std::map<char, int> sysIonoJumps2;

        std::map<char, double> sysCnr;
        std::map<char, double> sysCnr1;
        std::map<char, double> sysCnr2;
        double overallCnr = 0;
        double overallCnr1 = 0;
        double overallCnr2 = 0;

        double overallCompleteness = 0;

        int clockJumpEpochs = 0;
        ComprehensiveEval comprehensive;
    };


    QualityReport compute(const std::vector<QCObsEpoch> &epochs, double nominalInterval = 0.0);
}
