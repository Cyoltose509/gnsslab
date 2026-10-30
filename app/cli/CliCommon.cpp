#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "CliCommon.h"
#include "SolverTask.h"
#include "SolveDriver.h"
#include "core/IniFile.h"

static std::set<char> parseSystems(const std::string &s) {
    std::set<char> out;
    for (char c: s)
        if (c >= 'A' && c <= 'Z') out.insert(c);
    if (out.empty()) out = {'G', 'C'};
    return out;
}

SolverOptions loadConfig(const AppConfig::IniFile &ini, const GnssTask::SolverMode mode) {
    SolverOptions c = GnssTask::defaultSolverOptions(mode);

    c.source = ini.get("input", "source", c.source);
    c.obsPath = ini.get("input", "obs", "");
    c.navPath = ini.get("input", "nav", "");
    c.basePath = ini.get("input", "base", "");
    c.roverPath = ini.get("input", "rover", "");
    c.ip = ini.get("input", "ip", "");
    c.port = ini.getInt("input", "port", 0);
    c.baseIp = ini.get("input", "base_ip", c.baseIp);
    c.basePort = ini.getInt("input", "base_port", c.basePort);
    c.roverIp = ini.get("input", "rover_ip", c.roverIp);
    c.roverPort = ini.getInt("input", "rover_port", c.roverPort);

    c.sp3 = ini.get("input", "sp3", "");
    c.clk = ini.get("input", "clk", "");
    c.atx = ini.get("input", "atx", "");
    c.osb = ini.get("input", "osb", "");
    c.erp = ini.get("input", "erp", "");
    c.tro = ini.get("input", "tro", "");
    c.ionex = ini.get("input", "ionex", "");

    c.gnssSp3 = ini.get("input", "gnss_sp3", "");
    c.gnssClk = ini.get("input", "gnss_clk", "");
    c.gnssBrdc = ini.get("input", "gnss_brdc", "");
    c.refSp3 = ini.get("input", "ref_sp3", "");
    c.aux = ini.get("input", "aux", "");
    c.refPrn = ini.getInt("input", "ref_prn", c.refPrn);
    c.realtimeSec = ini.getInt("input", "realtime_sec", 0);
    c.maxEpochs = ini.getInt("input", "max_epochs", 0);

    c.cutoffDeg = ini.getD("solver", "cutoff_deg", c.cutoffDeg);
    c.systems = parseSystems(ini.get("solver", "systems", "GC"));
    c.sampleInterval = ini.getD("solver", "sample_interval", c.sampleInterval);

    c.spp.sigIFCode = ini.getD("solver", "sig_if_code", c.spp.sigIFCode);
    c.spp.sigmaPhase = ini.getD("solver", "sig_phase", c.spp.sigmaPhase);

    auto both = [&ini](const char *key, double &a, double &b) {
        a = ini.getD("solver", key, a);
        b = ini.getD("solver", key, b);
    };
    auto bothFlag = [&ini](const char *key, bool &a, bool &b) {
        a = ini.getInt("solver", key, a ? 1 : 0) != 0;
        b = ini.getInt("solver", key, b ? 1 : 0) != 0;
    };
    both("min_sin_elev", c.spp.minSinElev, c.rtk.phaseWeight.minSinElev);
    both("conv_eps", c.spp.convEps, c.rtk.convEps);
    both("ar_ratio_threshold", c.spp.arRatioThreshold, c.rtk.arThreshold);
    bothFlag("use_ar", c.spp.useAR, c.rtk.useAR);
    bothFlag("use_cycle_slip", c.spp.useCycleSlip, c.rtk.useCycleSlip);
    both("gf_slip_mult", c.spp.gfSlipMult, c.rtk.gfSlipMult);

    c.rtk.useGfCycleSlip = ini.getInt("solver", "use_gf_cycle_slip", c.rtk.useGfCycleSlip ? 1 : 0) != 0;
    c.rtk.fullCovariance = ini.getInt("solver", "full_covariance", c.rtk.fullCovariance ? 1 : 0) != 0;
    c.spp.applySolidTide = ini.getInt("solver", "solid_tide", c.spp.applySolidTide ? 1 : 0) != 0;
    c.spp.applyWindup = ini.getInt("solver", "windup", c.spp.applyWindup ? 1 : 0) != 0;
    c.kinematic = ini.getInt("solver", "kinematic", c.kinematic);
    c.posNoise = ini.getD("solver", "pos_noise", c.posNoise);
    c.staticPosNoise = ini.getD("solver", "static_pos_noise", c.staticPosNoise);
    c.spp.ztdProcNoise = ini.getD("solver", "ztd_q", c.spp.ztdProcNoise);
    c.spp.ambNoise = ini.getD("solver", "amb_q", c.spp.ambNoise);
    c.spp.troZtdSigma = ini.getD("solver", "tro_ztd_sigma", c.spp.troZtdSigma);
    if (ini.getInt("solver", "use_tro_ztd", 1) == 0) c.spp.troZtdSigma = 0.0;
    c.spp.useTroGrad = ini.getInt("solver", "use_tro_grad", c.spp.useTroGrad ? 1 : 0) != 0;

    c.spp.tropoMap = Troposphere::tropoMapFromName(
        ini.get("solver", "tropo_map", Troposphere::tropoMapName(c.spp.tropoMap)));

    c.spp.ztdPriorSigma = ini.getD("solver", "ztd_prior", c.spp.ztdPriorSigma);
    c.motionThresh = ini.getD("solver", "motion_thresh", c.motionThresh);
    c.sppVsPredThresh = ini.getD("solver", "spp_vs_pred_thresh", c.sppVsPredThresh);
    c.spp.firstEpochPosVar = ini.getD("solver", "first_epoch_pos_var", c.spp.firstEpochPosVar);
    c.spp.ztdInitValue = ini.getD("solver", "ztd_init", c.spp.ztdInitValue);
    c.spp.ztdInitVar = ini.getD("solver", "ztd_init_var", c.spp.ztdInitVar);
    c.spp.ambInitVar = ini.getD("solver", "amb_init_var", c.spp.ambInitVar);
    c.spp.keepAmbMaxGap = ini.getD("solver", "keep_amb_max_gap", c.spp.keepAmbMaxGap);
    c.spp.codeBiasProcNoise = ini.getD("solver", "code_bias_proc_noise", c.spp.codeBiasProcNoise);
    c.spp.codeBiasInitVar = ini.getD("solver", "code_bias_init_var", c.spp.codeBiasInitVar);
    c.spp.gloIfcbProcNoise = ini.getD("solver", "glo_ifcb_proc_noise", c.spp.gloIfcbProcNoise);
    c.spp.gloIfcbInitVar = ini.getD("solver", "glo_ifcb_init_var", c.spp.gloIfcbInitVar);
    c.spp.isbProcNoise = ini.getD("solver", "isb_proc_noise", c.spp.isbProcNoise);
    c.spp.isbInitVar = ini.getD("solver", "isb_init_var", c.spp.isbInitVar);
    c.spp.posVarFloor = ini.getD("solver", "pos_var_floor", c.spp.posVarFloor);

    c.spp.maxIter = ini.getInt("solver", "max_iter", c.spp.maxIter);
    c.spp.minCheckIter = ini.getInt("solver", "min_check_iter", c.spp.minCheckIter);
    c.spp.arSigGate = ini.getD("solver", "ar_sig_gate", c.spp.arSigGate);
    c.spp.clockNoise = ini.getD("solver", "clock_noise", c.spp.clockNoise);
    c.spp.minSatsForUpdate = ini.getInt("solver", "min_sats_for_update", c.spp.minSatsForUpdate);
    c.spp.maxReanchorIter = ini.getInt("solver", "max_reanchor_iter", c.spp.maxReanchorIter);
    c.spp.sppResidualThr = ini.getD("solver", "spp_residual_thr", c.spp.sppResidualThr);

    c.rtk.maxIterFloat = ini.getInt("solver", "max_iter_float", c.rtk.maxIterFloat);
    c.rtk.maxIterFix = ini.getInt("solver", "max_iter_fix", c.rtk.maxIterFix);
    c.rtk.minSats = ini.getInt("solver", "min_sats", c.rtk.minSats);
    c.rtk.codeOverPhaseVar = ini.getD("solver", "code_over_phase_var", c.rtk.codeOverPhaseVar);
    c.rtk.excludeGeoRef = ini.getInt("solver", "exclude_geo_ref", c.rtk.excludeGeoRef ? 1 : 0) != 0;
    c.rtk.phaseWeight.a = ini.getD("solver", "phase_weight_a", c.rtk.phaseWeight.a);
    c.rtk.phaseWeight.b = ini.getD("solver", "phase_weight_b", c.rtk.phaseWeight.b);

    c.csvPath = ini.get("output", "csv", "");
    c.refX = ini.getD("output", "ref_x", 0.0);
    c.refY = ini.getD("output", "ref_y", 0.0);
    c.refZ = ini.getD("output", "ref_z", 0.0);
    c.hasRef = c.refX != 0.0 || c.refY != 0.0 || c.refZ != 0.0;
    c.baseCoordX = ini.getD("input", "base_coord_x", 0.0);
    c.baseCoordY = ini.getD("input", "base_coord_y", 0.0);
    c.baseCoordZ = ini.getD("input", "base_coord_z", 0.0);

    return c;
}

namespace Cli {
    std::string defaultCsvName(const GnssTask::SolverMode m) {
        switch (m) {
            case GnssTask::SolverMode::Leo: return "gnss_leo.csv";
            case GnssTask::SolverMode::Rtk: return "gnss_rtk.csv";
            case GnssTask::SolverMode::Ppp: return "gnss_ppp.csv";
            default: return "gnss_spp.csv";
        }
    }

    void dumpCsv(const std::vector<GnssTask::EpochData> &epochs,
                 const GnssTask::SolverMode mode, const Eigen::Vector3d &ref,
                 const bool hasRefOrbit, const SolverOptions &cfg) {
        const std::string path = cfg.csvPath.empty() ? defaultCsvName(mode) : cfg.csvPath;
        std::ofstream out(path);
        if (!out) {
            std::cerr << "无法打开输出文件: " << path << "\n";
            return;
        }
        GnssTask::exportCsv(out, epochs, mode, ref, hasRefOrbit);
        std::cout << "已写出 " << epochs.size() << " 历元 -> " << path << "\n";
    }

    void dumpEmpty(const GnssTask::SolverMode mode, const SolverOptions &cfg) {
        dumpCsv({}, mode, Eigen::Vector3d(0, 0, 0), false, cfg);
    }
}
