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

SolverOptions loadConfig(const AppConfig::IniFile &ini) {
    SolverOptions c;

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
    c.sampleInterval = ini.getD("solver", "sample_interval", c.sampleInterval); // 0=按文件自动
    c.sigIFCode = ini.getD("solver", "sig_if_code", c.sigIFCode);
    c.sigPhase = ini.getD("solver", "sig_phase", c.sigPhase);
    c.useAR = ini.getInt("solver", "use_ar", c.useAR);
    c.useCycleSlip = ini.getInt("solver", "use_cycle_slip", c.useCycleSlip);
    c.useGfCycleSlip = ini.getInt("solver", "use_gf_cycle_slip", c.useGfCycleSlip);
    c.fullCovariance = ini.getInt("solver", "full_covariance", c.fullCovariance);
    c.solidTide = ini.getInt("solver", "solid_tide", c.solidTide);
    c.kinematic = ini.getInt("solver", "kinematic", c.kinematic);
    c.posNoise = ini.getD("solver", "pos_noise", c.posNoise);
    c.staticPosNoise = ini.getD("solver", "static_pos_noise", c.staticPosNoise);
    c.ztdQ = ini.getD("solver", "ztd_q", c.ztdQ);
    c.ambQ = ini.getD("solver", "amb_q", c.ambQ);
    c.troZtdSigma = ini.getD("solver", "tro_ztd_sigma", c.troZtdSigma);
    if (ini.getInt("solver", "use_tro_ztd", 1) == 0) c.troZtdSigma = 0.0;
    c.useTroGrad = ini.getInt("solver", "use_tro_grad", 1);
    c.tropoMap = ini.get("solver", "tropo_map", c.tropoMap);
    c.ztdPrior = ini.getD("solver", "ztd_prior", c.ztdPrior);
    c.motionThresh = ini.getD("solver", "motion_thresh", c.motionThresh);
    c.sppVsPredThresh = ini.getD("solver", "spp_vs_pred_thresh", c.sppVsPredThresh);
    c.firstEpochPosVar = ini.getD("solver", "first_epoch_pos_var", c.firstEpochPosVar);
    c.ztdInitValue = ini.getD("solver", "ztd_init", c.ztdInitValue);
    c.ztdInitVar = ini.getD("solver", "ztd_init_var", c.ztdInitVar);
    c.ambInitVar = ini.getD("solver", "amb_init_var", c.ambInitVar);
    c.keepAmbMaxGap = ini.getD("solver", "keep_amb_max_gap", c.keepAmbMaxGap);
    c.codeBiasProcNoise = ini.getD("solver", "code_bias_proc_noise", c.codeBiasProcNoise);
    c.codeBiasInitVar = ini.getD("solver", "code_bias_init_var", c.codeBiasInitVar);
    c.gloIfcbProcNoise = ini.getD("solver", "glo_ifcb_proc_noise", c.gloIfcbProcNoise);
    c.gloIfcbInitVar = ini.getD("solver", "glo_ifcb_init_var", c.gloIfcbInitVar);
    c.isbProcNoise = ini.getD("solver", "isb_proc_noise", c.isbProcNoise);
    c.isbInitVar = ini.getD("solver", "isb_init_var", c.isbInitVar);
    c.posVarFloor = ini.getD("solver", "pos_var_floor", c.posVarFloor);

    c.csvPath = ini.get("output", "csv", "");
    c.applyWindup = ini.getInt("solver", "windup", c.applyWindup);
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
