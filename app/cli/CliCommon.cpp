#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "CliCommon.h"
#include "solvers/common/Core.h"
#include "solvers/common/Config.h"
#include "core/IniFile.h"

static std::set<char> parseSystems(const std::string &s) {
    std::set<char> out;
    for (char c: s)
        if (c >= 'A' && c <= 'Z') out.insert(c);
    if (out.empty()) out = {'G', 'C'};
    return out;
}

/// ini 的 mode 键 → 解算模式。静态与动态是两个独立模式，各有各的参数。
Solvers::Mode modeFromKey(const std::string &s) {
    if (s == "spp") return Solvers::Mode::SPP;
    if (s == "ppp_static" || s == "ppp") return Solvers::Mode::PPPStatic;
    if (s == "ppp_kin" || s == "ppp_kinematic") return Solvers::Mode::PPPKinematic;
    if (s == "rtk") return Solvers::Mode::RTK;
    if (s == "leo") return Solvers::Mode::LEO;
    return Solvers::Mode::SPP;
}

namespace {
    /// SPP 家族（SPP / PPP / LEO）共用的观测量与滤波参数。
    void loadCommon(const IniFile &ini, SolverOptions &c) {
        c.cutoffDeg = ini.getD("solver", "cutoff_deg", c.cutoffDeg);
        c.systems = parseSystems(ini.get("solver", "systems", "GC"));
        c.sampleInterval = ini.getD("solver", "sample_interval", c.sampleInterval);

        c.spp.sigIFCode = ini.getD("solver", "sig_if_code", c.spp.sigIFCode);
        c.spp.sigmaPhase = ini.getD("solver", "sig_phase", c.spp.sigmaPhase);
        c.spp.minSinElev = ini.getD("solver", "min_sin_elev", c.spp.minSinElev);
        c.spp.convEps = ini.getD("solver", "conv_eps", c.spp.convEps);
        c.spp.maxIter = ini.getInt("solver", "max_iter", c.spp.maxIter);
        c.spp.minCheckIter = ini.getInt("solver", "min_check_iter", c.spp.minCheckIter);
        c.spp.useHealth = ini.getB("solver", "use_health", c.spp.useHealth);
        c.spp.applySolidTide = ini.getB("solver", "solid_tide", c.spp.applySolidTide);
        c.spp.applyWindup = ini.getB("solver", "windup", c.spp.applyWindup);
        c.spp.isbProcNoise = ini.getD("solver", "isb_proc_noise", c.spp.isbProcNoise);
        c.spp.isbInitVar = ini.getD("solver", "isb_init_var", c.spp.isbInitVar);
        c.spp.tropoMap = Troposphere::fromKey(
            ini.get("solver", "tropo_map", Troposphere::keyOf(c.spp.tropoMap)));

        c.spp.useAR = ini.getB("solver", "use_ar", c.spp.useAR);
        c.spp.arRatioThreshold = ini.getD("solver", "ar_ratio_threshold", c.spp.arRatioThreshold);
        c.spp.arSigGate = ini.getD("solver", "ar_sig_gate", c.spp.arSigGate);
        c.spp.ambInitVar = ini.getD("solver", "amb_init_var", c.spp.ambInitVar);
        c.spp.ambNoise = ini.getD("solver", "amb_q", c.spp.ambNoise);
        c.spp.keepAmbMaxGap = ini.getD("solver", "keep_amb_max_gap", c.spp.keepAmbMaxGap);
        c.spp.useCycleSlip = ini.getB("solver", "use_cycle_slip", c.spp.useCycleSlip);
        c.spp.gfSlipMult = ini.getD("solver", "gf_slip_mult", c.spp.gfSlipMult);
        c.spp.codeBiasProcNoise = ini.getD("solver", "code_bias_proc_noise", c.spp.codeBiasProcNoise);
        c.spp.codeBiasInitVar = ini.getD("solver", "code_bias_init_var", c.spp.codeBiasInitVar);
        c.spp.gloIfcbProcNoise = ini.getD("solver", "glo_ifcb_proc_noise", c.spp.gloIfcbProcNoise);
        c.spp.gloIfcbInitVar = ini.getD("solver", "glo_ifcb_init_var", c.spp.gloIfcbInitVar);
    }

    /// PPP / LEO 的对流层与滤波参数（静态与动态通用）。
    void loadPPPFilter(const IniFile &ini, SolverOptions &c) {
        c.spp.ztdProcNoise = ini.getD("solver", "ztd_q", c.spp.ztdProcNoise);
        c.spp.troZtdSigma = ini.getD("solver", "tro_ztd_sigma", c.spp.troZtdSigma);
        if (!ini.getB("solver", "use_tro_ztd", true)) c.spp.troZtdSigma = 0.0;
        c.spp.useTroGrad = ini.getB("solver", "use_tro_grad", c.spp.useTroGrad);
        c.spp.ztdPriorSigma = ini.getD("solver", "ztd_prior", c.spp.ztdPriorSigma);
        c.spp.ztdInitValue = ini.getD("solver", "ztd_init", c.spp.ztdInitValue);
        c.spp.ztdInitVar = ini.getD("solver", "ztd_init_var", c.spp.ztdInitVar);
        c.spp.firstEpochPosVar = ini.getD("solver", "first_epoch_pos_var", c.spp.firstEpochPosVar);
        c.spp.posVarFloor = ini.getD("solver", "pos_var_floor", c.spp.posVarFloor);
        c.spp.minSatsForUpdate = ini.getInt("solver", "min_sats_for_update", c.spp.minSatsForUpdate);
        c.spp.maxReanchorIter = ini.getInt("solver", "max_reanchor_iter", c.spp.maxReanchorIter);
        c.spp.clockNoise = ini.getD("solver", "clock_noise", c.spp.clockNoise);
        c.spp.sppResidualThr = ini.getD("solver", "spp_residual_thr", c.spp.sppResidualThr);
    }

    /// RTK 专属的双差与参考星参数。
    void loadRtk(const IniFile &ini, SolverOptions &c) {
        c.rtk.useAR = ini.getB("solver", "use_ar", c.rtk.useAR);
        c.rtk.fullCovariance = ini.getB("solver", "full_covariance", c.rtk.fullCovariance);
        c.rtk.arThreshold = ini.getD("solver", "ar_ratio_threshold", c.rtk.arThreshold);
        c.rtk.convEps = ini.getD("solver", "conv_eps", c.rtk.convEps);
        c.rtk.maxIterFloat = ini.getInt("solver", "max_iter_float", c.rtk.maxIterFloat);
        c.rtk.maxIterFix = ini.getInt("solver", "max_iter_fix", c.rtk.maxIterFix);
        c.rtk.minSats = ini.getInt("solver", "min_sats", c.rtk.minSats);
        c.rtk.useCycleSlip = ini.getB("solver", "use_cycle_slip", c.rtk.useCycleSlip);
        c.rtk.useGfCycleSlip = ini.getB("solver", "use_gf_cycle_slip", c.rtk.useGfCycleSlip);
        c.rtk.gfSlipMult = ini.getD("solver", "gf_slip_mult", c.rtk.gfSlipMult);
        c.rtk.phaseWeight.minSinElev = ini.getD("solver", "min_sin_elev", c.rtk.phaseWeight.minSinElev);
        c.rtk.phaseWeight.a = ini.getD("solver", "phase_weight_a", c.rtk.phaseWeight.a);
        c.rtk.phaseWeight.b = ini.getD("solver", "phase_weight_b", c.rtk.phaseWeight.b);
        c.rtk.codeOverPhaseVar = ini.getD("solver", "code_over_phase_var", c.rtk.codeOverPhaseVar);
        c.rtk.excludeGeoRef = ini.getB("solver", "exclude_geo_ref", c.rtk.excludeGeoRef);
    }
}

SolverOptions loadConfig(const IniFile &ini, const Solvers::Mode mode) {
    SolverOptions c = Solvers::defaultSolverOptions(mode);

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
    c.vmf = ini.get("input", "vmf", "");   // kIniListSep（|）分隔的一串 VMF3 时次文件
    c.gpt3 = ini.get("input", "gpt3", "");
    c.att = ini.get("input", "att", "");

    c.gnssSp3 = ini.get("input", "gnss_sp3", "");
    c.gnssClk = ini.get("input", "gnss_clk", "");
    c.gnssBrdc = ini.get("input", "gnss_brdc", "");
    c.refSp3 = ini.get("input", "ref_sp3", "");
    c.aux = ini.get("input", "aux", "");
    c.refPrn = ini.getInt("input", "ref_prn", c.refPrn);
    c.realtimeSec = ini.getInt("input", "realtime_sec", 0);
    c.maxEpochs = ini.getInt("input", "max_epochs", 0);

    // 只读本模式用得到的字段：SPP 家族走观测模型，RTK 走双差参数。
    if (mode == Solvers::Mode::RTK) {
        loadRtk(ini, c);
    } else {
        loadCommon(ini, c);
        if (mode == Solvers::Mode::SPP) {
            // SPP 只用码观测，无相位/滤波参数
        } else {
            loadPPPFilter(ini, c);
            if (mode == Solvers::Mode::LEO) {
                c.posNoise = ini.getD("solver", "pos_noise", c.posNoise);
                c.motionThresh = ini.getD("solver", "motion_thresh", c.motionThresh);
                c.sppVsPredThresh = ini.getD("solver", "spp_vs_pred_thresh", c.sppVsPredThresh);
            } else {
                c.posNoise = ini.getD("solver", "pos_noise", c.posNoise);
            }
        }
    }

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
    std::string defaultCSVName(const Solvers::Mode m) {
        switch (m) {
            case Solvers::Mode::LEO: return "gnss_leo.csv";
            case Solvers::Mode::RTK: return "gnss_rtk.csv";
            case Solvers::Mode::PPPStatic: return "gnss_ppp_static.csv";
            case Solvers::Mode::PPPKinematic: return "gnss_ppp_kin.csv";
            default: return "gnss_spp.csv";
        }
    }

    void dumpTask(Solvers::Task &task, const SolverOptions &cfg) {
        if (cfg.hasRef) task.refECEF = XYZ(cfg.refX, cfg.refY, cfg.refZ);
        const std::string path = cfg.csvPath.empty() ? defaultCSVName(task.mode) : cfg.csvPath;
        std::ofstream out(path);
        if (!out) {
            std::cerr << "无法打开输出文件: " << path << "\n";
            return;
        }
        std::size_t n = 0;
        {
            std::lock_guard lk(task.mutex);
            n = task.epochs.size();
        }
        task.exportCSV(out);
        std::cout << "已写出 " << n << " 历元 -> " << path << "\n";
    }

    int runTask(Solvers::Task &task, const SolverOptions &cfg) {
        task.maxEpochs = cfg.maxEpochs;
        task.realtimeSec = cfg.realtimeSec;

        if (cfg.source == "realtime") {
            std::thread waiter;
            // 没给时限也没给历元上限时，回车停止（与 GUI 的取消按钮等价）
            if (cfg.realtimeSec <= 0 && cfg.maxEpochs <= 0)
                waiter = std::thread([&task] {
                    std::cin.get();
                    task.cancel();
                });
            task.startRealtime();
            task.waitDone();
            if (waiter.joinable()) waiter.join();
        } else {
            task.startFile();
            task.waitDone();
        }

        if (task.errorFlag()) std::cerr << "错误: " << task.errorMessage() << "\n";
        dumpTask(task, cfg);
        return task.ok() ? 0 : 1;
    }
}
