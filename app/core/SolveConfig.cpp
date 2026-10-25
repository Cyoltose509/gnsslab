#include "core/SolveConfig.h"
#include "core/IniFile.h"

static std::set<char> parseSystems(const std::string &s) {
    std::set<char> out;
    for (char c : s)
        if (c >= 'A' && c <= 'Z') out.insert(c);
    if (out.empty()) out = {'G', 'C'};
    return out;
}

SolveConfig loadConfig(const AppConfig::IniFile &ini) {
    SolveConfig c;

    // ---- [input] ----
    c.source   = ini.get("input", "source", c.source);
    c.obsPath  = ini.get("input", "obs", "");
    c.navPath  = ini.get("input", "nav", "");
    c.basePath = ini.get("input", "base", "");
    c.roverPath = ini.get("input", "rover", "");
    c.ip       = ini.get("input", "ip", "");
    c.port     = ini.getInt("input", "port", 0);
    c.baseIp   = ini.get("input", "base_ip", c.baseIp);
    c.basePort = ini.getInt("input", "base_port", c.basePort);
    c.roverIp  = ini.get("input", "rover_ip", c.roverIp);
    c.roverPort = ini.getInt("input", "rover_port", c.roverPort);

    c.sp3  = ini.get("input", "sp3", "");
    c.clk  = ini.get("input", "clk", "");
    c.atx  = ini.get("input", "atx", "");
    c.osb  = ini.get("input", "osb", "");
    c.erp  = ini.get("input", "erp", "");
    c.tro  = ini.get("input", "tro", "");
    c.gpt2 = ini.get("input", "gpt2", "");
    c.ionex = ini.get("input", "ionex", "");

    c.gnssSp3  = ini.get("input", "gnss_sp3", "");
    c.gnssClk  = ini.get("input", "gnss_clk", "");
    c.gnssBrdc = ini.get("input", "gnss_brdc", "");
    c.refSp3   = ini.get("input", "ref_sp3", "");
    c.aux      = ini.get("input", "aux", "");
    c.refPrn   = ini.getInt("input", "ref_prn", c.refPrn);
    c.realtimeSec = ini.getInt("input", "realtime_sec", 0);
    c.maxEpochs   = ini.getInt("input", "max_epochs", 0);

    // ---- [solver] ----
    c.cutoffDeg   = ini.getD("solver", "cutoff_deg", c.cutoffDeg);
    c.systems     = parseSystems(ini.get("solver", "systems", "GC"));
    c.sigIFCode   = ini.getD("solver", "sig_if_code", c.sigIFCode);
    c.sigPhase    = ini.getD("solver", "sig_phase", c.sigPhase);
    c.useAR       = ini.getInt("solver", "use_ar", c.useAR);
    c.useCycleSlip = ini.getInt("solver", "use_cycle_slip", c.useCycleSlip);
    c.useGfCycleSlip = ini.getInt("solver", "use_gf_cycle_slip", c.useGfCycleSlip);
    c.fullCovariance = ini.getInt("solver", "full_covariance", c.fullCovariance);
    c.solidTide   = ini.getInt("solver", "solid_tide", c.solidTide);
    c.kinematic   = ini.getInt("solver", "kinematic", c.kinematic);
    c.posNoise    = ini.getD("solver", "pos_noise", c.posNoise);
    c.staticPosNoise = ini.getD("solver", "static_pos_noise", c.staticPosNoise);
    c.ztdQ        = ini.getD("solver", "ztd_q", c.ztdQ);
    c.ambQ        = ini.getD("solver", "amb_q", c.ambQ);
    c.troZtdSigma = ini.getD("solver", "tro_ztd_sigma", c.troZtdSigma);
    if (ini.getInt("solver", "use_tro_ztd", 1) == 0) c.troZtdSigma = 0.0; // use_tro_ztd 死键复活：=0 真正关闭 TRO ZTD 约束（tro= 文件仍加载，但不再对其做先验约束）
    c.useTroGrad = ini.getInt("solver", "use_tro_grad", 1); // TRO 水平梯度固定改正开关（默认开）
    c.tropoMap = ini.get("solver", "tropo_map", c.tropoMap); // 对流层映射函数 gmf/nmf
    c.ztdPrior    = ini.getD("solver", "ztd_prior", c.ztdPrior);
    c.motionThresh = ini.getD("solver", "motion_thresh", c.motionThresh);
    c.sppVsPredThresh = ini.getD("solver", "spp_vs_pred_thresh", c.sppVsPredThresh);
    c.firstEpochPosVar = ini.getD("solver", "first_epoch_pos_var", c.firstEpochPosVar);
    c.ztdInitValue = ini.getD("solver", "ztd_init", c.ztdInitValue);
    c.ztdInitVar   = ini.getD("solver", "ztd_init_var", c.ztdInitVar);
    c.ambInitVar  = ini.getD("solver", "amb_init_var", c.ambInitVar);
    c.keepAmbMaxGap = ini.getD("solver", "keep_amb_max_gap", c.keepAmbMaxGap);
    c.codeBiasProcNoise = ini.getD("solver", "code_bias_proc_noise", c.codeBiasProcNoise);
    c.codeBiasInitVar   = ini.getD("solver", "code_bias_init_var", c.codeBiasInitVar);
    // 逐 GLONASS 星 IFCB 改为「自动规则」：OSB 未覆盖的星才估内部 IFCB，故不再读 estimate_glo_ifcb / estimate_code_bias 开关。
    c.gloIfcbProcNoise  = ini.getD("solver", "glo_ifcb_proc_noise", c.gloIfcbProcNoise);
    c.gloIfcbInitVar    = ini.getD("solver", "glo_ifcb_init_var", c.gloIfcbInitVar);
    // ISB 过程噪声/初方差：标量 isb_proc_noise / isb_init_var。
    // isbInitVar<=0 ⇒ 未启用 ISB（非参考系统 cdt2~4 走 clock* 白噪声旧口径，逐历元自由重估）。
    // 默认 RW-ISB：isb_proc_noise=1.0e-3、isb_init_var=1.0（GLONASS 同样走 RW-ISB）。
    c.isbProcNoise      = ini.getD("solver", "isb_proc_noise", c.isbProcNoise);
    c.isbInitVar        = ini.getD("solver", "isb_init_var", c.isbInitVar);
    c.posVarFloor       = ini.getD("solver", "pos_var_floor", c.posVarFloor);

    // ---- [output] ----
    c.csvPath = ini.get("output", "csv", "");
    c.applyWindup = ini.getInt("solver", "windup", c.applyWindup);
    c.refX = ini.getD("output", "ref_x", 0.0);
    c.refY = ini.getD("output", "ref_y", 0.0);
    c.refZ = ini.getD("output", "ref_z", 0.0);
    c.hasRef = (c.refX != 0.0 || c.refY != 0.0 || c.refZ != 0.0);

    return c;
}
