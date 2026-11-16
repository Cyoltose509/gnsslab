#include <atomic>
#include <cctype>
#include "util/Clock.h"
#include <conio.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "CliCommon.h"
#include "i18n/I18n.h"
#include "FileUtils.h"
#include "solvers/common/Core.h"
#include "solvers/common/Config.h"
#include "core/IniFile.h"

static std::set<char> parseSystems(const std::string &s) {
    std::set<char> out;
    for (char c: s)
        if (c >= 'a' && c <= 'z') out.insert(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
        else if (c >= 'A' && c <= 'Z') out.insert(c);
    if (out.empty()) out = {'G', 'C'};
    return out;
}

/// ini 的 mode 键 → 解算模式。静态与动态是两个独立模式，各有各的参数。
std::optional<Solvers::Mode> modeFromKey(const std::string &s) {
    if (s == "spp") return Solvers::Mode::SPP;
    if (s == "ppp_static" || s == "ppp") return Solvers::Mode::PPPStatic;
    if (s == "ppp_kin" || s == "ppp_kinematic") return Solvers::Mode::PPPKinematic;
    if (s == "rtk") return Solvers::Mode::RTK;
    if (s == "leo") return Solvers::Mode::LEO;
    if (s == "ins") return Solvers::Mode::INS;
    return std::nullopt; // 拼错就报错退出，别静默按 SPP 跑一趟
}

namespace {
    /// SPP 家族（SPP / PPP / LEO）共用的观测量与滤波参数。
    void loadCommon(const CliSource &ini, SolverOptions &c) {
        c.cutoffDeg = ini.getD("solver", "cutoff_deg", c.cutoffDeg);
        c.systems = parseSystems(ini.get("solver", "systems", "GC"));
        c.sampleInterval = ini.getD("solver", "sample_interval", c.sampleInterval);

        c.spp.codeSigma = ini.getD("solver", "code_sigma", c.spp.codeSigma);
        c.spp.phaseSigma = ini.getD("solver", "phase_sigma", c.spp.phaseSigma);
        c.spp.minSinElev = ini.getD("solver", "min_sin_elev", c.spp.minSinElev);
        c.spp.convEps = ini.getD("solver", "conv_eps", c.spp.convEps);
        c.spp.maxIter = ini.getInt("solver", "max_iter", c.spp.maxIter);
        c.spp.minCheckIter = ini.getInt("solver", "min_check_iter", c.spp.minCheckIter);
        c.spp.useHealth = ini.getB("solver", "use_health", c.spp.useHealth);
        c.spp.applySolidTide = ini.getB("solver", "solid_tide", c.spp.applySolidTide);
        c.spp.applyWindup = ini.getB("solver", "windup", c.spp.applyWindup);
        c.spp.isbProcSigma = ini.getD("solver", "isb_proc_sigma", c.spp.isbProcSigma);
        c.spp.isbInitSigma = ini.getD("solver", "isb_init_sigma", c.spp.isbInitSigma);
        c.spp.tropoMap = Troposphere::fromKey(
            ini.get("solver", "tropo_map", Troposphere::keyOf(c.spp.tropoMap)));

        c.spp.useAR = ini.getB("solver", "use_ar", c.spp.useAR);
        c.spp.arRatioThreshold = ini.getD("solver", "ar_ratio_threshold", c.spp.arRatioThreshold);
        c.spp.arGateSigma = ini.getD("solver", "ar_gate_sigma", c.spp.arGateSigma);
        c.spp.ambInitSigma = ini.getD("solver", "amb_init_sigma", c.spp.ambInitSigma);
        c.spp.ambProcSigma = ini.getD("solver", "amb_proc_sigma", c.spp.ambProcSigma);
        c.spp.keepAmbMaxGap = ini.getD("solver", "keep_amb_max_gap", c.spp.keepAmbMaxGap);
        c.spp.useCycleSlip = ini.getB("solver", "use_cycle_slip", c.spp.useCycleSlip);
        c.spp.useGfCycleSlip = ini.getB("solver", "use_gf_cycle_slip", c.spp.useGfCycleSlip);
        c.spp.gfSlipMult = ini.getD("solver", "gf_slip_mult", c.spp.gfSlipMult);
        c.spp.codeBiasProcSigma = ini.getD("solver", "code_bias_proc_sigma", c.spp.codeBiasProcSigma);
        c.spp.codeBiasInitSigma = ini.getD("solver", "code_bias_init_sigma", c.spp.codeBiasInitSigma);
        c.spp.gloIfcbProcSigma = ini.getD("solver", "glo_ifcb_proc_sigma", c.spp.gloIfcbProcSigma);
        c.spp.gloIfcbInitSigma = ini.getD("solver", "glo_ifcb_init_sigma", c.spp.gloIfcbInitSigma);

        // 位置过程噪声与常速度模型（SPP 滤波档用；PPP/LEO 不接速度态）。
        c.posProcSigma = ini.getD("solver", "pos_proc_sigma", c.posProcSigma);
        c.accProcSigma = ini.getD("solver", "acc_proc_sigma", c.accProcSigma);
        c.velInitSigma = ini.getD("solver", "vel_init_sigma", c.velInitSigma);
        // SPP 专属钟差过程 σ；PPP 随后由 loadPPPFilter 的 clock_proc_sigma 覆盖，两套互不干扰。
        c.spp.clockProcSigma = ini.getD("solver", "spp_clock_proc_sigma", c.spp.clockProcSigma);
    }

    /// PPP / LEO 的对流层与滤波参数（静态与动态通用）。
    void loadPPPFilter(const CliSource &ini, SolverOptions &c) {
        c.spp.ztdProcSigma = ini.getD("solver", "ztd_proc_sigma", c.spp.ztdProcSigma);
        c.spp.troZtdSigma = ini.getD("solver", "tro_ztd_sigma", c.spp.troZtdSigma);
        if (!ini.getB("solver", "use_tro_ztd", true)) c.spp.troZtdSigma = 0.0;
        c.spp.useTroGrad = ini.getB("solver", "use_tro_grad", c.spp.useTroGrad);
        c.spp.ztdPriorSigma = ini.getD("solver", "ztd_prior_sigma", c.spp.ztdPriorSigma);
        c.spp.ztdInitValue = ini.getD("solver", "ztd_init", c.spp.ztdInitValue);
        c.spp.ztdInitSigma = ini.getD("solver", "ztd_init_sigma", c.spp.ztdInitSigma);
        c.spp.firstEpochPosSigma = ini.getD("solver", "first_epoch_pos_sigma", c.spp.firstEpochPosSigma);
        c.spp.posFloorSigma = ini.getD("solver", "pos_floor_sigma", c.spp.posFloorSigma);
        c.spp.minSatsForUpdate = ini.getInt("solver", "min_sats_for_update", c.spp.minSatsForUpdate);
        c.spp.maxReanchorIter = ini.getInt("solver", "max_reanchor_iter", c.spp.maxReanchorIter);
        c.spp.clockProcSigma = ini.getD("solver", "clock_proc_sigma", c.spp.clockProcSigma);
        c.spp.sppResidualThr = ini.getD("solver", "spp_residual_thr", c.spp.sppResidualThr);
    }

    /// RTK 专属的双差与参考星参数。
    void loadRTK(const CliSource &ini, SolverOptions &c) {
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
        c.rtk.codeOverPhaseSigma = ini.getD("solver", "code_over_phase_sigma", c.rtk.codeOverPhaseSigma);
        c.rtk.excludeGeoRef = ini.getB("solver", "exclude_geo_ref", c.rtk.excludeGeoRef);
        c.rtk.useHealth = ini.getB("solver", "rtk_use_health", c.rtk.useHealth);
        // 滤波方向与 SPP 家族同一套词表（none/forward/backward/both），缺省 none＝逐历元 LSQ；
        // 位置/速度 σ 与 SPP 共用同一组键，模糊度 σ 用 rtk_ 前缀避免与 PPP 撞车
        if (const std::string flt = ini.get("solver", "rtk_filter", ""); !flt.empty())
            c.filter = Solvers::filterFromKey(flt, c.filter);
        c.rtk.ambInitSigma = ini.getD("solver", "rtk_amb_init_sigma", c.rtk.ambInitSigma);
        c.rtk.ambProcSigma = ini.getD("solver", "rtk_amb_proc_sigma", c.rtk.ambProcSigma);
        c.posProcSigma = ini.getD("solver", "pos_proc_sigma", c.posProcSigma);
        c.accProcSigma = ini.getD("solver", "acc_proc_sigma", c.accProcSigma);
        c.velInitSigma = ini.getD("solver", "vel_init_sigma", c.velInitSigma);
    }

    /// 参考真值：新写 [ref] 段，同时认旧的 [input] ref_sp3 与 [output] ref_x/y/z。
    void loadRef(const CliSource &ini, const Solvers::Mode mode, SolverOptions &c) {
        c.refSp3 = ini.get("ref", "sp3", ini.get("input", "ref_sp3", ""));
        c.refX = ini.getD("ref", "x", ini.getD("output", "ref_x", 0.0));
        c.refY = ini.getD("ref", "y", ini.getD("output", "ref_y", 0.0));
        c.refZ = ini.getD("ref", "z", ini.getD("output", "ref_z", 0.0));
        c.hasRef = c.refX != 0.0 || c.refY != 0.0 || c.refZ != 0.0;

        const std::string form = ini.get("ref", "form", "auto");
        if (form == "fixed") c.refForm = RefForm::Fixed;
        else if (form == "file") c.refForm = RefForm::File;
        else if (form != "auto")
            std::cerr << I18n::tr("cli.ref.form_unknown", form) << "\n";

        if (c.refForm == RefForm::Fixed && !c.hasRef) {
            std::cerr << I18n::tr("cli.ref.fixed_no_xyz") << "\n";
            c.refForm = RefForm::None;
        }
        if (c.refForm == RefForm::File && c.refSp3.empty()) {
            std::cerr << I18n::tr("cli.ref.file_no_sp3") << "\n";
            c.refForm = RefForm::None;
            c.hasRef = false;
        }
        // 模式不接受的参考形式会被 setupTruth 静默降级成「无」，这里点明，
        // 免得跑完看到一列空偏差、以为是指标没算出来。
        if (c.hasRef && !Solvers::modeAllowsRefFixed(mode))
            std::cerr << I18n::tr("cli.ref.no_fixed", Solvers::modeLabel(mode))
                    << (Solvers::modeAllowsRefFile(mode) ? I18n::tr("cli.ref.no_fixed_hint") : std::string())
                    << "\n";
        if (!c.refSp3.empty() && !Solvers::modeAllowsRefFile(mode))
            std::cerr << I18n::tr("cli.ref.no_file", Solvers::modeLabel(mode)) << "\n";
    }
}

SolverOptions loadConfig(const CliSource &ini, const Solvers::Mode mode) {
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
    c.aux = ini.get("input", "aux", "");
    c.leoAnt = ini.get("input", "leo_ant", "");
    c.leoPco = ini.get("input", "leo_pco", "");
    c.realtimeSec = ini.getInt("input", "realtime_sec", 0);
    c.maxEpochs = ini.getInt("input", "max_epochs", 0);
    loadRef(ini, mode, c);

    if (mode == Solvers::Mode::INS) {
        c.ins.imuPath = ini.get("input", "ins_imu", "");
        c.ins.latDeg = ini.getD("input", "ins_lat", 0.0);
        c.ins.lonDeg = ini.getD("input", "ins_lon", 0.0);
        c.ins.hM = ini.getD("input", "ins_h", 0.0);
        c.ins.vN = ini.getD("input", "ins_vn", 0.0);
        c.ins.vE = ini.getD("input", "ins_ve", 0.0);
        c.ins.vD = ini.getD("input", "ins_vd", 0.0);
        c.ins.specifyAttitude = ini.getB("input", "ins_specify_att", false);
        c.ins.rollDeg = ini.getD("input", "ins_roll", 0.0);
        c.ins.pitchDeg = ini.getD("input", "ins_pitch", 0.0);
        c.ins.headingDeg = ini.getD("input", "ins_heading", 0.0);
        c.ins.zuptAccelTol = ini.getD("input", "ins_zupt_accel_tol", 0.3);
        c.ins.zuptGyroTol = ini.getD("input", "ins_zupt_gyro_tol", 0.02);
        c.ins.zuptHoldSec = ini.getD("input", "ins_zupt_hold", 0.5);
        c.ins.manual.accelMpsPerLsb = ini.getD("input", "ins_accel_lsb", 0.0);
        c.ins.manual.gyroRadPerLsb = ini.getD("input", "ins_gyro_lsb", 0.0);
        c.ins.manual.rateHz = ini.getD("input", "ins_rate_hz", 0.0);
    }

    // 只读本模式用得到的字段：SPP 家族走观测模型，RTK 走双差参数，INS 不读 GNSS 参数。
    if (mode == Solvers::Mode::RTK) {
        loadRTK(ini, c);
    } else if (mode != Solvers::Mode::INS) {
        loadCommon(ini, c);
        if (const std::string sm = ini.get("solver", "smooth", ""); !sm.empty())
            c.filter = Solvers::filterFromKey(sm, SolverOptions::FilterMode::Forward);
        if (mode != Solvers::Mode::SPP) loadPPPFilter(ini, c);
        if (mode == Solvers::Mode::LEO) {
            c.motionThresh = ini.getD("solver", "motion_thresh", c.motionThresh);
            c.sppVsPredThresh = ini.getD("solver", "spp_vs_pred_thresh", c.sppVsPredThresh);
        }
    }

    c.csvPath = ini.get("output", "csv", "");
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
            case Solvers::Mode::INS: return "gnss_ins.csv";
            default: return "gnss_spp.csv";
        }
    }

    void applyFieldList(const std::string &list, Solvers::Export::Spec &spec, const char *where) {
        std::string cur;
        for (const char ch: list + ",") {
            if (ch != ',' && ch != ' ' && ch != '\t') {
                cur.push_back(ch);
                continue;
            }
            if (cur.empty()) continue;
            if (const auto f = Solvers::Export::fieldByKey(cur)) spec.fields.push_back(*f);
            else std::cerr << I18n::tr("cli.fields.unknown", where, cur) << "\n";
            cur.clear();
        }
    }

    namespace {
        /// 换掉扩展名（路径里没有扩展名就补一个），只认最后一段里的点。
        std::string withExt(const std::string &path, const char *ext) {
            const auto slash = path.find_last_of("/\\");
            const auto dot = path.find_last_of('.');
            const bool hasExt = dot != std::string::npos && (slash == std::string::npos || dot > slash);
            return (hasExt ? path.substr(0, dot) : path) + "." + ext;
        }
    }

    ExportOpt loadExportOpt(const CliSource &ini, const Solvers::Mode mode) {
        using Solvers::Export::Format;
        ExportOpt o;

        const std::string fmtKey = ini.get("output", "format", "csv");
        const auto fmt = Solvers::Export::formatByKey(fmtKey);
        if (!fmt) {
            std::cerr << I18n::tr("cli.format.unknown", fmtKey) << "\n";
        } else if (!Solvers::Export::formatSolves(*fmt, mode)) {
            std::cerr << I18n::tr("cli.format.unsupported", Solvers::modeLabel(mode),
                                  Solvers::Export::formatName(*fmt)) << "\n";
        } else {
            o.spec.format = *fmt;
        }

        const double interval = ini.getD("output", "interval", 0.0);
        if (interval > 0.0) {
            o.spec.sampling.mode = Solvers::Export::Sampling::Mode::Seconds;
            o.spec.sampling.seconds = interval;
        }

        const int begin = ini.getInt("output", "begin", 0);
        const int end = ini.getInt("output", "end", 0);
        o.spec.begin = begin > 0 ? static_cast<std::size_t>(begin) : 0;
        o.spec.end = end > 0 ? static_cast<std::size_t>(end) : 0;

        applyFieldList(ini.get("output", "fields", ""), o.spec, "[output] fields");

        if (const std::string sat = ini.get("output", "sp3_sat", "L01"); sat.size() == 3) o.spec.satId = sat;
        else if (o.spec.format == Format::SP3)
            std::cerr << I18n::tr("cli.sp3_sat.bad", sat) << "\n";
        o.spec.marker = ini.get("output", "sp3_marker", "gnssLab");
        o.spec.sp3Velocity = ini.getB("output", "sp3_velocity", false);

        const std::string posForm = ini.get("output", "pos_form", "blh");
        if (posForm == "ecef") o.spec.posForm = Solvers::Export::PosForm::ECEF;
        else if (posForm == "enu") o.spec.posForm = Solvers::Export::PosForm::ENU;
        else if (posForm != "blh")
            std::cerr << I18n::tr("cli.pos_form.unknown", posForm) << "\n";
        o.spec.posVelocity = ini.getB("output", "pos_velocity", true);
        o.spec.posHeader = ini.getB("output", "pos_header", true);

        o.path = ini.get("output", "out", "");
        return o;
    }

    void dumpTask(Solvers::Task &task, const SolverOptions &cfg, const ExportOpt &opt) {
        std::string path = opt.path;
        // 显式给的 out 也要跟着最终格式换后缀：--format 可能在读完 ini 之后才覆盖。
        if (!path.empty()) path = withExt(path, Solvers::Export::fileExt(opt.spec.format));
        if (path.empty()) {
            path = cfg.csvPath.empty() ? defaultCSVName(task.mode) : cfg.csvPath;
            if (opt.spec.format != Solvers::Export::Format::CSV) {
                const std::size_t dot = path.find_last_of('.');
                if (dot != std::string::npos) path.resize(dot);
                if (opt.spec.format == Solvers::Export::Format::ORB) path += "_elements";
                if (opt.spec.format == Solvers::Export::Format::TRAJ ||
                    opt.spec.format == Solvers::Export::Format::KML)
                    path += "_traj";
                path += std::string(".") + Solvers::Export::fileExt(opt.spec.format);
            }
        }
        // 只有 SBET 开二进制：裸双精度里的 0x0A 会被文本模式翻成 0x0D 0x0A，其余格式保持逐字节不变。
        const std::size_t n = task.resultEpochCount();
        std::string error;
        if (!File::safeWrite(path, [&](std::ostream &out) {
                std::lock_guard lk(task.mutex);
                task.exportTo(out, opt.spec);
            }, error, opt.spec.format == Solvers::Export::Format::SBET)) {
            std::cerr << error << "\n";
            return;
        }
        std::cout << I18n::tr("cli.export.done", path, Solvers::Export::formatName(opt.spec.format), n) << "\n";
    }

    /// 命令行上写清这次到底用哪套参考真值：静默降级最容易让人对着空偏差列找 bug。
    std::string refText(const Solvers::Task &task, const SolverOptions &cfg) {
        switch (task.truth.form) {
            case RefForm::Fixed: {
                const auto fmt = [](const double v) {
                    std::ostringstream s;
                    s << std::fixed << std::setprecision(3) << v;
                    return s.str();
                };
                return I18n::tr("cli.ref.fixed_text", fmt(cfg.refX), fmt(cfg.refY), fmt(cfg.refZ));
            }
            case RefForm::File:
                return I18n::tr("cli.ref.file_text", cfg.refSp3);
            default:
                return I18n::tr("cli.ref.none_text");
        }
    }

    int runTask(Solvers::Task &task, const SolverOptions &cfg, const ExportOpt &opt) {
        task.maxEpochs = cfg.maxEpochs;
        task.realtimeSec = cfg.realtimeSec;

        task.setupTruth(cfg);
        std::cout << I18n::tr("cli.ref.line", refText(task, cfg)) << "\n";
        if (task.truth.form == RefForm::File && !std::filesystem::exists(cfg.refSp3))
            std::cerr << I18n::tr("cli.ref.file_missing", cfg.refSp3) << "\n";

        if (cfg.source == "realtime") {
            std::atomic<bool> quitWaiter{false};
            std::thread waiter;
            // 没给时限也没给历元上限时，回车停止（与 GUI 的取消按钮等价）。轮询键盘而不是阻塞
            // 在 std::cin.get() 上：流自己读完时 join 才不会永久卡住。
            if (cfg.realtimeSec <= 0 && cfg.maxEpochs <= 0)
                waiter = std::thread([&task, &quitWaiter] {
                    while (!quitWaiter.load()) {
                        if (_kbhit()) {
                            _getch();
                            task.cancel();
                            return;
                        }
                        Clock::sleep(Clock::milliseconds(50));
                    }
                });
            task.startRealtime();
            task.waitDone();
            quitWaiter = true;
            if (waiter.joinable()) waiter.join();
        } else {
            task.startFile();
            task.waitDone();
        }

        if (task.errorFlag()) std::cerr << I18n::tr("cli.task.error", task.errorMessage()) << "\n";
        dumpTask(task, cfg, opt);
        return task.errorFlag() ? 1 : 0;
    }
}
