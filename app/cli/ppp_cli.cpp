// PPP 无 GUI 命令行解算（静态 / 运动学）。用法：ppp_cli [ini文件]
// 默认读取同目录 ppp.ini。
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Exception.h"         // EndOfFile
#include "GnssStruct.h"        // ObsData
#include "SolverTask.h"        // GnssTask::EpochData / SolverMode
#include "SPP.h"
#include "PPPStatic.h"
#include "PPPKinematic.h"
#include "RinexObsReader.h"
#include "Log.h"
#include "core/SolveConfig.h"
#include "CliCommon.h"
#include "SolveDriver.h"        // 共享解算编排：ObsSource / solveStationStream / fillFrom*
#include "core/IniFile.h"

namespace {

void runPpp(const SolveConfig &cfg) {
    std::vector<GnssTask::EpochData> epochs;

    auto init = [&](auto &ppp) -> bool {
        using SolverT = std::decay_t<decltype(ppp)>;
        if (!cfg.sp3.empty())  ppp.loadSp3(cfg.sp3);
        if (!cfg.clk.empty())  ppp.loadClk(cfg.clk, cfg.systems);
        if (!cfg.navPath.empty()) ppp.loadBrdc(cfg.navPath);
        if (!cfg.atx.empty())  ppp.loadAtx(cfg.atx);
        if (!cfg.osb.empty())  ppp.loadOsb(cfg.osb);
        if (!cfg.erp.empty())  ppp.loadErp(cfg.erp);
        if (!cfg.tro.empty())  ppp.loadTro(cfg.tro);
        if (!cfg.gpt2.empty()) ppp.loadGpt2(cfg.gpt2); // GPT2 先验 ZWD：TRO 缺站时保高度收敛
        if (!cfg.ionex.empty()) ppp.loadIonex(cfg.ionex);
        // 极潮/对流层ZTD/高阶电离层改正：产品文件加载即应用（"文件为准"，无需独立开关）
        ppp.mApplySatPCO = cfg.satPCO != 0;
        ppp.mApplyPcv   = cfg.satPCV != 0;
        ppp.mApplyRcvPCO = cfg.rcvPCO != 0;
        ppp.mApplySolidTide = cfg.solidTide != 0;
        ppp.setMeasNoise(cfg.sigIFCode, cfg.sigPhase);
        ppp.setFirstEpochPosVar(cfg.firstEpochPosVar); // 静态/运动学都生效：过大先验会拖慢收敛
        ppp.mFirstEpochSPP = cfg.firstEpochSPP != 0; // 首历元 SPP 引导(harness 曾开，CLI 原先无法配置)
        // 下发 ZTD 初值/初方差（setTropo 会连带设 ztdQ，故传入当前 ztdQ 以免被默认值覆盖）
        ppp.setTropo(true, cfg.ztdInitValue, cfg.ztdInitVar, cfg.ztdQ);
        ppp.setAmbInitVar(cfg.ambInitVar); // 模糊度初方差(RTKLIB=3600)
        ppp.setZtdProcNoise(cfg.ztdQ);  // ZTD 过程噪声：让 ZTD 跟住日变化，避免 U 方向数小时摆动
        ppp.setZtdPriorSigma(cfg.ztdPrior);  // TRO 缺站时关闭先验(设大值)，改由观测估计 ZTD，避免日变湿延迟漏进 U
        if (cfg.kinematic) ppp.setPosProcNoise(cfg.posNoise); // 运动学：白噪声过程噪声
        else ppp.setPosProcNoise(cfg.staticPosNoise); // 静态：0=冻结(历史)；非0=对齐 RTKLIB 每历元微增(加速收敛)
        ppp.setEnabledSystems(cfg.systems);  // 星座筛选须显式下发：仅 loadClk 过滤钟差不会剔除卫星
        ppp.setCutoffElevDeg(cfg.cutoffDeg); // 截止高度角同样须显式下发，否则恒用默认 10°
        ppp.setAmbProcNoise(cfg.ambQ);       // 模糊度过程噪声旋钮（默认 1e-8 近似冻结）
        ppp.setTroZtdSigma(cfg.troZtdSigma); // TRO ZTD 约束 sigma（调大=ZTD 自由估计）
        ppp.mUseAR = cfg.useAR != 0;         // 模糊度固定开关（use_ar=0 则纯浮点）
        ppp.mUseTroGrad = cfg.useTroGrad != 0; // TRO 水平梯度固定改正开关（use_tro_grad=0 退化为仅 ZTD）
        ppp.mTropoMap = cfg.tropoMap; // 对流层映射函数 gmf(默认)/nmf
        ppp.setWindup(cfg.applyWindup != 0); // 相位缠绕改正
        return true;
    };

    Vector3d ref(0, 0, 0);

    if (cfg.obsPath.empty()) {
        std::cerr << "PPP 需要观测文件 [input] obs\n";
        Cli::dumpEmpty(GnssTask::SolverMode::Ppp, cfg);
        return;
    }
    SolveDriver::RinexFileSource src;
    if (!src.open(cfg.obsPath)) {
        std::cerr << "PPP 无法打开 RINEX: " << cfg.obsPath << "\n";
        Cli::dumpEmpty(GnssTask::SolverMode::Ppp, cfg);
        return;
    }

    if (cfg.kinematic) {
        PPPKinematic k;
        if (init(k)) epochs = SolveDriver::solveAll(src, k);
    } else {
        PPPStatic s;
        if (init(s)) epochs = SolveDriver::solveAll(src, s);
    }

    if (cfg.hasRef) ref = Vector3d(cfg.refX, cfg.refY, cfg.refZ);
    else SolveDriver::firstSolvedRef(epochs, ref);
    Cli::dumpCsv(epochs, GnssTask::SolverMode::Ppp, ref, false, cfg);
}

} // namespace

int main(int argc, char **argv) {
    std::string ini = (argc > 1) ? argv[1] : "ppp.ini";
    IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    SolveConfig cfg = loadConfig(f);
    Log::init("cli_gnsslab.log");
    std::cout << "[PPP] obs=" << cfg.obsPath
              << " 运动学=" << (cfg.kinematic ? "是" : "否") << "\n";
    runPpp(cfg);
    return 0;
}
