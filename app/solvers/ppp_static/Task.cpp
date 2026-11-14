#include "Task.h"

#include <future>
#include <memory>
#include <string>
#include <vector>

#include "SPP.h"
#include "StringUtils.h"
#include "core/atmo/Troposphere.h"
#include "io/ProductLoad.h"
#include "solver/ppp/Core.h"

namespace Solvers::PPPStatic {
    void Task::applyConfig(const SolverOptions &cfg) {
        opts = cfg;
        setupTruth(cfg);
        applyObsPath(cfg.obsPath);
    }

    bool Task::initSolver(::SPP &solver) {
        std::vector<std::future<bool> > jobs;
        auto &ppp = dynamic_cast<PPP::Core &>(solver);
        const auto products = ProductLoad::resolve(opts, modeKey(mode));
        ProductLoad::core(ppp, jobs, "PPP", products.paths, opts.systems);
        ProductLoad::loadPPPExtras(ppp, jobs, products);
        for (auto &j: jobs) j.get();
        orbitFrame = ppp.orbitFrame;
        hasOrbitFrame = ppp.hasOrbitFrame;
        const bool srcOk = ProductLoad::requireSatellite("PPP", products.paths.sp3, products.paths.brdc);
        ProductLoad::warnTropoFallback("PPP", opts.spp.tropoMap, ppp.tropoGridReady());
        return srcOk ? true : ProductLoad::fail(this);
    }

    std::unique_ptr<::SPP> Task::makeSolver() {
        return std::make_unique<PPP::Core>();
    }

    std::vector<Export::Col> Task::exportFields(const Export::Format fmt) const {
        if (fmt == Export::Format::ORB) return {}; // 根数只有低轨轨道有
        const std::vector<Export::Col> c = Export::floatFixColumns(false);
        return exportRefReady() ? c : Export::withoutRefColumns(c);
    }

    std::vector<Export::Col> Task::exportDefaults(const Export::Format fmt) const {
        return fmt == Export::Format::CSV ? exportFields(fmt) : std::vector<Export::Col>{};
    }

    void Task::writeCsvParams(CSV::Meta &m) const {
        m.num("码 σ (m)", opts.spp.codeSigma);
        m.num("相位 σ (m)", opts.spp.phaseSigma);
        m.num("高度角加权下限 (sinE)", opts.spp.minSinElev);
        m.flag("天线改正", opts.spp.applyAtx);
        m.flag("相位缠绕", opts.spp.applyWindup);
        m.flag("固体潮", opts.spp.applySolidTide);
        m.integer("滤波更新最少卫星数", opts.spp.minSatsForUpdate);
        m.num("钟差过程 σ", opts.spp.clockProcSigma);
        m.flag("使用 AR 固定", opts.spp.useAR);
        m.num("模糊度初始 σ", opts.spp.ambInitSigma);
        m.num("模糊度过程 σ", opts.spp.ambProcSigma);
        m.num("Ratio 阈值", opts.spp.arRatioThreshold);
        writeTropoParams(m);
        m.num("码偏过程 σ", opts.spp.codeBiasProcSigma);
        m.num("码偏初始 σ", opts.spp.codeBiasInitSigma);
        m.num("GLO IFCB 过程 σ", opts.spp.gloIfcbProcSigma);
        m.num("GLO IFCB 初始 σ", opts.spp.gloIfcbInitSigma);
        m.num("位置过程 σ", opts.posProcSigma);
        m.num("首历元位置 σ", opts.spp.firstEpochPosSigma);
        writeMotionParams(m);
        m.flag("使用周跳探测", opts.spp.useCycleSlip);
        m.num("GF 周跳阈值倍数", opts.spp.gfSlipMult);
        m.num("ISB 过程 σ", opts.spp.isbProcSigma);
        m.num("ISB 初始 σ", opts.spp.isbInitSigma);
        m.flag("剔除电文标记不健康的卫星", opts.spp.useHealth);
    }

    void Task::writeTropoParams(CSV::Meta &m) const {
        m.flag("对流层梯度", opts.spp.useTroGrad);
        m.text("映射函数", Troposphere::keyOf(opts.spp.tropoMap));
        m.num("ZTD 初值 (m)", opts.spp.ztdInitValue);
        m.num("ZTD 初始 σ", opts.spp.ztdInitSigma);
        m.num("ZTD 过程 σ", opts.spp.ztdProcSigma);
        m.num("ZTD 先验 σ", opts.spp.ztdPriorSigma);
        m.num("ZTD 约束 σ", opts.spp.troZtdSigma);
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->applyConfig(cfg);
        return task;
    }
}
