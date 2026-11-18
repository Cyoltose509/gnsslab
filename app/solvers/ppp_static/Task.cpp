#include "Task.h"

#include <memory>
#include <string>
#include <vector>

#include "SPP.h"
#include "StringUtils.h"
#include "core/atmo/Troposphere.h"
#include "io/ProductPaths.h"
#include "io/ProductReader.h"
#include "solver/ppp/Static.h"
#include "i18n/I18n.h"

namespace Solvers::PPPStatic {
    void Task::applyConfig(const SolverOptions &cfg) {
        opts = cfg;
        setupTruth(cfg);
        applyObsPath(cfg.obsPath);
    }

    bool Task::initSolver(::SPP &solver) {
        auto &ppp = dynamic_cast<::PPPStatic &>(solver);
        const auto d = AppConfig::ProductPaths::of(modeKey(mode)).resolved(opts);
        // PPP 要全套精密产品：轨道/钟差/偏差/姿态/自转/天线/对流层/电离层 + 两张格网
        ppp.prod = productsFor(d, "PPP", opts.systems,
                               {ProductType::NAV, ProductType::ORB, ProductType::CLK, ProductType::OSB,
                                ProductType::ATT, ProductType::ERP, ProductType::ATX, ProductType::TRO,
                                ProductType::ION, ProductType::VMF, ProductType::GPT3})
                      .prod;
        orbitFrame = ppp.prod->orbitFrame;
        hasOrbitFrame = ppp.prod->hasOrbitFrame;
        Troposphere::warnFallback("PPP", opts.spp.tropoMap, ppp.tropoGridReady());
        return requireOrbit(d.sp3, d.nav);
    }

    std::unique_ptr<::SPP> Task::makeSolver() {
        return std::make_unique<::PPPStatic>();
    }

    std::vector<Export::Col> Task::exportFields(const Export::Format fmt) const {
        if (fmt == Export::Format::ORB) return {}; // 根数只有低轨轨道有
        const std::vector<Export::Col> c = Export::floatFixColumns(false);
        return columnsForTruth(c);
    }

    std::vector<Export::Col> Task::exportDefaults(const Export::Format fmt) const {
        return fmt == Export::Format::CSV ? exportFields(fmt) : std::vector<Export::Col>{};
    }

    void Task::writeCSVParams(CSV::Meta &m) const {
        writeCommonParams(m);
        m.num(I18n::trc("param.phase_sigma"), opts.spp.phaseSigma);
        m.flag(I18n::trc("AntennaCorrection"), opts.spp.applyAtx);
        m.flag(I18n::trc("param.phase_windup"), opts.spp.applyWindup);
        m.flag(I18n::trc("param.solid_tide"), opts.spp.applySolidTide);
        m.integer(I18n::trc("param.min_sats_update"), opts.spp.minSatsForUpdate);
        m.num(I18n::trc("param.clock_proc_sigma"), opts.spp.clockProcSigma);
        m.flag(I18n::trc("param.use_ar"), opts.spp.useAR);
        m.num(I18n::trc("param.amb_init_sigma"), opts.spp.ambInitSigma);
        m.num(I18n::trc("param.amb_proc_sigma"), opts.spp.ambProcSigma);
        m.num(I18n::trc("param.ratio_threshold"), opts.spp.arRatioThreshold);
        writeTropoParams(m);
        m.num(I18n::trc("param.code_bias_proc_sigma"), opts.spp.codeBiasProcSigma);
        m.num(I18n::trc("param.code_bias_init_sigma"), opts.spp.codeBiasInitSigma);
        m.num(I18n::trc("param.glo_ifcb_proc_sigma"), opts.spp.gloIfcbProcSigma);
        m.num(I18n::trc("param.glo_ifcb_init_sigma"), opts.spp.gloIfcbInitSigma);
        m.num(I18n::trc("param.pos_proc_sigma"), opts.posProcSigma);
        m.num(I18n::trc("param.first_epoch_pos_sigma"), opts.spp.firstEpochPosSigma);
        m.flag(I18n::trc("param.use_cycle_slip"), opts.spp.useCycleSlip);
        m.flag(I18n::trc("param.use_gf_slip"), opts.spp.useGFCycleSlip);
        m.num(I18n::trc("param.gf_slip_mult"), opts.spp.GFSlipMult);
    }

    void Task::writeTropoParams(CSV::Meta &m) const {
        m.flag(I18n::trc("param.tropo_gradient"), opts.spp.useTroGrad);
        m.text(I18n::trc("MappingFunction"), Troposphere::keyOf(opts.spp.tropoMap));
        m.num(I18n::trc("param.ztd_init"), opts.spp.ztdInitValue);
        m.num(I18n::trc("param.ztd_init_sigma"), opts.spp.ztdInitSigma);
        m.num(I18n::trc("param.ztd_proc_sigma"), opts.spp.ztdProcSigma);
        m.num(I18n::trc("param.ztd_prior_sigma"), opts.spp.ztdPriorSigma);
        m.num(I18n::trc("param.ztd_constraint_sigma"), opts.spp.troZtdSigma);
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->applyConfig(cfg);
        return task;
    }
}
