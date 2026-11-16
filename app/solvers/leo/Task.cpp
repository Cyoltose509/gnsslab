#include "Task.h"
#include "FileUtils.h"

#include <memory>
#include <string>
#include <vector>

#include "LEO.h"
#include "Observer.h"
#include "SPP.h"
#include "StringUtils.h"
#include "core/IniFile.h"
#include "io/ProductPaths.h"
#include "io/ProductReader.h"
#include "i18n/I18n.h"

namespace Solvers::LEO {
    void Task::applyConfig(const SolverOptions &cfg) {
        opts = cfg;
        applyObsPath(cfg.obsPath);
        setupTruth(cfg);
        auxFiles = Str::split(cfg.aux, kIniListSep, false);
        antenna = {};
        antennaArpText = {};
        antennaPcoText = {};
        antennaDef = Antenna::defaults(cfg.obsPath);
        double v[3] = {0.0, 0.0, 0.0};
        if (Str::scanMeters(cfg.leoAnt.c_str(), v, 3) == 3)
            antenna.comToArp = {Eigen::Vector3d(v[0], v[1], v[2]), true, I18n::tr("leo.ant.from_cfg_ant")};
        if (Str::scanMeters(cfg.leoPco.c_str(), v, 3) == 3)
            antenna.pco = {Eigen::Vector3d(v[0], v[1], v[2]), true, I18n::tr("leo.ant.from_cfg_pco")};
        for (const auto &f: auxFiles) {
            if (const auto [comToArp, pco] = Antenna::fromHeaderFile(f); comToArp.known) {
                antennaDef.comToArp = comToArp;
                antennaDef.comToArp.from += I18n::tr("leo.ant.aux");
                break;
            }
        }
        antennaObsPath = cfg.obsPath;
        // LEO 的 GNSS 产品与观测分属两套路径，统一收进 opts 供 ProductReader 取用
        opts.sp3 = cfg.gnssSp3;
        opts.clk = cfg.gnssClk;
        opts.navPath = cfg.gnssBrdc;
    }

    bool Task::initSolver(::SPP &solver) {
        auto &leo = dynamic_cast<::LEO &>(solver);
        const auto d = AppConfig::ProductPaths::of(modeKey(Mode::LEO)).resolved(opts);
        if (opts.spp.applyAtx && d.atx.empty())
            LOG_WARN << "[LEO] " << I18n::trc("pl.log.no_atx");
        // LEO 只吃这 7 类：没有对流层梯度、电离层图与格网，多读会改变解算结果
        const auto loaded = productsFor(d, "LEO", opts.systems,
                                        {ProductType::NAV, ProductType::ORB, ProductType::CLK,
                                         ProductType::OSB, ProductType::ATT, ProductType::ERP,
                                         ProductType::ATX});
        leo.prod = loaded.prod;
        {
            const auto [comToArp, pco] = Antenna::merge(antenna, antennaDef);
            ::AntennaOffset offset;
            offset.comToArp = comToArp.xyz;
            offset.pco = pco.xyz;
            offset.hasArp = comToArp.known;
            offset.hasPco = pco.known;
            std::string src = comToArp.known ? comToArp.from : I18n::tr("leo.ant.unknown");
            if (pco.known && pco.from != comToArp.from) src += "；" + pco.from;
            leo.setAntennaOffset(offset, src);
        }
        // 解所在框架就是产品所在框架：记下来供界面与导出标注。
        orbitFrame = leo.prod->orbitFrame;
        hasOrbitFrame = leo.prod->hasOrbitFrame;

        return loaded.readOk && requireOrbit(d.sp3, d.nav);
    }

    bool Task::afterSolveEpoch(::SPP &, Epoch &data, Observer::Item &, const int) {
        return data.solved && data.sppResult.xyz.squaredNorm() > 1e12;
    }

    std::unique_ptr<::SPP> Task::makeSolver() {
        return std::make_unique<::LEO>();
    }

    namespace {
        using F = Export::Field;
        /// 轨道根数：低轨独有的一种导出格式，六个根数各是独立量，不并组。
        std::vector<Export::Col> orbCols() {
            return {
                {F::Week, "Wk"}, {F::Tow, "SOW"},
                {F::SmA, "a/m"}, {F::Ecc, "e"}, {F::Inc, "i/deg"},
                {F::Raan, "RAAN/deg"}, {F::Argp, "argp/deg"}, {F::MAnom, "M/deg"},
                {F::PerAlt, "hp/km"}, {F::ApoAlt, "ha/km"}, {F::Period, "T/s"},
            };
        }
    }

    std::vector<Export::Col> Task::exportFields(const Export::Format fmt) const {
        if (fmt == Export::Format::ORB) return orbCols();
        // 低轨多一组 RTN（径向/切向/法向偏差），和 ENU 一样只在有真值时才有意义。
        const std::vector<Export::Col> c = Export::floatFixColumns(true);
        return columnsForTruth(c);
    }

    std::vector<Export::Col> Task::exportDefaults(const Export::Format fmt) const {
        if (fmt == Export::Format::ORB) return orbCols();
        return fmt == Export::Format::CSV ? exportFields(fmt) : std::vector<Export::Col>{};
    }

    void Task::writeCSVParams(CSV::Meta &m) const {
        PPPKinematic::Task::writeCSVParams(m);
        m.num(I18n::trc("param.motion_threshold"), opts.motionThresh);
        m.num(I18n::trc("param.spp_pred_threshold"), opts.sppVsPredThresh);
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->applyConfig(cfg);
        return task;
    }
}
