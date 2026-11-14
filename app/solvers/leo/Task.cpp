#include "Task.h"
#include "FileUtils.h"

#include <future>
#include <memory>
#include <string>
#include <vector>

#include "LEO.h"
#include "Observer.h"
#include "SPP.h"
#include "StringUtils.h"
#include "io/ProductLoad.h"

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
            antenna.comToArp = {Eigen::Vector3d(v[0], v[1], v[2]), true, "配置项 leo_ant"};
        if (Str::scanMeters(cfg.leoPco.c_str(), v, 3) == 3)
            antenna.pco = {Eigen::Vector3d(v[0], v[1], v[2]), true, "配置项 leo_pco"};
        for (const auto &f: auxFiles) {
            if (const auto [comToArp, pco] = Antenna::fromHeaderFile(f); comToArp.known) {
                antennaDef.comToArp = comToArp;
                antennaDef.comToArp.from += "（aux 指定）";
                break;
            }
        }
        antennaObsPath = cfg.obsPath;
        // LEO 的 GNSS 产品与观测分属两套路径，统一收进 opts 供 ProductLoad 取用
        opts.sp3 = cfg.gnssSp3;
        opts.clk = cfg.gnssClk;
        opts.navPath = cfg.gnssBrdc;
    }

    bool Task::initSolver(::SPP &solver) {
        auto &leo = static_cast<::LEO &>(solver);
        bool loadOk = true;

        {
            std::vector<std::future<bool> > jobs;
            ProductLoad::core(*this, leo, jobs, "LEO", modeKey(Mode::LEO));
            for (auto &j: jobs) {
                if (!j.get()) loadOk = false;
            }
        }
        {
            const auto [comToArp, pco] = Antenna::merge(antenna, antennaDef);
            ::LEO::AntennaOffset offset;
            offset.comToArp = comToArp.xyz;
            offset.pco = pco.xyz;
            offset.hasArp = comToArp.known;
            offset.hasPco = pco.known;
            std::string src = comToArp.known ? comToArp.from : std::string("质心→ARP 未定");
            if (pco.known && pco.from != comToArp.from) src += "；" + pco.from;
            leo.setAntennaOffset(offset, src);
        }
        // 解所在框架就是产品所在框架：记下来供界面与导出标注。
        orbitFrame = leo.orbitFrame;
        hasOrbitFrame = leo.hasOrbitFrame;

        const auto products = ProductLoad::resolve(opts, modeKey(Mode::LEO));
        const bool srcOk = ProductLoad::requireSatellite("LEO", products.paths.sp3, products.paths.brdc);
        return loadOk && srcOk ? true : ProductLoad::fail(this);
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
        return exportRefReady() ? c : Export::withoutRefColumns(c);
    }

    std::vector<Export::Col> Task::exportDefaults(const Export::Format fmt) const {
        if (fmt == Export::Format::ORB) return orbCols();
        return fmt == Export::Format::CSV ? exportFields(fmt) : std::vector<Export::Col>{};
    }

    void Task::writeMotionParams(CSV::Meta &m) const {
        m.num("运动检测阈值", opts.motionThresh);
        m.num("SPP 预测阈值", opts.sppVsPredThresh);
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->applyConfig(cfg);
        return task;
    }
}
