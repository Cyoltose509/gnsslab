#pragma once

#include <future>
#include <functional>
#include <string>
#include <vector>

#include "Log.h"
#include "StringUtils.h"
#include "Troposphere.h"
#include "solvers/common/Core.h"
#include "core/Products.h"
#include "download/ProductStore.h"
#include "download/ProductCatalog.h"

namespace ProductLoad {
    template<class Fn>
    void launch(std::vector<std::future<bool> > &jobs, const char *tag,
                const char *name, const std::string &path, Fn fn) {
        if (path.empty()) return;
        jobs.emplace_back(std::async(std::launch::async, [tag, name, path, fn] {
            const bool ok = fn();
            LOG_INFO << "[" << tag << "] 读取" << name << ": " << path << (ok ? "" : "[失败]");
            return ok;
        }));
    }

    template<class Obj, class MemObj, class Ret>
    void launch(std::vector<std::future<bool> > &jobs, const char *tag,
                const char *name, const std::string &path,
                Obj &obj, Ret (MemObj::*load)(const std::string &)) {
        launch(jobs, tag, name, path, [&obj, load, path] { return (obj.*load)(path); });
    }

    template<class Obj, class MemObj, class Ret, class Arg>
    void launch(std::vector<std::future<bool> > &jobs, const char *tag,
                const char *name, const std::string &path,
                Obj &obj, Ret (MemObj::*load)(const std::string &, const Arg &), const Arg &arg) {
        launch(jobs, tag, name, path, [&obj, load, &arg, path] { return (obj.*load)(path, arg); });
    }

    inline void warnTropoFallback(const char *tag, const Troposphere::Map map, const bool gridReady) {
        if (gridReady) return;
        LOG_WARN << "[" << tag << "] 已选对流层模型 " << Troposphere::nameOf(map)
                << " 但格网未就位，映射回落 GMF";
    }

    /// SPP 对流层格网（VMF3 分号分隔多文件 + GPT3），空路径跳过；任一加载失败返回 false
    inline bool loadSPPGrids(SPP &spp, const std::string &vmf, const std::string &gpt3) {
        bool ok = true;
        if (!vmf.empty() && !spp.loadVMF3(Str::split(vmf, ';', false))) {
            LOG_WARN << "[SPP] 读取 VMF3 格网失败: " << vmf;
            ok = false;
        }
        if (!gpt3.empty() && !spp.loadGPT3(gpt3)) {
            LOG_WARN << "[SPP] 读取 GPT3 格网失败: " << gpt3;
            ok = false;
        }
        return ok;
    }

    /// PPP 精密产品之外的附加产品（TRO/IONEX/MET/VMF3/GPT3），空路径跳过
    template<class Solver>
    void loadPPPExtras(Solver &solver, std::vector<std::future<bool> > &jobs,
                       const std::string &tro, const std::string &ionex, const std::string &met,
                       const std::string &vmf, const std::string &gpt3) {
        launch(jobs, "PPP", "对流层 TRO", tro, solver, &Solver::loadTRO);
        launch(jobs, "PPP", "电离层 IONEX", ionex, solver, &Solver::loadIONEX);
        launch(jobs, "PPP", "气象文件 MET", met, solver, &Solver::loadMET);
        launch(jobs, "PPP", "映射函数格网 VMF3", vmf,
                [&solver, paths = Str::split(vmf, ';', false)] { return solver.loadVMF3(paths); });
        launch(jobs, "PPP", "对流层格网 GPT3", gpt3, solver, &Solver::loadGPT3);
    }

    inline bool fail(Solvers::Task *core) {
        core->setError("产品加载失败：需(精密轨道SP3 或 广播星历BRDC)至少其一提供卫星位置；"
            "另检查 SP3/CLK/OSB/ATX 等路径是否正确");
        return false;
    }

    inline bool requireSatellite(const char *tag, const std::string &sp3, const std::string &brdc) {
        if (!sp3.empty() || !brdc.empty()) return true;
        LOG_ERROR << "[" << tag << "] 既未填精密轨道 SP3 也未填广播星历 BRDC：无法获取卫星位置，全历元无解";
        return false;
    }

    inline bool requireSatellite(const char *tag) {
        const auto &prod = AppConfig::Products::instance();
        return requireSatellite(tag, prod.sp3, prod.brdc);
    }

    struct Paths {
        std::string sp3, clk, brdc, osb, att, erp, atx;
    };

    template<class Solver, class SysT>
    void core(Solver &solver, std::vector<std::future<bool> > &jobs, const char *tag,
              const Paths &paths, const SysT &systems) {
        launch(jobs, tag, "精密轨道 SP3", paths.sp3, solver, &Solver::loadSP3);
        launch(jobs, tag, "精密钟差 CLK", paths.clk, solver, &Solver::loadCLK, systems);
        launch(jobs, tag, "广播星历 BRDC", paths.brdc, solver, &Solver::loadBRDC);
        launch(jobs, tag, "码偏差 OSB", paths.osb, solver, &Solver::loadOSB);
        launch(jobs, tag, "卫星姿态 ATT", paths.att, solver, &Solver::loadATT);
        launch(jobs, tag, "地球自转 ERP", paths.erp, solver, &Solver::loadERP);
        if (!paths.atx.empty())
            launch(jobs, tag, "天线文件 ATX", paths.atx, solver, &Solver::loadATX);
    }

    template<class Task, class Solver>
    void core(Task &task, Solver &solver, std::vector<std::future<bool> > &jobs, const char *tag) {
        const auto &prod = AppConfig::Products::instance();
        Paths paths{prod.sp3, prod.clk, prod.brdc, prod.osb, prod.att, prod.erp, {}};
        if (task.core->opts.spp.applyAtx) {
            paths.atx = Download::Store::activeFile(Download::Catalog::atx());
            if (paths.atx.empty())
                LOG_WARN << "[" << tag << "] 未检测到 ATX 天线文件，本次按无 PCO/PCV 处理（参数页「天线改正」可点获取）";
        }
        core(solver, jobs, tag, paths, task.core->enabledSystems);
    }
}
