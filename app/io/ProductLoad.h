#pragma once

#include <functional>
#include <future>
#include <string>
#include <vector>

#include "Log.h"
#include "SolverOptions.h"
#include "core/IniFile.h"
#include "StringUtils.h"
#include "Troposphere.h"

/// 精密产品的读取与派发
namespace ProductLoad {
    struct Paths {
        std::string sp3, clk, brdc, osb, att, erp, atx;
    };

    struct Defaults {
        Paths paths;
        std::string tro, ionex, met, vmf, gpt3;
    };

    inline std::function<Defaults(const std::string &scope)> &defaultsProvider() {
        static std::function<Defaults(const std::string &scope)> provider;
        return provider;
    }

    /// 显式优先、缺项回落该模式的默认。
    inline Defaults resolve(const SolverOptions &opts, const std::string &scope) {
        const auto [paths, tro, ionex, met, vmf, gpt3] = defaultsProvider() && !scope.empty()
                                                             ? defaultsProvider()(scope)
                                                             : Defaults{};
        const auto pick = [](const std::string &explicitPath, const std::string &dflt) {
            return explicitPath.empty() ? dflt : explicitPath;
        };
        Defaults r;
        r.paths.sp3 = pick(opts.sp3, paths.sp3);
        r.paths.clk = pick(opts.clk, paths.clk);
        r.paths.brdc = pick(opts.navPath, paths.brdc);
        r.paths.osb = pick(opts.osb, paths.osb);
        r.paths.att = pick(opts.att, paths.att);
        r.paths.erp = pick(opts.erp, paths.erp);
        r.paths.atx = pick(opts.atx, paths.atx);
        r.tro = pick(opts.tro, tro);
        r.ionex = pick(opts.ionex, ionex);
        r.met = pick(opts.met, met);
        r.vmf = pick(opts.vmf, vmf);
        r.gpt3 = pick(opts.gpt3, gpt3);
        return r;
    }

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

    /// SPP 对流层格网（VMF3 可多文件，kIniListSep 分隔 + GPT3），空路径跳过；任一加载失败返回 false
    inline bool loadSPPGrids(SPP &spp, const std::string &vmf, const std::string &gpt3) {
        bool ok = true;
        if (!vmf.empty() && !spp.loadVMF3(Str::split(vmf, kIniListSep, false))) {
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
    void loadPPPExtras(Solver &solver, std::vector<std::future<bool> > &jobs, const Defaults &d) {
        launch(jobs, "PPP", "对流层 TRO", d.tro, solver, &Solver::loadTRO);
        launch(jobs, "PPP", "电离层 IONEX", d.ionex, solver, &Solver::loadIONEX);
        launch(jobs, "PPP", "气象文件 MET", d.met, solver, &Solver::loadMET);
        launch(jobs, "PPP", "映射函数格网 VMF3", d.vmf,
                [&solver, paths = Str::split(d.vmf, kIniListSep, false)] { return solver.loadVMF3(paths); });
        launch(jobs, "PPP", "对流层格网 GPT3", d.gpt3, solver, &Solver::loadGPT3);
    }

    inline bool fail(Solvers::Task *task) {
        task->setError("产品加载失败：需(精密轨道SP3 或 广播星历BRDC)至少其一提供卫星位置；"
            "另检查 SP3/CLK/OSB/ATX 等路径是否正确");
        return false;
    }

    inline bool requireSatellite(const char *tag, const std::string &sp3, const std::string &brdc) {
        if (!sp3.empty() || !brdc.empty()) return true;
        LOG_ERROR << "[" << tag << "] 既未填精密轨道 SP3 也未填广播星历 BRDC：无法获取卫星位置，全历元无解";
        return false;
    }

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
    void core(Task &task, Solver &solver, std::vector<std::future<bool> > &jobs, const char *tag,
              const std::string &scope) {
        const Defaults d = resolve(task.core->opts, scope);
        if (task.core->opts.spp.applyAtx && d.paths.atx.empty())
            LOG_WARN << "[" << tag << "] 未检测到 ATX 天线文件，本次按无 PCO/PCV 处理（参数页「天线改正」可点获取）";
        core(solver, jobs, tag, d.paths, task.core->enabledSystems);
    }
}
