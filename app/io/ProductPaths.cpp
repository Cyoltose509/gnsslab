#include "ProductPaths.h"
#include "FileUtils.h"

#include "core/PathMemory.h"
#include "download/local/Store.h"
#include "StringUtils.h"
#include "download/local/FileMeta.h"
#include "download/local/Store.h"
#include "download/remote/Catalog.h"

#include <map>
#include <mutex>

namespace AppConfig {
    namespace {
        std::mutex g_poolMtx;

        /// 每模式一份产品池：工作线程（ProductReader 在解算线程跑）与界面线程会同时取，插入必须串行。
        std::map<std::string, ProductPaths> &pool() {
            static std::map<std::string, ProductPaths> p;
            return p;
        }
    }

    ProductPaths::ProductPaths(std::string scope) : scopeName(std::move(scope)) {
    }

    ProductPaths &ProductPaths::of(const std::string &scope) {
        std::lock_guard lk(g_poolMtx);
        auto &p = pool();
        return p.try_emplace(scope, scope).first->second;
    }

    std::string ProductPaths::key(const char *name) const {
        return scopeName + "_" + name;
    }

    std::vector<std::string> ProductPaths::VMF3Paths() const {
        return Str::split(vmf, kIniListSep, false);
    }

    void ProductPaths::load() {
        const auto &pm = PathMemory::instance();
        nav = pm.get(key("nav"));
        sp3 = pm.get(key("sp3"));
        clk = pm.get(key("clk"));
        osb = pm.get(key("osb"));
        erp = pm.get(key("erp"));
        tro = pm.get(key("tro"));
        ion = pm.get(key("ion"));
        vmf = pm.get(key("vmf"));
        att = pm.get(key("att"));
        // 版本型产品没有日期目录，取下载库里当前安装的那一份
        atx = Download::Store::activeFile(Download::Catalog::kATXID);
        gpt3 = Download::Store::activeFile(Download::Catalog::kGPT3ID);

        files = Str::split(pm.get(key("files")), kIniListSep, false);
        if (!files.empty()) syncFromFiles(); // 以清单为准派生，避免 ini 与清单各说一套
    }

    void ProductPaths::save() const {
        const auto &pm = PathMemory::instance();
        pm.set(key("sp3"), sp3);
        pm.set(key("clk"), clk);
        pm.set(key("nav"), nav);
        pm.set(key("osb"), osb);
        pm.set(key("erp"), erp);
        pm.set(key("tro"), tro);
        pm.set(key("ion"), ion);
        pm.set(key("vmf"), vmf);
        pm.set(key("att"), att);
        pm.set(key("files"), Str::join(files, kIniListSep));
    }

    void ProductPaths::syncFromFiles() {
        std::map<ProductType, std::vector<std::string> > byType;
        for (const auto &f: files) {
            if (const auto m = FileMeta::parse(File::baseName(f))) byType[m->type].push_back(f);
        }
        const auto pickOf = [&byType](const ProductType t) -> std::vector<std::string> {
            const auto it = byType.find(t);
            return it == byType.end() ? std::vector<std::string>{} : it->second;
        };

        // 时间序列产品（SP3/CLK/BRDC/OSB/ATT/ERP）全部保留：解算器会按历元合并相邻多日多份，
        // 覆盖跨 UTC 午夜的低轨弧（像 VMF 保留整组时次文件那样）。ATX/TRO/ION 是单版本文件，取最优一份。
        const auto joinAll = [](const std::vector<std::string> &v) {
            return v.empty() ? std::string() : Str::join(v, kIniListSep);
        };
        sp3 = joinAll(pickOf(ProductType::ORB));
        clk = joinAll(pickOf(ProductType::CLK));
        nav = joinAll(pickOf(ProductType::NAV));
        osb = joinAll(pickOf(ProductType::OSB));
        erp = joinAll(pickOf(ProductType::ERP));
        att = joinAll(pickOf(ProductType::ATT));
        tro = Download::Store::bestPath(pickOf(ProductType::TRO), ProductType::TRO);
        ion = Download::Store::bestPath(pickOf(ProductType::ION), ProductType::ION);

        vmf = Str::join(Download::Store::bestPaths(pickOf(ProductType::VMF), ProductType::VMF), kIniListSep);
    }

    ProductPaths ProductPaths::resolved(const SolverOptions &opts) const {
        const auto pick = [](const std::string &explicitPath, const std::string &fallback) {
            return explicitPath.empty() ? fallback : explicitPath;
        };
        ProductPaths r = *this;
        r.sp3 = pick(opts.sp3, sp3);
        r.clk = pick(opts.clk, clk);
        r.nav = pick(opts.navPath, nav);
        r.osb = pick(opts.osb, osb);
        r.att = pick(opts.att, att);
        r.erp = pick(opts.erp, erp);
        r.atx = pick(opts.atx, atx);
        r.tro = pick(opts.tro, tro);
        r.ion = pick(opts.ionex, ion);
        r.vmf = pick(opts.vmf, vmf);
        r.gpt3 = pick(opts.gpt3, gpt3);
        return r;
    }
}
