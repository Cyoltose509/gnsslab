#include "Products.h"
#include "FileUtils.h"

#include "core/PathMemory.h"
#include "ProductPicker.h"
#include "StringUtils.h"
#include "download/local/ProductMeta.h"

#include <map>
#include <mutex>

namespace AppConfig {
    namespace {
        std::mutex g_poolMtx;

        /// 每模式一份产品池：工作线程（ProductLoad 在解算线程跑）与界面线程会同时取，插入必须串行。
        std::map<std::string, Products> &pool() {
            static std::map<std::string, Products> p;
            return p;
        }
    }

    Products::Products(std::string scope) : scopeName(std::move(scope)) {
    }

    Products &Products::of(const std::string &scope) {
        std::lock_guard lk(g_poolMtx);
        auto &p = pool();
        if (const auto it = p.find(scope); it != p.end()) return it->second;
        return p.emplace(scope, Products(scope)).first->second;
    }

    std::string Products::key(const char *name) const {
        return scopeName + "_" + name;
    }

    std::vector<std::string> Products::VMF3Paths() const {
        return Str::split(vmf, kIniListSep, false);
    }

    void Products::load() {
        const auto &pm = PathMemory::instance();
        sp3 = pm.get(key("sp3"));
        clk = pm.get(key("clk"));
        brdc = pm.get(key("brdc"));
        osb = pm.get(key("osb"));
        erp = pm.get(key("erp"));
        tro = pm.get(key("tro"));
        ionex = pm.get(key("ionex"));
        vmf = pm.get(key("vmf"));
        att = pm.get(key("att"));

        files = Str::split(pm.get(key("files")), kIniListSep, false);
        if (!files.empty()) syncFromFiles(); // 以清单为准派生，避免 ini 与清单各说一套
    }

    void Products::save() const {
        const auto &pm = PathMemory::instance();
        pm.set(key("sp3"), sp3);
        pm.set(key("clk"), clk);
        pm.set(key("brdc"), brdc);
        pm.set(key("osb"), osb);
        pm.set(key("erp"), erp);
        pm.set(key("tro"), tro);
        pm.set(key("ionex"), ionex);
        pm.set(key("vmf"), vmf);
        pm.set(key("att"), att);
        pm.set(key("files"), Str::join(files, kIniListSep));
    }

    void Products::syncFromFiles() {
        std::map<ProductType, std::vector<std::string> > byType;
        for (const auto &f: files) {
            if (const auto m = ProductMeta::parse(File::baseName(f))) byType[m->type].push_back(f);
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
        brdc = joinAll(pickOf(ProductType::NAV));
        osb = joinAll(pickOf(ProductType::OSB));
        erp = joinAll(pickOf(ProductType::ERP));
        att = joinAll(pickOf(ProductType::ATT));
        tro = ProductPicker::bestPath(pickOf(ProductType::TRO), ProductType::TRO);
        ionex = ProductPicker::bestPath(pickOf(ProductType::ION), ProductType::ION);

        vmf = Str::join(ProductPicker::bestPaths(pickOf(ProductType::VMF), ProductType::VMF), kIniListSep);
    }
}
