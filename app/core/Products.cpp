#include "Products.h"

#include "PathMemory.h"
#include "io/ProductPicker.h"
#include "StringUtils.h"
#include "download/ProductMeta.h"

#include <map>

namespace AppConfig {
    Products::Products(std::string scope) : scopeName(std::move(scope)) {
    }

    Products &Products::of(const std::string &scope) {
        static std::map<std::string, Products> pool;
        if (const auto it = pool.find(scope); it != pool.end()) return it->second;
        return pool.emplace(scope, Products(scope)).first->second;
    }

    std::string Products::key(const char *name) const {
        return scopeName + "_" + name;
    }

    std::vector<std::string> Products::VMF3Paths() const {
        return Str::split(vmf, kIniListSep, false);
    }

    std::vector<std::string> Products::recent() const {
        return PathMemory::instance().getRecent(key("files"));
    }

    void Products::addRecent(const std::string &path) const {
        PathMemory::instance().addRecent(key("files"), path);
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
        met = pm.get(key("met"));

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
        pm.set(key("met"), met);
        pm.set(key("files"), Str::join(files, kIniListSep));
    }

    void Products::syncFromFiles() {
        std::map<ProductType, std::vector<std::string> > byType;
        for (const auto &f: files) {
            if (const auto m = ProductMeta::parse(Str::baseName(f))) byType[m->type].push_back(f);
        }
        const auto pickOf = [&byType](const ProductType t) -> std::vector<std::string> {
            const auto it = byType.find(t);
            return it == byType.end() ? std::vector<std::string>{} : it->second;
        };

        sp3 = ProductPicker::bestPath(pickOf(ProductType::ORB), ProductType::ORB);
        clk = ProductPicker::bestPath(pickOf(ProductType::CLK), ProductType::CLK);
        brdc = ProductPicker::bestPath(pickOf(ProductType::NAV), ProductType::NAV);
        osb = ProductPicker::bestPath(pickOf(ProductType::OSB), ProductType::OSB);
        erp = ProductPicker::bestPath(pickOf(ProductType::ERP), ProductType::ERP);
        tro = ProductPicker::bestPath(pickOf(ProductType::TRO), ProductType::TRO);
        ionex = ProductPicker::bestPath(pickOf(ProductType::ION), ProductType::ION);
        att = ProductPicker::bestPath(pickOf(ProductType::ATT), ProductType::ATT);
        met = ProductPicker::bestPath(pickOf(ProductType::MET), ProductType::MET);

        vmf = Str::join(ProductPicker::bestPaths(pickOf(ProductType::VMF), ProductType::VMF), kIniListSep);
    }
}
