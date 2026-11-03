#include "Products.h"

#include "PathMemory.h"
#include "io/ProductPicker.h"
#include "StringUtils.h"
#include "download/ProductMeta.h"

#include <map>

namespace AppConfig {
    namespace {

        std::string joinPaths(const std::vector<std::string> &v) {
            std::string s;
            for (const auto &x: v) {
                if (!s.empty()) s += ';';
                s += x;
            }
            return s;
        }
    }

    Products &Products::instance() {

        static Products p = [] {
            Products q;
            q.load();
            return q;
        }();
        return p;
    }

    std::vector<std::string> Products::VMF3Paths() const {
        return Str::split(vmf, ';', false);
    }

    void Products::load() {
        const auto &pm = PathMemory::instance();
        sp3 = pm.get("prod_sp3");
        clk = pm.get("prod_clk");
        brdc = pm.get("prod_brdc");
        osb = pm.get("prod_osb");
        erp = pm.get("prod_erp");
        tro = pm.get("prod_tro");
        ionex = pm.get("prod_ionex");
        vmf = pm.get("prod_vmf");
        att = pm.get("prod_att");
        met = pm.get("prod_met");

        files = Str::split(pm.get("prod_files"), ';', false);
        if (!files.empty()) syncFromFiles(); // 以清单为准派生，避免 ini 与清单各说一套
    }

    void Products::save() const {
        const auto &pm = PathMemory::instance();
        pm.set("prod_sp3", sp3);
        pm.set("prod_clk", clk);
        pm.set("prod_brdc", brdc);
        pm.set("prod_osb", osb);
        pm.set("prod_erp", erp);
        pm.set("prod_tro", tro);
        pm.set("prod_ionex", ionex);
        pm.set("prod_vmf", vmf);
        pm.set("prod_att", att);
        pm.set("prod_met", met);
        pm.set("prod_files", joinPaths(files));
    }

    void Products::syncFromFiles() {
        std::map<ProductType, std::vector<std::string> > byType;
        for (const auto &f: files) {
            if (const auto m = ProductMeta::parse(Str::baseName(f))) byType[m->type].push_back(f);
        }
        const auto of = [&byType](const ProductType t) -> std::vector<std::string> {
            const auto it = byType.find(t);
            return it == byType.end() ? std::vector<std::string>{} : it->second;
        };

        sp3 = ProductPicker::bestPath(of(ProductType::ORB), ProductType::ORB);
        clk = ProductPicker::bestPath(of(ProductType::CLK), ProductType::CLK);
        brdc = ProductPicker::bestPath(of(ProductType::NAV), ProductType::NAV);
        osb = ProductPicker::bestPath(of(ProductType::OSB), ProductType::OSB);
        erp = ProductPicker::bestPath(of(ProductType::ERP), ProductType::ERP);
        tro = ProductPicker::bestPath(of(ProductType::TRO), ProductType::TRO);
        ionex = ProductPicker::bestPath(of(ProductType::ION), ProductType::ION);
        att = ProductPicker::bestPath(of(ProductType::ATT), ProductType::ATT);
        met = ProductPicker::bestPath(of(ProductType::MET), ProductType::MET);

        vmf = joinPaths(ProductPicker::bestPaths(of(ProductType::VMF), ProductType::VMF));
    }
}
