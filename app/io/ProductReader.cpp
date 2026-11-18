#include "ProductReader.h"

#include <algorithm>
#include <exception>
#include <future>
#include <vector>

#include "ProductPaths.h"
#include "StringUtils.h"
#include "core/IniFile.h"
#include "rinex/RINEXNavStore.h"

namespace {
    using Jobs = std::vector<std::future<bool>>;

    std::vector<std::string> splitPaths(const std::string &joined) {
        return Str::split(joined, kIniListSep, false);
    }

    /// 一个产品种类的全部读取知识：身份、路径取自 Paths 的哪一栏、怎么读、拆不拆多文件、能不能并行。
    struct Entry {
        ProductType type;
        std::string AppConfig::ProductPaths::*path;
        bool (*read)(Products &, const std::string &, const std::set<SatSystem> &, const std::string &tag);
        bool multiFile; // 路径串按 kIniListSep 拆成多个文件逐个读
        bool serial; // 独占一趟：与别的项写同一份数据的（星历表、对流层格网）不能并行
    };

    bool readBRDC(Products &p, const std::string &path, const std::set<SatSystem> &, const std::string &) {
        RINEXNavStore store;
        store.loadFile(path, p.eph);
        return true;
    }

    bool readSP3(Products &p, const std::string &path, const std::set<SatSystem> &, const std::string &tag) {
        const bool ok = p.sp3.read(path, true);
        // 解继承产品的参考框架：SP3 头写什么就用什么
        p.orbitFrame = p.sp3.frame;
        p.hasOrbitFrame = p.sp3.hasFrame;
        if (!p.hasOrbitFrame && ok)
            LOG_WARN << "[" << tag << "] " << I18n::tr("log.ppp.no_frame", path);
        return ok;
    }

    bool readCLK(Products &p, const std::string &path, const std::set<SatSystem> &systems, const std::string &) {
        return p.clk.read(path, systems, true);
    }

    bool readOSB(Products &p, const std::string &path, const std::set<SatSystem> &, const std::string &) {
        return p.osb.read(path);
    }

    bool readATT(Products &p, const std::string &path, const std::set<SatSystem> &, const std::string &) {
        return p.att.read(path);
    }

    bool readERP(Products &p, const std::string &path, const std::set<SatSystem> &, const std::string &) {
        return p.erp.read(path, true);
    }

    bool readATX(Products &p, const std::string &path, const std::set<SatSystem> &, const std::string &) {
        return p.atx.read(path);
    }

    bool readTRO(Products &p, const std::string &path, const std::set<SatSystem> &, const std::string &) {
        return p.tro.read(path);
    }

    bool readIONEX(Products &p, const std::string &path, const std::set<SatSystem> &, const std::string &) {
        return p.ion.read(path);
    }

    bool readVMF3(Products &p, const std::string &path, const std::set<SatSystem> &, const std::string &tag) {
        if (p.tropo.loadVMF3(splitPaths(path))) return true;
        LOG_WARN << "[" << tag << "] " << I18n::tr("pl.err.vmf3", path);
        return false;
    }

    bool readGPT3(Products &p, const std::string &path, const std::set<SatSystem> &, const std::string &tag) {
        if (p.tropo.loadGPT3(path)) return true;
        LOG_WARN << "[" << tag << "] " << I18n::tr("pl.err.gpt3", path);
        return false;
    }

    /// 一个产品一行：左列是产品身份，右列是它落在 Products/Paths 的哪一栏，中间没有第三方命名
    constexpr Entry kTable[] = {
        {ProductType::NAV, &AppConfig::ProductPaths::nav, readBRDC, true, true},
        {ProductType::ORB, &AppConfig::ProductPaths::sp3, readSP3, true, false},
        {ProductType::CLK, &AppConfig::ProductPaths::clk, readCLK, true, false},
        {ProductType::OSB, &AppConfig::ProductPaths::osb, readOSB, true, false},
        {ProductType::ATT, &AppConfig::ProductPaths::att, readATT, true, false},
        {ProductType::ERP, &AppConfig::ProductPaths::erp, readERP, true, false},
        {ProductType::ATX, &AppConfig::ProductPaths::atx, readATX, false, false},
        {ProductType::TRO, &AppConfig::ProductPaths::tro, readTRO, false, false},
        {ProductType::ION, &AppConfig::ProductPaths::ion, readIONEX, false, false},
        {ProductType::VMF, &AppConfig::ProductPaths::vmf, readVMF3, false, true},
        {ProductType::GPT3, &AppConfig::ProductPaths::gpt3, readGPT3, false, true},
    };

    constexpr bool uniqueTypes() {
        for (size_t i = 0; i < std::size(kTable); ++i)
            for (size_t j = i + 1; j < std::size(kTable); ++j)
                if (kTable[i].type == kTable[j].type) return false;
        return true;
    }

    static_assert(uniqueTypes(), "kTable has duplicate product type");

    /// 读一个文件并落日志；异常只废本项
    bool readOne(const Entry &e, Products &p, const std::string &tag, const std::string &path,
                 const std::set<SatSystem> &systems) {
        bool ok = false;
        try {
            ok = e.read(p, path, systems, tag);
        } catch (const std::exception &ex) {
            LOG_ERROR << "[" << tag << "] " << I18n::tr("pl.err.read", typeLabel(e.type), ex.what());
        }
        LOG_INFO << "[" << tag << "] " << I18n::tr("pl.log.read", typeLabel(e.type), path)
                 << (ok ? "" : " [FAIL]");
        return ok;
    }
}

namespace ProductReader {
    std::vector<Info> describe(const AppConfig::ProductPaths &d) {
        std::vector<Info> out;
        out.reserve(sizeof(kTable) / sizeof(kTable[0]));
        for (const auto &e: kTable) out.push_back({e.type, &(d.*(e.path))});
        return out;
    }

    Loaded load(const AppConfig::ProductPaths &d, const std::string &tag,
                const std::set<SatSystem> &systems, const std::vector<ProductType> &wanted) {
        Loaded out;
        Products &p = *out.prod;
        Jobs jobs;
        const auto joinAll = [&jobs, &out] {
            for (auto &j: jobs)
                if (!j.get()) out.readOk = false;
            jobs.clear();
        };

        for (const ProductType type: wanted) {
            const auto row = std::find_if(std::begin(kTable), std::end(kTable),
                                          [type](const Entry &e) { return e.type == type; });
            if (row == std::end(kTable)) continue;
            const Entry &e = *row;
            const std::string &raw = d.*(e.path);
            if (raw.empty()) continue;
            if (e.serial) joinAll();
            const auto paths = e.multiFile ? splitPaths(raw) : std::vector<std::string>{raw};
            for (const auto &path: paths) {
                if (e.serial) {
                    if (!readOne(e, p, tag, path, systems)) out.readOk = false;
                } else {
                    jobs.emplace_back(std::async(std::launch::async, [&p, &tag, &systems, e, path] {
                        return readOne(e, p, tag, path, systems);
                    }));
                }
            }
        }
        joinAll();
        return out;
    }
}
