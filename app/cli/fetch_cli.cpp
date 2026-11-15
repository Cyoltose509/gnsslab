#include <algorithm>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <map>
#include <string>
#include <vector>

#include "CliCommon.h"
#include "i18n/I18n.h"
#include "Log.h"
#include "core/IniFile.h"
#include "download/local/Store.h"
#include "download/remote/Catalog.h"
#include "download/remote/Earthdata.h"
#include "download/remote/Fetcher.h"
#include "download/remote/SourceTable.h"
#include "download/remote/http/Core.h"
#include "core/PathMemory.h"
#include "util/Clock.h"

namespace {

    bool collectArgs(const int argc, char **argv, std::string &iniPath, bool &iniGiven,
                     std::map<std::string, std::string> &over) {
        for (int i = 1; i < argc; ++i) {
            const std::string a = argv[i];
            if (a.rfind("--", 0) != 0) {
                if (!iniGiven) {
                    iniPath = a;
                    iniGiven = true;
                } else if (iniPath != a) {
                    std::cerr << I18n::tr("cli.args.extra", a) << "\n";
                    return false;
                }
                continue;
            }
            std::string key = a.substr(2), val;
            bool hasVal = false;
            if (const auto eq = key.find('='); eq != std::string::npos) {
                val = key.substr(eq + 1);
                key.resize(eq);
                hasVal = true;
            } else if (i + 1 < argc && argv[i + 1][0] != '-') {
                val = argv[++i];
                hasVal = true;
            }
            over[key] = hasVal ? val : "1";
        }
        return true;
    }

    std::vector<std::string> splitList(const std::string &s) {
        std::vector<std::string> out;
        std::string cur;
        for (const char ch: s + ",") {
            if (ch == ',' || ch == ' ' || ch == '\t' || ch == kIniListSep) {
                if (!cur.empty()) out.push_back(cur);
                cur.clear();
            } else {
                cur.push_back(ch);
            }
        }
        return out;
    }

    /// 日期条目 → (year, doy)。"2026-215"；范围用 "2026-215..217"。
    bool parseDateOne(const std::string &t, std::vector<std::pair<int, int> > &out) {
        int year = 0, first = 0, last = 0;
        const char *hyphen = nullptr;
        if (const auto [p, ec] = std::from_chars(t.data(), t.data() + t.size(), year);
            ec == std::errc() && p != t.data() + t.size() && *p == '-')
            hyphen = p;
        if (!hyphen) return false;
        const std::string rest(hyphen + 1, t.data() + t.size());
        const auto dots = rest.find("..");
        const std::string a = dots == std::string::npos ? rest : rest.substr(0, dots);
        const std::string b = dots == std::string::npos ? rest : rest.substr(dots + 2);
        if (const auto [pa, e1] = std::from_chars(a.data(), a.data() + a.size(), first);
            e1 != std::errc() || pa != a.data() + a.size()) return false;
        if (dots == std::string::npos) {
            last = first;
        } else if (const auto [pb, e2] = std::from_chars(b.data(), b.data() + b.size(), last);
            e2 != std::errc() || pb != b.data() + b.size()) {
            return false;
        }
        if (year < 1980 || first < 1 || first > 366 || last < first || last > 366) return false;
        for (int d = first; d <= last; ++d) out.emplace_back(year, d);
        return true;
    }

    std::vector<std::pair<int, int> > parseDates(const std::string &s) {
        std::vector<std::pair<int, int> > out;
        if (const auto items = splitList(s); !items.empty()) {
            for (const auto &t: items)
                if (!parseDateOne(t, out)) std::cerr << I18n::tr("cli.fetch.bad_date", t) << "\n";
            return out;
        }
        // 空 = 今天
        const std::tm tm = Clock::localNow();
        const int year = tm.tm_year + 1900;
        const int doy = YearMonthDay{year, tm.tm_mon + 1, tm.tm_mday}.toDOY();
        if (doy > 0) out.emplace_back(year, doy);
        return out;
    }

    ProductType parseType(std::string s) {
        s = Str::toLower(s);
        if (s == "obs") s = "rinex";            // 下载目录名 rinex 的别名
        if (s == "snx") s = "ref_truth/snx";    // 同上
        return typeFromDir(s);
    }

    /// 低轨卫星的宽松解析：S6A / Sentinel-6A / Swarm C / COSMIC-2 / GRACE-FO …
    struct LeoSel {
        std::string mission;
        char sat = 0;
        [[nodiscard]] bool empty() const { return mission.empty(); }
    };

    LeoSel parseSat(const std::string& s) {
        std::string n;
        for (const char c: s)
            if (std::isalnum(static_cast<unsigned char>(c))) n.push_back(static_cast<char>(std::toupper(c)));
        if (n.size() < 3) return {};
        if (n.rfind("S6A", 0) == 0) return {"Sentinel-6", 'A'};
        if (n.rfind("S6B", 0) == 0) return {"Sentinel-6", 'B'};
        if (n.rfind("SWARM", 0) == 0 && n.back() >= 'A' && n.back() <= 'C') return {"Swarm", n.back()};
        if (n.rfind("COSMIC", 0) == 0) return {"COSMIC-2", 0};
        if (n.rfind("GRACE", 0) == 0) return {"GRACE-FO", 0};
        return {};
    }

    /// 落下来的文件是不是选的那颗星的（观测与轨道两套名字都认）。
    bool nameSel(const ProductType t, const LeoSel &sel, const std::string &name) {
        if (sel.empty()) return true;
        if (sel.mission == "Sentinel-6") return name.find(sel.sat == 'B' ? "S6B" : "S6A") != std::string::npos;
        if (sel.mission == "Swarm") {
            if (t == ProductType::LEO_SP3)
                return name.find(std::string("SP3") + sel.sat + "COM") != std::string::npos;
            return Download::SourceTable::SwarmSat(name) == sel.sat;
        }
        if (sel.mission == "COSMIC-2")
            return name.find(t == ProductType::LEO_SP3 ? "leoOrb" : "podCrx") != std::string::npos;
        if (sel.mission == "GRACE-FO") return name.find("GRACE") != std::string::npos;
        return true;
    }

    /// 该类型在源表里的目录来源。
    std::vector<Download::SourceTable::Dir> dirsFor(const ProductType t, const int year, const int doy) {
        if (t == ProductType::LEO_OBS) return Download::SourceTable::LEOObs(year, doy);
        if (t == ProductType::LEO_SP3 || t == ProductType::SNX) {
            std::vector<Download::SourceTable::Dir> out;
            for (auto &d: Download::SourceTable::refTruth(year, doy))
                if (d.type == t) out.push_back(std::move(d));
            return out;
        }
        Download::SourceTable::Query q;
        q.types = {t};
        q.year = year;
        q.doy = doy;
        return Download::SourceTable::daily(q);
    }

    std::string hostOf(const std::string &url) {
        const auto scheme = url.find("://");
        if (scheme == std::string::npos) return {};
        const std::size_t b = scheme + 3;
        const auto e = url.find('/', b);
        return Str::toLower(url.substr(b, e == std::string::npos ? std::string::npos : e - b));
    }

    /// 数据中心筛选：url 主机名含任一关键字才算。镜像源只保留命中的一条地址。
    void filterDc(std::vector<Download::SourceTable::Dir> &dirs, const std::vector<std::string> &dcs) {
        if (dcs.empty()) return;
        const auto hit = [&dcs](const std::string &url) {
            if (url.empty()) return false;
            const std::string host = hostOf(url);
            for (const auto &dc: dcs)
                if (host.find(Str::toLower(dc)) != std::string::npos) return true;
            return false;
        };
        std::vector<Download::SourceTable::Dir> out;
        for (auto &d: dirs) {
            const bool mainHit = hit(d.url);
            const bool backHit = hit(d.backupUrl);
            if (!mainHit && !backHit) continue;
            if (!mainHit) d.url = d.backupUrl;               // 主地址没命中，把命中的镜像提为主地址
            d.backupUrl = backHit && d.backupUrl != d.url ? d.backupUrl : std::string();
            out.push_back(std::move(d));
        }
        dirs = std::move(out);
    }

    /// 匹配字段 → 候选 URL。matches 按书写顺序即优先级：前一个字段命中的候选先收，
    /// count_limit 没填满再让下一个字段补。单字段时所有含它的文件都算候选，按名字排序。
    std::vector<std::string> matchPick(const std::vector<std::string> &urls,
                                       const std::vector<std::string> &matches, const int limit) {
        std::vector<std::string> out;
        const auto full = [&] {
            return limit < 0 || static_cast<int>(out.size()) < limit;
        };
        if (matches.empty()) {
            std::vector<std::string> sorted = urls;
            std::sort(sorted.begin(), sorted.end());
            if (limit >= 0 && static_cast<int>(sorted.size()) > limit)
                sorted.resize(static_cast<std::size_t>(limit));
            return sorted;
        }
        for (const auto &m: matches) {
            std::vector<std::string> cands;
            for (const auto &u: urls)
                if (u.find(m) != std::string::npos && std::find(out.begin(), out.end(), u) == out.end())
                    cands.push_back(u);
            std::sort(cands.begin(), cands.end());
            for (const auto &u: cands) {
                out.push_back(u);
                if (!full()) return out;
            }
        }
        return out;
    }

    /// 下载一条（落盘与 GUI 同库），打印进度与结果。
    bool fetchOne(const Download::Fetcher::Pending &p, const int year, const int doy) {
        if (!Download::Store::findLocal(p.type, year, doy, p.remoteName).empty()) {
            std::cout << "  " << I18n::tr("common.local_exist", p.remoteName) << "\n";
            return true;
        }
        const auto prog = [&p](const double total, const double now) {
            const float f = Download::Http::fracOf(total, now);
            if (f < 0.0f) return;
            std::cout << "\r  " << I18n::tr("cli.fetch.downloading", p.remoteName) << "  "
                    << static_cast<int>(f * 100.0f) << "%" << std::flush;
        };
        const auto r = Download::Fetcher::fetchNow(p, prog, nullptr);
        if (r.paused) {
            std::cout << "\r  " << I18n::tr("cli.fetch.paused", p.remoteName) << "        \n";
            return false;
        }
        if (!r.ok) {
            std::cout << "\r  " << I18n::tr("cli.fetch.failed", p.remoteName, r.err) << "        \n";
            return false;
        }
        std::cout << "\r  " << I18n::tr("ref.fetch.got", r.path.empty() ? p.remoteName : r.path)
                << "        \n";
        return true;
    }

    /// 一类产品 × 一天的完整流程：源表 → 数据中心筛选 → 列目录 → 匹配/限数 → 逐条落地。
    int fetchType(const ProductType t, const int year, const int doy, const LeoSel &sel,
                  const std::vector<std::string> &dcs, const std::vector<std::string> &matches,
                  const int limit) {
        auto dirs = dirsFor(t, year, doy);
        if (dirs.empty()) {
            std::cout << I18n::tr("cli.fetch.no_source", typeLabel(t), year, doy) << "\n";
            return 0;
        }
        if (!sel.empty()) {
            std::vector<Download::SourceTable::Dir> out;
            for (auto &d: dirs)
                if ((d.mission.empty() || d.mission == sel.mission) &&
                    (d.sat == 0 || sel.sat == 0 || d.sat == sel.sat))
                    out.push_back(std::move(d));
            dirs = std::move(out);
            if (dirs.empty()) {
                std::cout << I18n::tr(sel.sat ? "cli.fetch.no_mission_sat" : "cli.fetch.no_mission",
                                      typeLabel(t), year, doy, sel.mission) << "\n";
                return 0;
            }
        }
        filterDc(dirs, dcs);
        if (dirs.empty()) {
            std::cout << I18n::tr("cli.fetch.no_dc", typeLabel(t), year, doy) << "\n";
            return 0;
        }

        // 先扫本机库：已有且对得上的文件直接报出来，不再重复下载
        std::vector<std::string> localHave;
        for (const auto &lf: Download::Store::listFiles(t, year, doy))
            if (nameSel(t, sel, lf.name)) localHave.push_back(lf.name);

        std::string listErr;
        const auto urls = Download::Fetcher::resolve(dirs, year, doy, &listErr);
        std::vector<std::string> picked;
        for (const auto &u: urls) {
            const std::string name = Download::Http::nameOfUrl(u);
            // IGS 周目录是混合目录：长名自报的产品类型与要的不符就跳过（认不出类型的信任目录类型）
            if (const auto m = FileMeta::parse(name);
                m && m->type != ProductType::Unknown && m->type != t)
                continue;
            if (nameSel(t, sel, name)) picked.push_back(u);
        }
        picked = matchPick(picked, matches, limit);
        if (picked.empty()) {
            if (!localHave.empty()) {
                for (const auto &n: localHave)
                    std::cout << "  " << I18n::tr("common.local_exist", n) << "\n";
                return 0;
            }
            std::string tail;
            if (!matches.empty()) {
                std::string list;
                for (const auto &m: matches) list += " " + m;
                tail += I18n::tr("cli.fetch.match_suffix", list);
            }
            if (!listErr.empty()) tail += I18n::tr("cli.fetch.err_suffix", listErr);
            std::cout << I18n::tr("cli.fetch.no_match", typeLabel(t), year, doy) << tail << "\n";
            return 0;
        }

        int ok = 0;
        for (const auto &u: picked) {
            Download::Fetcher::Pending p;
            p.remoteName = Download::Http::nameOfUrl(u);
            p.url = u;
            p.type = t;
            p.year = year;
            p.doy = doy;
            ok += fetchOne(p, year, doy) ? 1 : 0;
        }
        return ok;
    }
}

int main(const int argc, char **argv) {
    std::string iniPath = "fetch.ini";
    std::map<std::string, std::string> over;
    if (bool iniGiven = false; !collectArgs(argc, argv, iniPath, iniGiven, over))
        return 2;

    IniFile f;
    if (!f.load(iniPath)) {
        std::cerr << I18n::tr("cli.ini.unreadable", iniPath) << "\n";
        return 1;
    }
    const CliSource src(f, std::move(over));
    Cli::setupLanguage(src);
    Log::init("fetch_cli.log");

    const std::string user = src.get("earthdata", "user");
    const std::string pass = src.get("earthdata", "pass");
    // ini/参数没填时回落 GUI 存的 PathMemory 账密（同一台机器填过就开箱即用）
    std::string authSrc = "fetch.ini";
    std::string effUser = user, effPass = pass;
    if (effUser.empty() || effPass.empty()) {
        const auto &pm = AppConfig::PathMemory::instance();
        if (effUser.empty()) effUser = pm.get(Download::Earthdata::kUserKey);
        if (effPass.empty()) effPass = pm.get(Download::Earthdata::kPassKey);
        if (!effUser.empty() && !effPass.empty()) authSrc = I18n::tr("cli.fetch.auth.gui");
    }
    Download::Http::setEarthdata(effUser, effPass);
    if ((effUser.empty() || effPass.empty()) &&
        (src.get("fetch", "type").find("leo") != std::string::npos ||
         src.get("fetch", "datacenter").find("cddis") != std::string::npos))
        std::cerr << I18n::tr("cli.fetch.auth_hint") << "\n";
    else
        std::cout << I18n::tr("cli.fetch.auth_from", authSrc,
                              effUser.empty() ? I18n::tr("cli.fetch.anonymous") : std::string()) << "\n";

    const auto dates = parseDates(src.get("fetch", "date"));
    if (dates.empty()) {
        std::cerr << I18n::tr("cli.fetch.no_date") << "\n";
        return 2;
    }

    int okTotal = 0;

    // 辅助产品同步更新：atx / gpt3，不走日期循环
    for (const auto &id: splitList(src.get("fetch", "update"))) {
        std::cout << I18n::tr("cli.fetch.updating", id) << "\n";
        if (std::string detail; Download::Catalog::updateProduct(id, detail)) {
            std::cout << "  " << detail << "\n";
            ++okTotal;
        } else {
            std::cerr << "  " << I18n::tr("cli.fetch.update_failed", detail) << "\n";
        }
    }

    const LeoSel sel = parseSat(src.get("fetch", "sat"));
    if (!src.get("fetch", "sat").empty() && sel.empty()) {
        std::cerr << I18n::tr("cli.fetch.bad_sat", src.get("fetch", "sat")) << "\n";
        return 2;
    }

    // 指定站名 → 自动搜索该站该日的 SINEX（本地累积缓存命中就不再联网）
    for (const auto &site: splitList(src.get("fetch", "site"))) {
        for (const auto &[year, doy]: dates) {
            double v[3]{0, 0, 0};
            bool cached = false;
            std::string detail;
            const auto prog = [&site](const double total, const double now) {
                const float fr = Download::Http::fracOf(total, now);
                if (fr >= 0.0f)
                    std::cout << "\r  " << I18n::tr("cli.fetch.fetching_sinex", site) << "  "
                            << static_cast<int>(fr * 100.0f) << "%" << std::flush;
            };
            const bool ok = Download::Catalog::ensureSiteTruth(site, year, doy, v, cached, detail, prog);
            std::cout << "\r";
            if (ok) {
                std::ostringstream xyz;
                xyz << v[0] << ", " << v[1] << ", " << v[2];
                std::cout << "  " << I18n::tr("cli.fetch.site_ok", site, year, doy, xyz.str(),
                                              cached ? I18n::tr("cli.fetch.cached") : std::string(),
                                              detail) << "\n";
                ++okTotal;
            } else {
                std::cout << "  " << I18n::tr("cli.fetch.site_failed", site, year, doy, detail) << "\n";
            }
        }
    }

    const auto dcs = splitList(src.get("fetch", "datacenter"));
    const auto matches = splitList(src.get("fetch", "match"));
    const int limit = src.getInt("fetch", "count_limit", 1);
    const auto types = splitList(src.get("fetch", "type"));
    if (types.empty() && src.get("fetch", "site").empty()) {
        std::cerr << I18n::tr("cli.fetch.no_type") << "\n";
        return 2;
    }
    for (const auto &t: types) {
        const ProductType pt = parseType(t);
        if (pt == ProductType::Unknown) {
            std::cerr << I18n::tr("cli.fetch.bad_type", t) << "\n";
            return 2;
        }
        for (const auto &[year, doy]: dates)
            okTotal += fetchType(pt, year, doy, sel, dcs, matches, limit);
    }

    for (const auto &k: src.unusedKeys())
        std::cerr << I18n::tr("cli.args.unused_fetch", k) << "\n";
    std::cout << I18n::tr("cli.fetch.summary", okTotal) << "\n";
    return okTotal > 0 ? 0 : 1;
}
