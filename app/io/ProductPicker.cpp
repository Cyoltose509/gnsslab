#include "ProductPicker.h"

#include "StringUtils.h"
#include "TimeConvert.h"
#include "download/DownloadStore.h"

#include <algorithm>
#include <cctype>
#include <filesystem>

namespace fs = std::filesystem;

namespace ProductPicker {
    namespace {
        std::string leadingAlpha(const std::string &s) {
            std::string r;
            for (const char c: s) {
                if (!std::isalpha(static_cast<unsigned char>(c))) break;
                r.push_back(c);
            }
            return r;
        }


        std::string centerOf(const std::string &name) {
            std::string c = Str::toUpper(leadingAlpha(name));
            return c.size() > 3 ? c.substr(0, 3) : c;
        }


        std::string latencyOf(const std::string &name) {
            if (const auto us = name.find('_'); us != std::string::npos && us >= 3) {
                if (const std::string t = Str::toUpper(name.substr(us - 3, 3)); t == "FIN" || t == "RAP" || t == "ULT") return t;
            }
            const std::string p = Str::toLower(name.substr(0, 3));
            if (p == "igs") return "FIN";
            if (p == "igr") return "RAP";
            if (p == "igu") return "ULT";
            return {};
        }

        const char *latencyName(const std::string &lat) {
            if (lat == "FIN") return "最终";
            if (lat == "RAP") return "快速";
            if (lat == "ULT") return "超快";
            return nullptr;
        }

        int latencyScore(const std::string &lat) {
            if (lat == "FIN") return 3;
            if (lat == "RAP") return 2;
            if (lat == "ULT") return 1;
            return 2;
        }

        int centerScore(const std::string &c) {
            if (c == "COD") return 6;
            if (c == "IGS") return 5;
            if (c == "GFZ") return 4;
            if (c == "GRG" || c == "WUM") return 3;
            if (c == "ESA" || c == "JPL") return 2;
            return 1;
        }
    }

    std::string describe(const Candidate &c) {
        std::string s;
        if (!c.center.empty()) s += c.center;
        if (const char *ln = latencyName(c.latency)) {
            if (!s.empty()) s += " ";
            s += ln;
        }
        return s.empty() ? c.name : s;
    }

    namespace {
        void fill(Candidate &c, const std::string &name, const std::string &path,
                  const ProductMeta &meta, std::error_code &ec) {
            c.path = path;
            c.name = name;
            c.meta = meta;
            c.center = centerOf(name);
            c.latency = latencyOf(name);
            c.slot = meta.slot;
            c.size = static_cast<long long>(fs::file_size(path, ec));


            c.groupKey = c.center + "|" + c.latency + "|" + meta.interval + "|" +
                         (c.slot.empty() ? meta.ext : std::string());
            c.score = latencyScore(c.latency) * 100 + centerScore(c.center) * 10 +
                      (meta.spanDays == 1 ? 1 : 0);
        }
    }

    std::vector<Candidate> rank(const std::vector<std::string> &paths,
                                const ProductType t) {
        std::vector<Candidate> out;
        std::error_code ec;
        for (const auto &path: paths) {
            const auto meta = ProductMeta::parse(Str::baseName(path));
            if (!meta || meta->type != t) continue;
            Candidate c;
            fill(c, Str::baseName(path), path, *meta, ec);
            out.push_back(std::move(c));
        }
        std::sort(out.begin(), out.end(), [](const Candidate &a, const Candidate &b) {
            if (a.score != b.score) return a.score > b.score;
            if (a.slot != b.slot) return a.slot < b.slot;
            return a.name < b.name;
        });
        return out;
    }

    std::vector<Candidate> scan(const ProductType t, const int year,
                                const int doy, const int backDays) {
        std::vector<Candidate> out;
        if (year < 1980 || doy < 1) return out;

        std::error_code ec;
        for (int k = 0; k <= backDays; ++k) {
            const long ord = Time::civilToYMD({year, 1, 1}) + doy - 1 - k;
            const YearMonthDay ymd = Time::YMDToCivil(ord);
            const int dd = static_cast<int>(ord - Time::civilToYMD({ymd.year, 1, 1})) + 1;

            const std::string dir = DownloadStore::dirFor(t, ymd.year, dd);
            if (!fs::is_directory(dir, ec)) continue;

            for (const auto &ent: fs::directory_iterator(dir, ec)) {
                if (!ent.is_regular_file(ec)) continue;
                const std::string name = ent.path().filename().string();

                if (Str::endsWith(name, ".part") || name.find(".tmp_") != std::string::npos) continue;

                const auto meta = ProductMeta::parse(name);
                if (!meta || meta->type != t) continue;
                if (!meta->covers(year, doy)) continue;

                Candidate c;
                fill(c, name, ent.path().string(), *meta, ec);
                out.push_back(std::move(c));
            }
        }

        std::sort(out.begin(), out.end(), [](const Candidate &a, const Candidate &b) {
            if (a.score != b.score) return a.score > b.score;
            if (a.slot != b.slot) return a.slot < b.slot;
            return a.name < b.name;
        });
        return out;
    }

    std::vector<Candidate> pick(const ProductType t, const int year,
                                const int doy) {
        const std::vector<Candidate> all = scan(t, year, doy);
        if (all.empty()) return {};

        const Candidate &best = all.front();


        if (!best.slot.empty()) {
            std::vector<Candidate> group;
            for (const auto &c: all)
                if (c.groupKey == best.groupKey && !c.slot.empty()) group.push_back(c);
            if (group.size() > 1) {
                const std::string why = describe(best) + "，" + std::to_string(group.size()) +
                                        " 个时次（同一天的多个时次都要，不能只取一个）";
                for (auto &c: group) c.reason = why;
                return group;
            }
        }

        Candidate r = best;
        r.reason = describe(best);
        if (all.size() > 1)
            r.reason += "，" + std::to_string(all.size()) + " 份候选中质量最优";
        return {r};
    }

    std::string bestPath(const std::vector<std::string> &paths, const ProductType t) {
        const std::vector<Candidate> ranked = rank(paths, t);
        if (!ranked.empty()) return ranked.front().path;
        return paths.empty() ? std::string() : paths.front();
    }

    std::vector<std::string> bestPaths(const std::vector<std::string> &paths, const ProductType t) {
        const std::vector<Candidate> ranked = rank(paths, t);
        if (ranked.empty()) return paths;
        std::vector<std::string> out;
        out.reserve(ranked.size());
        for (const auto &c: ranked) out.push_back(c.path);
        return out;
    }
}
