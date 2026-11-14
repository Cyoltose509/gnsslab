#include "ProductPicker.h"
#include "FileUtils.h"

#include "StringUtils.h"
#include "TimeStruct.h"
#include "download/local/Store.h"

#include <algorithm>
#include <filesystem>

namespace fs = std::filesystem;

namespace ProductPicker {
    namespace {
        std::string centerOf(const std::string &name) {
            std::string c = Str::toUpper(Str::leadingAlpha(name));
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

        void fill(Candidate &c, const std::string &name, const std::string &path,
                  const ProductMeta &meta) {
            c.path = path;
            c.name = name;
            c.center = centerOf(name);
            c.latency = latencyOf(name);
            c.slot = meta.slot;

            c.groupKey = c.center + "|" + c.latency + "|" + meta.interval + "|" +
                         (c.slot.empty() ? meta.ext : std::string());
            c.score = latencyScore(c.latency) * 100 + centerScore(c.center) * 10 +
                      (meta.spanDays == 1 ? 1 : 0);
        }
    }

    std::vector<Candidate> rank(const std::vector<std::string> &paths, const ProductType t) {
        std::vector<Candidate> out;
        for (const auto &path: paths) {
            const auto meta = ProductMeta::parse(File::baseName(path));
            if (!meta || meta->type != t) continue;
            Candidate c;
            fill(c, File::baseName(path), path, *meta);
            out.push_back(std::move(c));
        }
        std::sort(out.begin(), out.end(), [](const Candidate &a, const Candidate &b) {
            if (a.score != b.score) return a.score > b.score;
            if (a.slot != b.slot) return a.slot < b.slot;
            return a.name < b.name;
        });
        return out;
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
