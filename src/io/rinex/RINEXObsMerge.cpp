#include "rinex/RINEXObsMerge.h"

#include "StringUtils.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace RINEX {
    namespace {
        struct Piece {
            bool valid = false;
            int nTypes = 0;
            int rowsPerSat = 0;
            std::string marker;
            std::string lastObs;
            std::vector<std::string> header;
        };

        std::string labelOf(const std::string &line) {
            return line.size() > 60 ? Str::strip(line.substr(60)) : std::string();
        }

        Piece readPiece(const std::string &path) {
            Piece p;
            std::ifstream in(path);
            if (!in) return p;
            std::string line;
            for (int i = 0; i < 300 && std::getline(in, line); ++i) {
                p.header.push_back(line);
                const std::string label = labelOf(line);
                if (label.rfind("END OF HEADER", 0) == 0) {
                    p.valid = true;
                    break;
                }
                if (label.rfind("MARKER NAME", 0) == 0)
                    p.marker = Str::strip(Str::sub(line, 0, 60));
                else if (label.rfind("# / TYPES OF OBSERV", 0) == 0)
                    p.nTypes = Str::toInt(Str::sub(line, 0, 6));
                else if (label.rfind("TIME OF LAST OBS", 0) == 0)
                    p.lastObs = line;
            }
            if (p.nTypes > 0) p.rowsPerSat = (p.nTypes + 4) / 5;
            return p;
        }

        bool isEpochLine(const std::string &line, int &flag, int &numSats) {
            if (line.size() < 32) return false;
            double t[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            if (Str::scanDoubles(line.substr(0, 26).c_str(), t, 6) != 6) return false;
            if (t[1] < 1.0 || t[1] > 12.0 || t[2] < 1.0 || t[2] > 31.0 || t[3] < 0.0 ||
                t[3] > 23.0 || t[4] < 0.0 || t[4] > 59.0 || t[5] < 0.0 || t[5] >= 60.0)
                return false;
            const char f = line[28];
            if (!((f >= '0' && f <= '6') || f == ' ')) return false;
            flag = f >= '0' ? f - '0' : 0;
            numSats = Str::toInt(Str::sub(line, 29, 3));
            return numSats > 0;
        }

        std::string timeKey(const std::string &line) {
            double t[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            if (Str::scanDoubles(line.substr(0, 26).c_str(), t, 6) != 6) return {};
            std::ostringstream os;
            os << std::fixed << std::setprecision(7);
            for (const double v: t) os << v << '|';
            return os.str();
        }

        std::string cleanKey(const std::string &raw) {
            std::string out;
            for (const char c: raw) {
                if (std::isalnum(static_cast<unsigned char>(c)) != 0)
                    out += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
                else if (!out.empty() && out.back() != '_')
                    out += '_';
            }
            while (!out.empty() && out.back() == '_') out.pop_back();
            return out;
        }
    }

    std::vector<ObsGroup> groupObsFiles(const std::vector<std::string> &files) {
        std::map<std::string, std::vector<std::string> > byKey;
        for (const auto &path: files) {
            const Piece p = readPiece(path);
            if (!p.valid || p.rowsPerSat <= 0) continue;
            const std::string key = cleanKey(p.marker.empty()
                                                 ? std::filesystem::path(path).stem().string()
                                                 : p.marker);
            if (key.empty()) continue;
            byKey[key].push_back(path);
        }
        std::vector<ObsGroup> out;
        out.reserve(byKey.size());
        for (auto &[key, paths]: byKey) {
            std::sort(paths.begin(), paths.end());
            out.push_back({key, std::move(paths)});
        }
        return out;
    }

    int mergeObsFiles(const std::vector<std::string> &files, const std::string &outPath,
                      std::string &err) {
        std::vector<std::string> sorted = files;
        std::sort(sorted.begin(), sorted.end());
        if (sorted.empty()) {
            err = "没有可合并的观测分片";
            return 0;
        }
        const Piece tmpl = readPiece(sorted.front());
        if (!tmpl.valid || tmpl.rowsPerSat <= 0) {
            err = "首片 RINEX 头不可读";
            return 0;
        }
        const Piece tail = readPiece(sorted.back());
        const std::string lastObs = tail.valid && !tail.lastObs.empty() ? tail.lastObs : tmpl.lastObs;

        std::ofstream out(outPath, std::ios::binary | std::ios::trunc);
        if (!out) {
            err = "无法创建合并文件 " + outPath;
            return 0;
        }
        for (const std::string &h: tmpl.header) {
            if (labelOf(h).rfind("TIME OF LAST OBS", 0) == 0 && !lastObs.empty())
                out << lastObs << '\n';
            else
                out << h << '\n';
        }

        std::set<std::string> seen;
        int epochs = 0;
        std::vector<std::string> skipped;
        for (const auto &path: sorted) {
            const Piece p = readPiece(path);
            if (!p.valid || p.rowsPerSat != tmpl.rowsPerSat) {
                skipped.push_back(path);
                continue;
            }
            std::ifstream in(path);
            if (!in) {
                skipped.push_back(path);
                continue;
            }
            std::string line;
            for (std::size_t i = 0; i < p.header.size() && std::getline(in, line); ++i) {
            }
            while (std::getline(in, line)) {
                int flag = 0;
                int numSats = 0;
                if (!isEpochLine(line, flag, numSats)) continue;
                const int listRows = (numSats + 11) / 12;
                std::vector block{line};
                bool broken = false;
                for (int i = 1; i < listRows && !broken; ++i) {
                    if (!std::getline(in, line)) broken = true;
                    else block.push_back(line);
                }
                for (int i = 0; i < numSats * p.rowsPerSat && !broken; ++i) {
                    if (!std::getline(in, line)) broken = true;
                    else block.push_back(line);
                }
                if (broken) break;
                if (flag >= 2) continue;
                if (const std::string key = timeKey(block.front());
                    key.empty() || !seen.insert(key).second) continue;
                for (const auto &b: block) out << b << '\n';
                ++epochs;
            }
        }
        out.flush();
        if (!out) {
            err = "写合并文件失败";
            return 0;
        }
        if (epochs == 0) {
            err = "没有写出任何历元";
            return 0;
        }
        if (!skipped.empty()) err = "跳过 " + std::to_string(skipped.size()) + " 个观测类型不一致的分片";
        return epochs;
    }
}
