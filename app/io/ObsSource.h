#pragma once

#include <cctype>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rinex/RINEXObsReader.h"
#include "rinex/RINEXNavStore.h"
#include "novatel/LogReader.h"
#include "Observer.h"
#include "DualPair.h"

namespace ObsSource {
    using Puller = Observer::DualPair::Puller;

    Puller makePull(bool rinex, std::unique_ptr<NovAtel::LogReader> &oem,
                    std::unique_ptr<RINEXObsReader> &rnx,
                    std::fstream &fs, const std::string &path, const char *who,
                    ObsData &cache, bool &cached,
                    std::shared_ptr<EphemerisTable> &cacheEph, std::string &err);

    CommonTime probeRINEXFirstEpoch(const std::string &path, bool &ok);

    struct Handles {
        std::unique_ptr<NovAtel::LogReader> oemBase, oemRover;
        std::unique_ptr<RINEXObsReader> rnxBase, rnxRover;
        std::fstream fsBase, fsRover;
        ObsData baseCache, roverCache;
        bool baseCached = false, roverCached = false;
        std::shared_ptr<EphemerisTable> baseCacheEph, roverCacheEph;
    };

    bool openDual(bool baseRinex, bool roverRinex,
                  const std::string &basePath, const std::string &roverPath,
                  Handles &h,
                  Puller &nextBase, Puller &nextRover,
                  std::string &err);

    std::string rtkInputError(const std::string &basePath, const std::string &roverPath,
                              const std::string &navPath);

    bool loadNav(const std::string &navPath, EphemerisTable &navTable, std::string &err);

    inline std::vector<std::string> scanNavFiles(const std::string &obsPath) {
        std::vector<std::string> result;
        if (obsPath.size() < 4) return result;

        auto sameNoCase = [](const std::string &a, const std::string &b) {
            if (a.size() != b.size()) return false;
            for (std::size_t i = 0; i < a.size(); ++i) {
                if (std::tolower(static_cast<unsigned char>(a[i])) !=
                    std::tolower(static_cast<unsigned char>(b[i]))) return false;
            }
            return true;
        };

        std::vector<std::string> cands;
        if (const char tail = obsPath.back(); tail == 'O' || tail == 'o') {
            const std::string base = obsPath.substr(0, obsPath.size() - 1);
            for (auto &ext: {"N", "G", "C", "F", "E", "J", "I", "L"}) cands.push_back(base + ext);
        } else {
            const auto slash = obsPath.find_last_of("/\\");
            const auto dot = obsPath.find_last_of('.');
            if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) return result;
            const std::string stem = obsPath.substr(0, dot);
            for (auto &ext: {".nav", ".21N", ".21C"}) cands.push_back(stem + ext);
        }

        for (const auto &np: cands) {
            if (sameNoCase(np, obsPath)) continue;
            if (std::ifstream t(np); t.good()) {
                t.close();
                result.push_back(np);
            }
        }
        return result;
    }

    inline std::string autoDetectNav(const std::string &obsPath) {
        const auto &cands = scanNavFiles(obsPath);
        return cands.empty() ? std::string() : cands.front();
    }
}
