#pragma once

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

// 观测源接入：按路径打开 RINEX / NovAtel 观测，统一成 Observer::DualPair::Puller，
// 并在双站 RINEX 间做起始历元对齐。仅做 IO，不涉及解算编排（逐历元引擎见 solvers/common/Driver.h）。
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

    /// 扫描观测路径的伴生广播星历（同主干换星历扩展名）。
    inline std::vector<std::string> scanNavFiles(const std::string &obsPath) {
        std::vector<std::string> result;
        if (obsPath.size() < 4) return result;
        const std::string base = obsPath.substr(0, obsPath.size() - 1);
        for (auto &ext: {"N", "G", "C", "F", "E", "J", "I", "L"}) {
            std::string np = base + ext;
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
