#include "ObsSource.h"

#include "Exception.h"
#include "Log.h"

namespace ObsSource {
    Puller makePull(const bool rinex, std::unique_ptr<NovAtel::LogReader> &oem,
                    std::unique_ptr<RINEXObsReader> &rnx,
                    std::fstream &fs, const std::string &path, const char *who,
                    ObsData &cache, bool &cached,
                    std::shared_ptr<EphemerisTable> &cacheEph, std::string &err) {
        if (!rinex) {
            oem = Observer::makeNovAtelReader(path);
            if (!oem || !oem->open(path)) {
                err = std::string("无法打开 ") + who + " NovAtel 日志";
                return nullptr;
            }
            if (oem->getNextEpoch(cache)) {
                cached = true;
                cacheEph = oem->ephSnapshot();
            }
            return [&oem, &cache, &cached, &cacheEph](ObsData &out, std::shared_ptr<EphemerisTable> &eph) -> bool {
                if (cached) {
                    out = cache;
                    cached = false;
                    eph = cacheEph;
                    return true;
                }
                if (!oem->getNextEpoch(out)) return false;
                eph = oem->ephSnapshot();
                return true;
            };
        }
        fs.open(path, std::ios::in);
        if (!fs.is_open()) {
            err = std::string("无法打开 ") + who + " RINEX 观测文件";
            return nullptr;
        }
        rnx = Observer::makeRINEXReader(fs);
        try { rnx->parseRINEXObs(); }
        catch (...) {
            err = std::string(who) + " RINEX 解析失败";
            return nullptr;
        }
        return [&rnx](ObsData &out, std::shared_ptr<EphemerisTable> &eph) -> bool {
            try { out = rnx->parseRINEXObs(); }
            catch (const EndOfFile &) { return false; } catch (...) { return false; }
            eph = nullptr;
            return true;
        };
    }

    CommonTime probeRINEXFirstEpoch(const std::string &path, bool &ok) {
        ok = false;
        try {
            std::fstream pf(path, std::ios::in);
            if (!pf.is_open()) return CommonTime();
            const auto pr = Observer::makeRINEXReader(pf);
            const ObsData o = pr->parseRINEXObs();
            ok = true;
            return o.epoch;
        } catch (...) { return CommonTime(); }
    }

    bool openDual(const bool baseRinex, const bool roverRinex,
                  const std::string &basePath, const std::string &roverPath,
                  Handles &h,
                  Puller &nextBase, Puller &nextRover,
                  std::string &err) {
        nextBase = makePull(baseRinex, h.oemBase, h.rnxBase, h.fsBase, basePath, "基准站",
                            h.baseCache, h.baseCached, h.baseCacheEph, err);
        if (!nextBase) return false;
        nextRover = makePull(roverRinex, h.oemRover, h.rnxRover, h.fsRover, roverPath, "流动站",
                             h.roverCache, h.roverCached, h.roverCacheEph, err);
        if (!nextRover) return false;
        if (baseRinex) {
            CommonTime t;
            bool ok = false;
            if (h.roverCached) { t = h.roverCache.epoch; ok = true; }
            else if (roverRinex) t = probeRINEXFirstEpoch(roverPath, ok);
            if (ok && h.rnxBase) h.rnxBase->fastForwardToEpoch(t);
        }
        if (roverRinex) {
            CommonTime t;
            bool ok = false;
            if (h.baseCached) { t = h.baseCache.epoch; ok = true; }
            else if (baseRinex) t = probeRINEXFirstEpoch(basePath, ok);
            if (ok && h.rnxRover) h.rnxRover->fastForwardToEpoch(t);
        }
        return true;
    }

    std::string rtkInputError(const std::string &basePath, const std::string &roverPath,
                              const std::string &navPath) {
        if (basePath.empty() || roverPath.empty()) return "请先选择基准站与流动站观测文件";
        const auto fb = Observer::classifyObsPath(basePath);
        const auto fr = Observer::classifyObsPath(roverPath);
        if (fb == Observer::Format::Unknown || fr == Observer::Format::Unknown)
            return "无法识别观测文件格式";
        if ((Observer::isRINEX(fb) || Observer::isRINEX(fr)) && navPath.empty())
            return "RINEX 模式需要广播星历文件（任一观测文件为 RINEX 即需要）";
        return {};
    }

    bool loadNav(const std::string &navPath, EphemerisTable &navTable, std::string &err) {
        RINEXNavStore nav;
        nav.loadFile(navPath, navTable);
        if (navTable.gps.empty() && navTable.bds.empty()) {
            err = "广播星历为空";
            return false;
        }
        return true;
    }

}
