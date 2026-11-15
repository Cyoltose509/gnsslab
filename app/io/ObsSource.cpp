#include "ObsSource.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <filesystem>

#include "download/local/FileMeta.h"
#include "io/ObsDate.h"
#include "io/rinex/RINEXNavStore.h"
#include "i18n/I18n.h"

namespace ObsSource {
    bool ObsPair::open(const std::string &basePath, const std::string &roverPath,
                       const std::string &navPath) {
        // 观测头的 GLONASS 槽位可能只列首历元那几颗，缺的按广播星历频率号补。
        std::map<std::string, int> gloSlots;
        if (!navPath.empty()) RINEXNavStore::readGlonassSlots(navPath, gloSlots);
        if (!base.open(basePath, &err, &gloSlots)) {
            err = I18n::tr("io.err.base_prefix", err);
            return false;
        }
        if (!rover.open(roverPath, &err, &gloSlots)) {
            err = I18n::tr("io.err.rover_prefix", err);
            return false;
        }
        return alignStart();
    }

    bool ObsPair::alignStart() {
        try {
            Observer::Item b, r;
            if (!base.peek(b) || !rover.peek(r)) {
                err = I18n::tr("io.err.no_epoch");
                return false;
            }
            base.fastForward(r.obs.epoch);
            rover.fastForward(b.obs.epoch);
            return true;
        } catch (const std::exception &e) {
            err = I18n::tr("io.err.first_epoch", e.what());
            return false;
        }
    }

    Puller nextOf(Observer::ObsStream &s) {
        return [&s](ObsData &out, DualPair::EphPtr &eph) {
            Observer::Item it;
            if (!s.next(it)) return false;
            out = std::move(it.obs);
            eph = std::move(it.eph);
            return true;
        };
    }

    std::string RTKInputError(const std::string &basePath, const std::string &roverPath,
                              const std::string &navPath) {
        if (basePath.empty() || roverPath.empty()) return I18n::tr("io.err.need_base_rover");
        const auto fb = Observer::classifyObsPath(basePath);
        const auto fr = Observer::classifyObsPath(roverPath);
        if (fb == Observer::Format::Unknown || fr == Observer::Format::Unknown)
            return I18n::tr("io.err.bad_format");
        if (!Observer::carriesEph(fb) || !Observer::carriesEph(fr)) {
            if (navPath.empty()) return I18n::tr("rtk.err.need_nav");
        }
        return {};
    }

    bool loadNav(const std::string &navPath, EphemerisTable &navTable, std::string &err) {
        RINEXNavStore nav;
        nav.loadFile(navPath, navTable);
        if (navTable.empty()) {
            err = I18n::tr("io.err.empty_nav");
            return false;
        }
        return true;
    }

    std::vector<std::string> scanNavFiles(const std::string &obsPath) {
        std::vector<std::string> result;
        if (obsPath.size() < 4) return result;

        const auto sameNoCase = [](const std::string &a, const std::string &b) {
            if (a.size() != b.size()) return false;
            for (std::size_t i = 0; i < a.size(); ++i)
                if (std::tolower(static_cast<unsigned char>(a[i])) !=
                    std::tolower(static_cast<unsigned char>(b[i])))
                    return false;
            return true;
        };
        const auto exists = [&](const std::string &p) {
            return !sameNoCase(p, obsPath) && std::filesystem::is_regular_file(std::filesystem::path(p));
        };

        if (const char tail = obsPath.back(); tail == 'O' || tail == 'o') {
            const std::string base = obsPath.substr(0, obsPath.size() - 1);
            for (const char *ext: {"N", "G", "C", "F", "E", "J", "I", "L"})
                if (exists(base + ext)) result.push_back(base + ext);
            return result;
        }

        const auto od = parseObsDate(obsPath);
        const auto dir = std::filesystem::path(obsPath).parent_path();
        if (!od || dir.empty()) return result;
        char stamp[16];
        std::snprintf(stamp, sizeof stamp, "%04d%03d", od->year, od->doy);

        std::error_code ec;
        for (const auto &ent: std::filesystem::directory_iterator(dir, ec)) {
            if (!ent.is_regular_file(ec)) continue;
            const std::string name = ent.path().filename().string();
            if (name.find(stamp) == std::string::npos) continue;
            if (sameNoCase(ent.path().string(), obsPath)) continue;
            if (const auto m = FileMeta::parse(name);
                m && m->type == ProductType::NAV)
                result.push_back(ent.path().string());
        }
        std::sort(result.begin(), result.end());
        return result;
    }

    std::string autoDetectNav(const std::string &obsPath) {
        const auto cands = scanNavFiles(obsPath);
        return cands.empty() ? std::string() : cands.front();
    }
}
