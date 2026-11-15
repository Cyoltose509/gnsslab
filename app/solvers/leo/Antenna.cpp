#include "Antenna.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

#include "AntennaTable.h"
#include "StringUtils.h"
#include "i18n/I18n.h"

namespace Solvers::LEO::Antenna {
    namespace {
        /// `.HDR`（Earth Explorer 产品头）里解算只用得到 `r_CoG_ARP` 一行
        bool readLeverArm(const std::string &path, Eigen::Vector3d &out) {
            std::ifstream in(path);
            if (!in) return false;
            std::string line;
            while (std::getline(in, line)) {
                const std::size_t key = line.find("r_CoG_ARP");
                if (key == std::string::npos) continue;
                const std::size_t open = line.find('>', key);
                if (open == std::string::npos) continue;
                double v[3] = {0.0, 0.0, 0.0};
                if (Str::scanDoubles(line.c_str() + open + 1, v, 3) < 3) return false;
                out = Eigen::Vector3d(v[0], v[1], v[2]);
                return true;
            }
            return false;
        }

        /// 观测同目录、同主名的 `.HDR`
        std::string siblingHeader(const std::string &obsPath) {
            std::error_code ec;
            const std::filesystem::path obs(obsPath);
            const std::filesystem::path dir = obs.parent_path();
            if (dir.empty() || !std::filesystem::is_directory(dir, ec)) return {};

            std::string sameStem;
            std::vector<std::string> found;
            for (const auto &ent: std::filesystem::directory_iterator(dir, ec)) {
                std::error_code fec;
                if (!ent.is_regular_file(fec) || fec) continue;
                std::string ext = ent.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(),
                               [](const unsigned char c) { return static_cast<char>(std::tolower(c)); });
                if (ext != ".hdr") continue;
                found.push_back(ent.path().string());
                if (ent.path().stem() == obs.stem()) sameStem = ent.path().string();
            }
            if (!sameStem.empty()) return sameStem;
            return found.size() == 1 ? found.front() : std::string();
        }

        std::string markerName(const std::string &obsPath) {
            std::ifstream in(obsPath);
            if (!in) return {};
            std::string line;
            for (int i = 0; i < 200 && std::getline(in, line); ++i) {
                if (const std::size_t pos = line.find("MARKER NAME"); pos != std::string::npos) return line.substr(0, pos);
            }
            return {};
        }

        bool readBodyDelta(const std::string &obsPath, Eigen::Vector3d &out) {
            std::ifstream in(obsPath);
            if (!in) return false;
            bool spaceborne = false;
            bool have = false;
            Eigen::Vector3d v = Eigen::Vector3d::Zero();
            std::string line;
            for (int i = 0; i < 200 && std::getline(in, line); ++i) {
                const std::string label = line.size() > 60 ? Str::strip(line.substr(60)) : std::string();
                if (label.rfind("END OF HEADER", 0) == 0) break;
                if (label.rfind("MARKER TYPE", 0) == 0 &&
                    Str::toUpper(Str::strip(Str::sub(line, 0, 60))) == "SPACEBORNE")
                    spaceborne = true;
                if (label.rfind("ANTENNA: DELTA X/Y/Z", 0) == 0) {
                    double a[3] = {0.0, 0.0, 0.0};
                    if (Str::scanDoubles(Str::sub(line, 0, 42).c_str(), a, 3) != 3) continue;
                    v = Eigen::Vector3d(a[0], a[1], a[2]);
                    have = true;
                }
            }
            if (!spaceborne || !have || v.squaredNorm() < 1.0e-12) return false;
            out = v;
            return true;
        }

        std::string lookupKey(const std::string &obsPath) {
            if (obsPath.empty()) return {};
            return missionKey(obsPath) + " " +
                   Str::toUpper(std::filesystem::path(obsPath).filename().string());
        }
    }

    Correction fromHeaderFile(const std::string &path) {
        Correction fix;
        Eigen::Vector3d v;
        if (!readLeverArm(path, v)) return fix;
        fix.comToArp = {v, true, I18n::tr("leo.ant.from_file", std::filesystem::path(path).filename().string())};
        return fix;
    }

    Correction fromHeader(const std::string &obsPath) {
        const std::string hdr = siblingHeader(obsPath);
        return hdr.empty() ? Correction{} : fromHeaderFile(hdr);
    }

    std::string missionKey(const std::string &obsPath) {
        if (obsPath.empty()) return {};
        std::string key = markerName(obsPath);
        if (key.find_first_not_of(" \t") == std::string::npos)
            key = std::filesystem::path(obsPath).stem().string();
        return Str::toUpper(key);
    }

    Correction defaults(const std::string &obsPath) {
        Correction c;
        if (obsPath.empty()) return c;
        const std::string probe = lookupKey(obsPath);
        const auto row = AntennaTable::find(probe);

        if (const auto [comToArp, pco] = fromHeader(obsPath); comToArp.known)
            c.comToArp = comToArp;
        else if (row)
            c.comToArp = {row->comToArp, true, I18n::tr("leo.ant.from_mission", row->antenna)};
        else if (Eigen::Vector3d v; readBodyDelta(obsPath, v))
            c.comToArp = {v, true, I18n::tr("leo.ant.from_rinex")};

        if (row && row->hasPco) c.pco = {row->pco, true, I18n::tr("leo.ant.from_mission", row->antenna)};
        return c;
    }

    Correction merge(const Correction &user, const Correction &def) {
        Correction c;
        c.comToArp = user.comToArp.known ? user.comToArp : def.comToArp;
        c.pco = user.pco.known ? user.pco : def.pco;
        return c;
    }
}
