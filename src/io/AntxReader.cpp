#include "AntxReader.h"
#include "CoordConvert.h"
#include <sstream>
#include <charconv>
#include <string_view>
#include <StringUtils.h>

namespace {

    bool isSatId(const std::string &s) {
        if (s.empty() || s.size() > 4 || !std::isalpha(static_cast<unsigned char>(s[0])))
            return false;
        for (size_t i = 1; i < s.size(); ++i)
            if (!std::isdigit(static_cast<unsigned char>(s[i]))) return false;
        return s.size() >= 2;
    }


    std::pair<std::string, std::string> ifBandCodes(const char sys) {
        switch (sys) {
            case 'G': return {"G01", "G02"};
            case 'C': return {"C01", "C02"};
            case 'R': return {"R01", "R02"};
            case 'E': return {"E01", "E07"};
            case 'J': return {"J01", "J02"};
            default: return {"", ""};
        }
    }
    std::string rcvKey(const std::string &rcvType) {
        std::istringstream ss(rcvType);
        std::string a, b;
        ss >> a >> b;
        return a + (b.empty() ? "" : " " + b);
    }
}

double PcvGrid::value(const double zenDeg, const double azDeg) const {
    if (noazi.empty()) return 0.0;
    if (!std::isfinite(zenDeg) || !std::isfinite(azDeg)) return 0.0;
    const int n = static_cast<int>(noazi.size());
    const double dz = dzen > 1e-9 ? dzen : 1.0;
    double zf = (zenDeg - zen1) / dz;
    if (zf < 0.0) zf = 0.0;
    if (zf > n - 1) zf = static_cast<double>(n - 1);
    const int i0 = static_cast<int>(std::floor(zf));
    const int i1 = std::min(i0 + 1, n - 1);
    const double t = zf - i0;
    if (azi.empty() || dazi <= 1e-9) return noazi[i0] + t * (noazi[i1] - noazi[i0]);

    const int m = static_cast<int>(azi.size());
    double af = std::fmod(azDeg / dazi, static_cast<double>(m));
    if (af < 0.0) af += m;
    const int j0 = static_cast<int>(std::floor(af));
    const int j1 = (j0 + 1) % m;
    const double u = af - j0;
    const auto &r0 = azi[j0], &r1 = azi[j1];
    const double v0 = r0[i0] + t * (r0[i1] - r0[i0]);
    const double v1 = r1[i0] + t * (r1[i1] - r1[i0]);
    return v0 + u * (v1 - v0);
}

bool AntxReader::read(const std::string &path) {
    std::string line;
    SatID curSat;
    std::string curRcv;
    std::string curBand;
    bool curIsSat = false;
    Eigen::Vector3d curPCO(0, 0, 0);
    bool gotPCO = false;

    double curZen1 = 0.0, curZen2 = 90.0, curDzen = 5.0, curDazi = 0.0;
    PcvGrid curGrid;

    auto consumePCO = [&] {
        if (!gotPCO) return;
        if (curIsSat) {
            if (!curBand.empty())
                satPCO[curSat][curBand] = curPCO;
        } else if (!curRcv.empty()) {
            if (rcvPCO.find(curRcv) == rcvPCO.end())
                rcvPCO[curRcv] = curPCO;
        }
        gotPCO = false;
    };

    std::vector<double> vals;
    vals.reserve(64);
    const bool opened = forEachLine(path, [&](const std::string_view sv) {
        std::string_view lab;
        if (const size_t n = sv.size(); n > 60 && n <= 80) {
            if (const size_t a = sv.find_first_not_of(" \t", 60); a != std::string_view::npos) {
                const size_t b = sv.find_last_not_of(" \t");
                lab = sv.substr(a, b - a + 1);
            }
        }

        if (!lab.empty()) {

            if (lab.find("TYPE / SERIAL NO") != std::string_view::npos ||
                lab.find("TYPE/SERIAL") != std::string_view::npos) {
                consumePCO();
                curZen1 = 0.0;
                curZen2 = 90.0;
                curDzen = 5.0;
                curDazi = 0.0;
                curGrid = PcvGrid{};
                curSat = SatID();
                curRcv.clear();
                curBand.clear();
                line.assign(sv);
                std::istringstream ss(line);
                std::string tok;
                SatID firstSat;
                bool hasSat = false;
                std::vector<std::string> rcvParts;
                while (ss >> tok) {
                    if (isSatId(tok) && !hasSat) {
                        firstSat = SatID(tok[0], std::stoi(tok.substr(1)));
                        hasSat = true;
                    } else if (tok != "TYPE" && tok != "/" && tok != "SERIAL" && tok != "NO") {
                        rcvParts.push_back(tok);
                    }
                }
                if (hasSat) {
                    curSat = firstSat;
                    curIsSat = true;
                    curRcv.clear();
                } else {
                    std::string key;
                    for (size_t i = 0; i < rcvParts.size() && i < 2; ++i) {
                        if (i) key += ' ';
                        key += rcvParts[i];
                    }
                    curRcv = key;
                    curIsSat = false;
                    curSat = SatID();
                }
            } else if (lab.find("START OF FREQUENCY") != std::string_view::npos) {
                consumePCO();
                line.assign(sv);
                if (std::istringstream ss(line); ss >> curBand) {
                    /* e.g. "C01" */
                } else curBand.clear();
                curGrid = PcvGrid{};
                curGrid.zen1 = curZen1;
                curGrid.zen2 = curZen2;
                curGrid.dzen = curDzen;
                curGrid.dazi = curDazi;
            } else if (lab.find("ZEN1 / ZEN2 / DZEN") != std::string_view::npos) {
                line.assign(sv);
                std::istringstream ss(line);

                if (double a, b, c; ss >> a >> b >> c) {
                    curZen1 = a;
                    curZen2 = b;
                    curDzen = c;
                }
            } else if (lab.find("DAZI") != std::string_view::npos) {
                line.assign(sv);
                std::istringstream ss(line);
                if (double d; ss >> d) curDazi = d;
            } else if (lab.find("NORTH / EAST / UP") != std::string_view::npos) {
                line.assign(sv);
                std::istringstream ss(line);

                if (double a, b, c; ss >> a >> b >> c) {
                    curPCO = Eigen::Vector3d(a, b, c) * 1e-3;
                    gotPCO = true;
                }
            } else if (lab.find("END OF FREQUENCY") != std::string_view::npos) {

                consumePCO();
                if (curGrid.valid() && !curBand.empty()) {
                    if (curIsSat) satPCV[curSat][curBand] = std::move(curGrid);
                    else if (!curRcv.empty()) rcvPCV[curRcv][curBand] = std::move(curGrid);
                }
                curGrid = PcvGrid{};
                curBand.clear();
            } else if (lab.find("END OF ANTENNA") != std::string_view::npos) {
                consumePCO();
                curSat = SatID();
                curRcv.clear();
                curBand.clear();
                curGrid = PcvGrid{};
            }
            return;
        }

        if (curBand.empty()) return;
        {
            const char *q = sv.data();
            const char *qe = q + sv.size();
            while (q < qe && (*q == ' ' || *q == '\t')) ++q;
            const bool noazi = qe - q >= 5 && std::memcmp(q, "NOAZI", 5) == 0;
            if (noazi) {
                q += 5;
            } else {
                double az = 0.0;
                auto [ptr, ec] = std::from_chars(q, qe, az);
                if (ec != std::errc()) return;
                q = ptr;
                if (q < qe && *q != ' ' && *q != '	') return;
            }

            vals.clear();
            while (q < qe) {
                while (q < qe && (*q == ' ' || *q == '	')) ++q;
                if (q >= qe) break;
                double x = 0.0;
                auto [ptr, ec] = std::from_chars(q, qe, x);
                if (ec != std::errc()) break;
                vals.push_back(x * 1e-3);
                q = ptr;
            }
            if (noazi) curGrid.noazi = vals;
            else if (!vals.empty()) curGrid.azi.push_back(vals);
        }
    });
    if (!opened) return false;
    consumePCO();
    return !satPCO.empty() || !rcvPCO.empty();
}

Eigen::Vector3d AntxReader::getSatPCO(const SatID &sat, const std::string &band) const {
    const auto it = satPCO.find(sat);
    if (it == satPCO.end()) return {0, 0, 0};
    const auto jt = it->second.find(band);
    if (jt == it->second.end()) return {0, 0, 0};
    return jt->second;
}


Eigen::Vector3d AntxReader::getRcvPCOENU(const std::string &rcvType) const {
    std::istringstream ss(rcvType);
    std::string a, b;
    ss >> a >> b;
    if (a.empty()) return {0, 0, 0};
    const std::string key = a + (b.empty() ? "" : " " + b);
    if (const auto it = rcvPCO.find(key); it != rcvPCO.end()) {
        const Eigen::Vector3d neu = it->second;
        return {neu.y(), neu.x(), neu.z()};
    }
    for (const auto &[k, v]: rcvPCO) {
        if (k.size() > a.size() && k.compare(0, a.size(), a) == 0 && k[a.size()] == ' ') {
            return {v.y(), v.x(), v.z()};
        }
    }
    return {0, 0, 0};
}

void AntxReader::applySatPCO(const SatID &sat, Eigen::Vector3d &pTx, const CommonTime &epoch,
                             const FreqCombo &def) const {
    auto [b1, b2] = ifBandCodes(sat.system);
    const Eigen::Vector3d pco1 = getSatPCO(sat, b1);
    const Eigen::Vector3d pco2 = getSatPCO(sat, b2);
    if (pco1.squaredNorm() < 1e-12 && pco2.squaredNorm() < 1e-12) return;

    const Eigen::Vector3d pco = def.c1 * pco1 + def.c2 * pco2;
    if (pco.squaredNorm() < 1e-12) return;

    double gmst;
    Eigen::Vector3d rsun, rmoon;
    Geodesy::sunMoonECEF(epoch, rsun, rmoon, gmst);
    const Eigen::Matrix3d M = ECEFToSVBodyDCM(pTx, rsun);
    const Eigen::Vector3d pcoEcef = M.transpose() * pco;
    pTx += pcoEcef;
}


double AntxReader::rcvPcvIF(const std::string &rcvType, const char sys, const FreqCombo &def,
                            const double zenDeg, const double azDeg) const {
    auto [b1, b2] = ifBandCodes(sys);
    if (b1.empty() || b2.empty()) return 0.0;
    const std::string key = rcvKey(rcvType);
    if (key.empty()) return 0.0;
    auto it = rcvPCV.find(key);
    if (it == rcvPCV.end()) {
        const std::string type = key.substr(0, key.find(' '));
        for (const auto &[k, v]: rcvPCV) {
            if (k.size() > type.size() && k.compare(0, type.size(), type) == 0 && k[type.size()] == ' ') {
                it = rcvPCV.find(k);
                break;
            }
        }
    }
    if (it == rcvPCV.end()) return 0.0;
    const auto &tbl = it->second;
    const auto g1 = tbl.find(b1), g2 = tbl.find(b2);
    const double p1 = g1 == tbl.end() ? 0.0 : g1->second.value(zenDeg, azDeg);
    const double p2 = g2 == tbl.end() ? 0.0 : g2->second.value(zenDeg, azDeg);
    return def.c1 * p1 + def.c2 * p2;
}

double AntxReader::satPcvIF(const SatID &sat, const FreqCombo &def, const double nadirDeg) const {
    auto [b1, b2] = ifBandCodes(sat.system);
    if (b1.empty() || b2.empty()) return 0.0;
    const auto it = satPCV.find(sat);
    if (it == satPCV.end()) return 0.0;
    const auto &tbl = it->second;
    const auto g1 = tbl.find(b1), g2 = tbl.find(b2);
    const double p1 = g1 == tbl.end() ? 0.0 : g1->second.value(nadirDeg, 0.0);
    const double p2 = g2 == tbl.end() ? 0.0 : g2->second.value(nadirDeg, 0.0);
    return def.c1 * p1 + def.c2 * p2;
}
