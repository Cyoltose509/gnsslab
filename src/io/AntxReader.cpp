#include "AntxReader.h"
#include <fstream>
#include <sstream>
#include <cctype>

namespace {
    // 卫星号：系统字符(G/C/R/E/J/S/I) + 1..3 位数字
    bool isSatId(const std::string &s) {
        if (s.empty() || s.size() > 4 || !std::isalpha(static_cast<unsigned char>(s[0])))
            return false;
        for (size_t i = 1; i < s.size(); ++i)
            if (!std::isdigit(static_cast<unsigned char>(s[i]))) return false;
        return s.size() >= 2;
    }

    // 系统 -> ATX 中该系统的 IF 双频点码（无频率信息时的兜底）
    std::pair<std::string, std::string> ifBandCodes(const char sys) {
        switch (sys) {
            case 'G': return {"G01", "G02"}; // L1, L2
            case 'C': return {"C01", "C02"}; // B1I, B2I
            case 'R': return {"R01", "R02"}; // G1, G2
            case 'E': return {"E01", "E07"}; // E1, E5b
            case 'J': return {"J01", "J02"}; // L1, L2
            default: return {"", ""};
        }
    }
}

bool AntxReader::read(const std::string &path) {
    std::ifstream in(path);
    if (!in) return false;

    std::string line;
    SatID curSat;
    std::string curRcv;
    std::string curBand;
    bool curIsSat = false;
    Vector3d curPCO(0, 0, 0);
    bool gotPCO = false;
    // PCV 网格参数：天顶距/方位角网格定义按天线给出，网格内容按频点给出
    double curZen1 = 0.0, curZen2 = 90.0, curDzen = 5.0, curDazi = 0.0;
    PcvGrid curGrid;

    // 行尾 61-80 列的标签；PCV 网格行标签为空
    auto trailingLabel = [](const std::string &s) -> std::string {
        // PCV 网格行有 19 个数值、行长可达 ~140 字符，远超 80 列，直接视为无标签数据行
        if (s.size() <= 60 || s.size() > 80) return {};
        const size_t a = s.find_first_not_of(" \t", 60);
        if (a == std::string::npos) return {};
        const size_t b = s.find_last_not_of(" \t\r\n");
        return s.substr(a, b - a + 1);
    };

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

    while (std::getline(in, line)) {
        if (size_t p = line.find_first_not_of(" \t"); p != std::string::npos) line = line.substr(p);

        if (line.find("TYPE / SERIAL NO") != std::string::npos ||
            line.find("TYPE/SERIAL") != std::string::npos) {
            consumePCO();
            curZen1 = 0.0;
            curZen2 = 90.0;
            curDzen = 5.0;
            curDazi = 0.0;
            curGrid = PcvGrid{};
            curSat = SatID();
            curRcv.clear();
            curBand.clear();
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
        } else if (line.find("START OF FREQUENCY") != std::string::npos) {
            consumePCO();
            if (std::istringstream ss(line); ss >> curBand) {
                /* e.g. "C01" */
            } else curBand.clear();
            curGrid = PcvGrid{};
            curGrid.zen1 = curZen1;
            curGrid.zen2 = curZen2;
            curGrid.dzen = curDzen;
            curGrid.dazi = curDazi;
        } else if (line.find("ZEN1 / ZEN2 / DZEN") != std::string::npos) {
            std::istringstream ss(line);

            if (    double a, b, c;ss >> a >> b >> c) {
                curZen1 = a;
                curZen2 = b;
                curDzen = c;
            }
        } else if (line.find("DAZI") != std::string::npos) {
            std::istringstream ss(line);
            if (double d; ss >> d) curDazi = d;
        } else if (line.find("NORTH / EAST / UP") != std::string::npos) {
            std::istringstream ss(line);

            if (double a, b, c;ss >> a >> b >> c) {
                curPCO = Vector3d(a, b, c) * 1e-3;
                gotPCO = true;
            }
        } else if (line.find("END OF FREQUENCY") != std::string::npos) {
            // 频点结束：先把当前频点 PCO / PCV 落库，再清空频点（避免最后一个频点丢失）
            consumePCO();
            if (curGrid.valid() && !curBand.empty()) {
                if (curIsSat) satPCV[curSat][curBand] = std::move(curGrid);
                else if (!curRcv.empty()) rcvPCV[curRcv][curBand] = std::move(curGrid);
            }
            curGrid = PcvGrid{};
            curBand.clear();
        } else if (line.find("END OF ANTENNA") != std::string::npos) {
            consumePCO();
            curSat = SatID();
            curRcv.clear();
            curBand.clear();
            curGrid = PcvGrid{};
        } else if (!curBand.empty() && trailingLabel(line).empty()) {
            // 频点内、行尾无标签 => PCV 网格行："NOAZI" 开头或与方位角值开头
            std::istringstream ss(line);
            if (std::string first; ss >> first) {
                if (first == "NOAZI") {
                    std::vector<double> v;
                    double x;
                    while (ss >> x) v.push_back(x * 1e-3);
                    curGrid.noazi = std::move(v);
                } else {
                    size_t pos = 0;
                    bool numeric = true;
                    try { (void) std::stod(first, &pos); } catch (...) { numeric = false; }
                    if (numeric && pos == first.size()) {
                        std::vector<double> v;
                        double x;
                        while (ss >> x) v.push_back(x * 1e-3);
                        if (!v.empty()) curGrid.azi.push_back(std::move(v));
                    }
                }
            }
        }
    }
    consumePCO();
    return !satPCO.empty() || !rcvPCO.empty();
}

Vector3d AntxReader::getSatPCO(const SatID &sat, const std::string &band) const {
    const auto it = satPCO.find(sat);
    if (it == satPCO.end()) return {0, 0, 0};
    const auto jt = it->second.find(band);
    if (jt == it->second.end()) return {0, 0, 0};
    return jt->second;
}


Vector3d AntxReader::getRcvPCOENU(const std::string &rcvType) const {
    std::istringstream ss(rcvType);
    std::string a, b;
    ss >> a >> b;
    if (a.empty()) return {0, 0, 0};
    const std::string key = a + (b.empty() ? "" : " " + b);
    if (const auto it = rcvPCO.find(key); it != rcvPCO.end()) {
        const Vector3d neu = it->second;
        return {neu.y(), neu.x(), neu.z()};
    }
    for (const auto &[k, v]: rcvPCO) {
        if (k.size() > a.size() && k.compare(0, a.size(), a) == 0 && k[a.size()] == ' ') {
            return {v.y(), v.x(), v.z()};
        }
    }
    return {0, 0, 0};
}

void AntxReader::applySatPCO(const SatID &sat, Vector3d &pTx, const CommonTime &epoch,
                             const FreqCombo &def) const {
    auto [b1, b2] = ifBandCodes(sat.system);
    const Vector3d pco1 = getSatPCO(sat, b1);
    const Vector3d pco2 = getSatPCO(sat, b2);
    if (pco1.squaredNorm() < 1e-12 && pco2.squaredNorm() < 1e-12) return;

    const Vector3d pco = def.c1 * pco1 + def.c2 * pco2; // 机体 (X,Y,Z)，单位 m
    if (pco.squaredNorm() < 1e-12) return;

    double gmst;
    Vector3d rsun, rmoon;
    Geodesy::sunMoonECEF(epoch, rsun, rmoon, gmst);
    Vector3d ex, ey, ez;
    Geodesy::satelliteBodyFrame(pTx, rsun, ex, ey, ez);

    const Vector3d pcoEcef = ex * pco.x() + ey * pco.y() + ez * pco.z();
    pTx += pcoEcef;
}

namespace {
    std::string rcvKey(const std::string &rcvType) {
        std::istringstream ss(rcvType);
        std::string a, b;
        ss >> a >> b;
        return a + (b.empty() ? "" : " " + b);
    }
}

double AntxReader::rcvPcvIF(const std::string &rcvType, const char sys, const FreqCombo &def,
                            const double zenDeg, const double azDeg) const {
    auto [b1, b2] = ifBandCodes(sys);
    if (b1.empty() || b2.empty()) return 0.0;
    const std::string key = rcvKey(rcvType);
    if (key.empty()) return 0.0;
    auto it = rcvPCV.find(key);
    if (it == rcvPCV.end()) {
        // 与 PCO 一致：型号相同、罩不同时退回同型号的第一条标定
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
