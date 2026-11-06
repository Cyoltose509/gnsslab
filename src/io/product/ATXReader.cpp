#include "product/ATXReader.h"
#include "CoordConvert.h"
#include "FileCache.h"
#include "MathUtils.h"
#include "TimeConvert.h"
#include <cstdio>
#include <filesystem>
#include <sstream>
#include <string_view>
#include <StringUtils.h>

namespace fs = std::filesystem;

namespace {
    // ANTEX 是定宽格式：前 60 列是数据，第 61 列起是右对齐的记录名。
    std::string recordLabel(const std::string &line) {
        return line.size() > 60 ? Str::strip(line.substr(60)) : std::string();
    }

    bool isSatId(const std::string &s) {
        if (s.empty() || s.size() > 4 || !std::isalpha(static_cast<unsigned char>(s[0])))
            return false;
        for (std::size_t i = 1; i < s.size(); ++i)
            if (!std::isdigit(static_cast<unsigned char>(s[i]))) return false;
        return s.size() >= 2;
    }

    std::pair<std::string, std::string> ifBandCodes(const char sys) {
        switch (sys) {
            case 'G': return {"G01", "G02"};
            case 'C': return {"C02", "C06"};
            case 'R': return {"R01", "R02"};
            case 'E': return {"E01", "E05"};
            case 'J': return {"J01", "J02"};
            default: return {"", ""};
        }
    }

    std::pair<std::string, std::string> atxBands(const char sys, const FreqCombo &def) {
        const auto band = [&](const std::string &code) -> std::string {
            if (code.size() >= 2 && std::isdigit(static_cast<unsigned char>(code[1])))
                return std::string(1, sys) + '0' + code[1];
            return {};
        };
        std::string b1 = band(def.code1), b2 = band(def.code2);
        if (b1.empty() || b2.empty() || b1 == b2) return ifBandCodes(sys);
        return {std::move(b1), std::move(b2)};
    }

    std::string rcvKey(const std::string &rcvType) {
        std::istringstream ss(rcvType);
        std::string a, b;
        ss >> a >> b;
        return a + (b.empty() ? "" : " " + b);
    }

    bool parseAntexMjd(const std::string_view sv, double &mjd) {
        const char *q = sv.data();
        const char *const qe = q + sv.size();
        int y = 0, mo = 0, d = 0, hh = 0, mi = 0;
        if (!Str::nextInt(q, qe, y) || !Str::nextInt(q, qe, mo) || !Str::nextInt(q, qe, d) ||
            !Str::nextInt(q, qe, hh) || !Str::nextInt(q, qe, mi))
            return false;
        double sec = 0.0;
        (void) Str::nextDouble(q, qe, sec); // 秒可缺，缺了按 0 处理
        if (mo < 1 || mo > 12 || d < 1 || d > 31) return false;
        mjd = static_cast<double>(Time::civilToYMD({y, mo, d})) +
              static_cast<double>(UNIX_MJD);
        mjd += (static_cast<double>(hh) * 3600.0 + static_cast<double>(mi) * 60.0 + sec) / SEC_PER_DAY;
        return true;
    }
}

double PCVGrid::value(const double zenDeg, const double azDeg) const {
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
    if (azi.empty() || dazi <= 1e-9) return Math::lerp(noazi[i0], noazi[i1], t);

    const int m = static_cast<int>(azi.size());
    double af = std::fmod(azDeg / dazi, static_cast<double>(m));
    if (af < 0.0) af += m;
    const int j0 = static_cast<int>(std::floor(af));
    const int j1 = (j0 + 1) % m;
    const double u = af - j0;
    const auto &r0 = azi[j0], &r1 = azi[j1];
    return Math::lerp(Math::lerp(r0[i0], r0[i1], t), Math::lerp(r1[i0], r1[i1], t), u);
}

bool ATXReader::read(const std::string &path) {
    tab.reset();
    const IoCache::Key key = IoCache::keyOf(path);
    if (key.size <= 0) return false;
    if (const auto hit = IoCache::find<Tables>(key)) {
        tab = hit;
        return true;
    }

    auto t = std::make_shared<Tables>();

    std::string line;
    SatID curSat;
    std::string curRcv;
    std::string curBand;
    bool curIsSat = false;
    Eigen::Vector3d curPCO(0, 0, 0);
    bool gotPCO = false;

    double curFromMjd = -1.0e9;
    double curUntilMjd = 1.0e9;

    double curZen1 = 0.0, curZen2 = 90.0, curDzen = 5.0, curDazi = 0.0;
    PCVGrid curGrid;

    const auto resetPeriod = [&] {
        curFromMjd = -1.0e9;
        curUntilMjd = 1.0e9;
    };

    const auto commit = [&](const bool withPco, const bool withGrid) {
        if (!curIsSat && curRcv.empty()) return;
        BandEntries *tbl;
        if (curIsSat) {
            if (curBand.empty()) return;
            tbl = &t->satEnt[curSat];
        } else {
            tbl = &t->rcvEnt[curRcv];
        }
        const auto touch = [&](const std::string &band) -> Entry & {
            std::vector<Entry> &vec = (*tbl)[band];
            for (auto &x: vec)
                if (std::fabs(x.fromMjd - curFromMjd) < 1e-6) return x;
            vec.push_back(Entry{});
            Entry &e = vec.back();
            e.fromMjd = curFromMjd;
            e.untilMjd = curUntilMjd;
            return e;
        };
        if (withPco) {
            touch(curIsSat ? curBand : std::string()).pco = curPCO;
            gotPCO = false;
        }
        if (withGrid && !curBand.empty()) touch(curBand).grid = curGrid;
    };

    std::vector<double> vals;
    vals.reserve(64);
    const bool opened = Str::forEachLine(path, [&](const std::string_view sv) {
        std::string_view lab;
        if (sv.size() > 60) {
            if (const std::size_t a = sv.find_first_not_of(" \t", 60);
                a != std::string_view::npos && std::isalpha(static_cast<unsigned char>(sv[a]))) {
                const std::size_t b = sv.find_last_not_of(" \t\r");
                lab = sv.substr(a, b - a + 1);
            }
        }

        if (!lab.empty()) {
            if (lab.find("TYPE / SERIAL NO") != std::string_view::npos ||
                lab.find("TYPE/SERIAL") != std::string_view::npos) {
                commit(gotPCO, false);
                resetPeriod();
                curZen1 = 0.0;
                curZen2 = 90.0;
                curDzen = 5.0;
                curDazi = 0.0;
                curGrid = PCVGrid{};
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
                    std::string ckey;
                    for (std::size_t i = 0; i < rcvParts.size() && i < 2; ++i) {
                        if (i) ckey += ' ';
                        ckey += rcvParts[i];
                    }
                    curRcv = ckey;
                    curIsSat = false;
                    curSat = SatID();
                }
            } else if (lab.find("VALID FROM") != std::string_view::npos) {
                if (double v; parseAntexMjd(sv, v)) curFromMjd = v;
            } else if (lab.find("VALID UNTIL") != std::string_view::npos) {
                if (double v; parseAntexMjd(sv, v)) curUntilMjd = v;
            } else if (lab.find("START OF FREQUENCY") != std::string_view::npos) {
                commit(gotPCO, false);
                line.assign(sv);
                if (std::istringstream ss(line); ss >> curBand) {
                    /* e.g. "C01" */
                } else curBand.clear();
                curGrid = PCVGrid{};
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
                commit(gotPCO, curGrid.valid() && !curBand.empty());
                curGrid = PCVGrid{};
                curBand.clear();
            } else if (lab.find("END OF ANTENNA") != std::string_view::npos) {
                commit(gotPCO, false);
                resetPeriod();
                curSat = SatID();
                curRcv.clear();
                curBand.clear();
                curGrid = PCVGrid{};
            }
            return;
        }

        if (curBand.empty()) return;
        {
            const char *q = sv.data();
            const char *const qe = q + sv.size();
            while (q < qe && (*q == ' ' || *q == '\t')) ++q;
            const bool noazi = qe - q >= 5 && std::memcmp(q, "NOAZI", 5) == 0;
            if (noazi) {
                q += 5;
            } else {
                if (double az = 0.0; !Str::nextDouble(q, qe, az)) return;
                if (q < qe && *q != ' ' && *q != '\t') return;
            }

            vals.clear();
            double x = 0.0;
            while (Str::nextDouble(q, qe, x)) vals.push_back(x * 1e-3);
            if (noazi) curGrid.noazi = vals;
            else if (!vals.empty()) curGrid.azi.push_back(vals);
        }
    });
    if (!opened) return false;
    commit(gotPCO, false);
    if (t->empty()) return false;
    tab = t;
    IoCache::keep<Tables>(key, std::move(t));
    return true;
}

// 按历元挑时段块：优先命中 [FROM, UNTIL] 且起点最晚的那块；
// 历元落在所有时段之外时退化为「不晚于历元的最晚一块」，再不行取第一块。
const ATXReader::Entry *ATXReader::pick(const std::vector<Entry> &v, const double mjd) {
    const Entry *hit = nullptr;
    const Entry *past = nullptr;
    for (const auto &e: v) {
        if (mjd >= e.fromMjd && mjd <= e.untilMjd) {
            if (!hit || e.fromMjd > hit->fromMjd) hit = &e;
        }
        if (e.fromMjd <= mjd && (!past || e.fromMjd > past->fromMjd)) past = &e;
    }
    if (hit) return hit;
    if (past) return past;
    return v.empty() ? nullptr : &v.front();
}

Eigen::Vector3d ATXReader::getSatPCO(const SatID &sat, const std::string &band, const CommonTime &t) const {
    if (!tab) return {0, 0, 0};
    const auto it = tab->satEnt.find(sat);
    if (it == tab->satEnt.end()) return {0, 0, 0};
    const auto jt = it->second.find(band);
    if (jt == it->second.end()) return {0, 0, 0};
    if (const Entry *e = pick(jt->second, static_cast<double>(Time::CommonToMJD(t).mjd)); e) return e->pco;
    return {0, 0, 0};
}


Eigen::Vector3d ATXReader::getRcvPCOENU(const std::string &rcvType, const CommonTime &t) const {
    if (!tab) return {0, 0, 0};
    const std::string key = rcvKey(rcvType);
    if (key.empty()) return {0, 0, 0};
    const double mjd = static_cast<double>(Time::CommonToMJD(t).mjd);
    const auto lookup = [&](const std::string &k) -> Eigen::Vector3d {
        const auto it = tab->rcvEnt.find(k);
        if (it == tab->rcvEnt.end()) return Eigen::Vector3d::Zero();
        const auto jt = it->second.find("");
        if (jt == it->second.end()) return Eigen::Vector3d::Zero();
        const Entry *e = pick(jt->second, mjd);
        return e ? e->pco : Eigen::Vector3d::Zero();
    };
    Eigen::Vector3d neu = lookup(key);
    if (neu.squaredNorm() < 1e-18) {
        // 型号全名对不上时退回「同型号（首段相同）」的第一条
        for (const auto &[k, v]: tab->rcvEnt) {
            if (k.size() > key.size() && k.compare(0, key.size(), key) == 0 && k[key.size()] == ' ') {
                neu = lookup(k);
                break;
            }
        }
    }
    if (neu.squaredNorm() < 1e-18) return {0, 0, 0};
    // ATX 存的是北东天 (N,E,U)，这里换回 E/N/U
    return {neu.y(), neu.x(), neu.z()};
}

void ATXReader::applySatPCO(const SatID &sat, Eigen::Vector3d &pTx, const CommonTime &epoch,
                            const FreqCombo &def, const Eigen::Matrix3d *satBody) const {
    auto [b1, b2] = atxBands(sat.system, def);
    const Eigen::Vector3d pco1 = getSatPCO(sat, b1, epoch);
    const Eigen::Vector3d pco2 = getSatPCO(sat, b2, epoch);
    if (pco1.squaredNorm() < 1e-12 && pco2.squaredNorm() < 1e-12) return;

    const Eigen::Vector3d pco = def.c1 * pco1 + def.c2 * pco2;
    if (pco.squaredNorm() < 1e-12) return;

    Eigen::Matrix3d M;
    if (satBody) {
        M = *satBody;
    } else {
        double gmst;
        Eigen::Vector3d rsun, rmoon;
        Geodesy::sunMoonECEF(epoch, rsun, rmoon, gmst);
        M = Coord::ECEFToSVBodyDCM(pTx, rsun);
    }
    const Eigen::Vector3d pcoEcef = M.transpose() * pco;
    pTx += pcoEcef;
}

// 某天线在某频点、某历元下的 PCV 值（无该频点标定则 0）。
double ATXReader::pcvAt(const BandEntries &tbl, const std::string &band,
                        const double zenDeg, const double azDeg, const double mjd) {
    const auto it = tbl.find(band);
    if (it == tbl.end()) return 0.0;
    const Entry *e = pick(it->second, mjd);
    return e ? e->grid.value(zenDeg, azDeg) : 0.0;
}

double ATXReader::rcvPCVIF(const std::string &rcvType, const char sys, const FreqCombo &def,
                           const double zenDeg, const double azDeg, const CommonTime &t) const {
    auto [b1, b2] = atxBands(sys, def);
    if (b1.empty() || b2.empty()) return 0.0;
    if (!tab) return 0.0;
    const std::string key = rcvKey(rcvType);
    if (key.empty()) return 0.0;
    const auto it = tab->rcvEnt.find(key);
    if (it == tab->rcvEnt.end()) return 0.0;
    const double mjd = static_cast<double>(Time::CommonToMJD(t).mjd);
    const double p1 = pcvAt(it->second, b1, zenDeg, azDeg, mjd);
    const double p2 = pcvAt(it->second, b2, zenDeg, azDeg, mjd);
    return def.c1 * p1 + def.c2 * p2;
}

double ATXReader::satPCVIF(const SatID &sat, const FreqCombo &def, const double nadirDeg,
                           const CommonTime &t) const {
    auto [b1, b2] = atxBands(sat.system, def);
    if (b1.empty() || b2.empty()) return 0.0;
    if (!tab) return 0.0;
    const auto it = tab->satEnt.find(sat);
    if (it == tab->satEnt.end()) return 0.0;
    const double mjd = static_cast<double>(Time::CommonToMJD(t).mjd);
    const double p1 = pcvAt(it->second, b1, nadirDeg, 0.0, mjd);
    const double p2 = pcvAt(it->second, b2, nadirDeg, 0.0, mjd);
    return def.c1 * p1 + def.c2 * p2;
}

bool ATXReader::validate(const std::string &path, std::string &detail) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        detail = "无法打开文件";
        return false;
    }
    std::error_code ec;
    if (const auto fileSize = fs::file_size(path, ec); ec || fileSize < 1024) {
        detail = "文件过小（<1 KB），不像是完整的 ANTEX 天线文件";
        return false;
    }

    constexpr char kTypeSerial[] = "TYPE / SERIAL NO";
    constexpr std::size_t kCarry = sizeof(kTypeSerial) - 2;

    std::string head, tail, carry;
    long long total = 0;
    int antennas = 0;
    std::vector<char> buf(1 << 20);
    while (true) {
        f.read(buf.data(), static_cast<std::streamsize>(buf.size()));
        const auto n = static_cast<std::size_t>(f.gcount());
        if (n == 0) break;
        const std::string chunk(buf.data(), n);

        if (head.size() < 64 * 1024) head += chunk.substr(0, 64 * 1024 - head.size());
        tail += chunk;
        if (tail.size() > 8192) tail.erase(0, tail.size() - 8192);

        // carry + chunk 里数，避免模式正好跨两块时漏计（也不会重复计）
        const std::string scan = carry + chunk;
        for (std::size_t p = scan.find(kTypeSerial); p != std::string::npos;
             p = scan.find(kTypeSerial, p + 1))
            ++antennas;
        carry = scan.size() > kCarry ? scan.substr(scan.size() - kCarry) : scan;
        total += static_cast<long long>(n);
        if (f.eof()) break;
    }

    std::string ver;
    int freqBlocks = 0;
    std::size_t pos = 0;
    while (pos < head.size()) {
        std::size_t nl = head.find('\n', pos);
        if (nl == std::string::npos) nl = head.size();
        std::string line = head.substr(pos, nl - pos);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        pos = nl + 1;

        const std::string label = recordLabel(line);
        if (label.empty()) continue;
        if (label.find("ANTEX VERSION") != std::string::npos)
            ver = Str::strip(line.substr(0, 8));
        else if (label.find("START OF FREQUENCY") != std::string::npos)
            ++freqBlocks;
    }

    if (ver.empty()) {
        detail = "不是 ANTEX 文件（缺少 ANTEX VERSION / SYST 头记录）";
        return false;
    }
    if (antennas == 0) {
        detail = "ANTEX 里没有任何天线块（TYPE / SERIAL NO）";
        return false;
    }
    if (freqBlocks == 0) {
        detail = "ANTEX 头部没有任何频点标定块（START OF FREQUENCY），文件可能被截断";
        return false;
    }

    std::string last;
    if (const std::size_t end = tail.find_last_not_of("\r\n"); end != std::string::npos) {
        const std::size_t nl = tail.rfind('\n', end);
        const std::size_t from = nl == std::string::npos ? 0 : nl + 1;
        last = tail.substr(from, end - from + 1);
    }
    if (const std::string lastLabel = recordLabel(last); lastLabel.find("END OF ANTENNA") == std::string::npos &&
                                                         lastLabel.find("END OF FILE") == std::string::npos) {
        detail = "文件末尾不是完整的收尾记录（缺 END OF ANTENNA / END OF FILE），"
                 "疑似下载或拷贝被截断";
        return false;
    }

    char mb[32];
    std::snprintf(mb, sizeof mb, "%.1f MB", static_cast<double>(total) / 1048576.0);
    detail = "ANTEX " + ver + "，" + std::to_string(antennas) + " 个天线块，" + mb;
    return true;
}
