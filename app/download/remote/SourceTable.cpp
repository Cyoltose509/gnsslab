#include "download/remote/SourceTable.h"
#include "FileUtils.h"

#include "StringUtils.h"
#include "time/TimeStruct.h"

#include <cstdio>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace Download::SourceTable {
    namespace {
        /// CMR virtual-directory 的集合号：Sentinel-6A L1B GNSS-POD 日文件。
        constexpr auto kCmrS6aDaily = "C1968980591-POCLOUD";
        constexpr auto kCmrVdirRoot = "https://cmr.earthdata.nasa.gov/virtual-directory/collections/";

        constexpr auto kIgsRoot = "ftp://igs.gnsswhu.cn/pub/gps";

        /// CDDIS 镜像根：cddisOf 拼 GNSS 各类产品，DORIS 镜像另给一份前缀。
        constexpr auto kCddisRoot = "https://cddis.nasa.gov/archive";
        constexpr auto kCddisDorisRoot = "https://cddis.nasa.gov/archive/doris/products/orbits/";

        constexpr auto kUcarNrtRoot = "https://data.cosmic.ucar.edu/gnss-ro/cosmic2/nrt/";
        constexpr auto kGfzOrbitRoot = "https://isdc-data.gfz.de/orbit/";
        constexpr auto kVmfRoot = "https://vmf.geo.tuwien.ac.at/trop_products/GRID/5x5/VMF3/VMF3_OP";

        /// Swarm L1B 的 GPS 无线电掩星产品
        constexpr auto kSwarmRoot =
                "https://swarm-diss.eo.esa.int/swarm/Level1b/Latest_baselines/GPSx_RO/Sat_";
        /// swarm-diss 文件管理器端点（?do=list 列目录 / ?do=download 取文件，均匿名可用）
        constexpr auto kSwarmDissRoot = "https://swarm-diss.eo.esa.int/";
        /// POD/RD 列目录查询串：翻页时 numAfter 改 pos，末尾接卫星号
        constexpr auto kSwarmPodQuery =
                "?do=list&maxfiles=20000&pos=0&file=swarm%2FLevel2daily%2F"
                "Latest_baselines%2FPOD%2FRD%2FSat_";
        constexpr char kSwarmSats[] = {'A', 'B', 'C'};
        constexpr const char *kSwarmVersions[] = {"0602", "0601"};

        /// GFZ ISDC 有 RSO（快速科学轨道）且近年仍在更新的级别
        constexpr const char *kGfzLevels[] = {"L06", "L07", "L09", "L10", "L13", "L20", "L64", "L65"};

        /// IDS 的 DORIS 轨道，IGN 有匿名 FTP 镜像
        constexpr auto kDorisRoot = "ftp://doris.ign.fr/pub/doris/products/orbits/";

        /// AIUB 新下载区的浏览接口
        constexpr auto kAIUBBrowse =
                "https://code.aiub.unibe.ch/s3_script/aiub_s3_bucket_listing.php?path=CODE/";

        constexpr char kCodePrefix[] = "COD0";

        std::string cddisOf(const std::string &igsUrl) {
            std::string rel = igsUrl.substr(std::string(kIgsRoot).size());
            const std::string mgex = "/products/mgex/";
            if (const std::size_t at = rel.find(mgex); at != std::string::npos)
                rel = rel.substr(0, at) + "/products/" + rel.substr(at + mgex.size());
            return std::string(kCddisRoot) + "/gnss" + rel;
        }

        std::string ucarLEO(const int year, const int doy, const char *level) {
            char buf[192];
            std::snprintf(buf, sizeof(buf), "%s%s/%04d/%03d/",
                          kUcarNrtRoot, level, year, doy);
            return buf;
        }

        std::string gzfRSO(const char *level, const int year, const int doy) {
            char buf[192];
            std::snprintf(buf, sizeof(buf), "%s%s/RSO/%04d/%03d/",
                          kGfzOrbitRoot, level, year, doy);
            return buf;
        }

        /// CMR 按日历日分 temporal 页
        std::string cmrTemporal(const YearMonthDay &ymd) {
            char buf[224];
            std::snprintf(buf, sizeof(buf), "%s%s/temporal/%04d/%02d/%02d/",
                          kCmrVdirRoot, kCmrS6aDaily, ymd.year, ymd.month, ymd.day);
            return buf;
        }

        std::string aiubCode(const int year) {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "%s%d", kAIUBBrowse, year);
            return buf;
        }

        std::string swarmName(const char sat, const int year, const int month, const int day,
                              const char *ver) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                          "SW_OPER_GPS%c_RO_1B_%04d%02d%02dT000000_%04d%02d%02dT235959_%s.ZIP",
                          sat, year, month, day, year, month, day, ver);
            return buf;
        }

        /// 带 CDDIS 镜像的源：那一路要过 Earthdata 认证，连接比原站脆，多给两次机会。
        Dir mirrored(const std::string &origUrl, const ProductType type = ProductType::Unknown) {
            Dir d;
            d.url = origUrl;
            d.type = type;
            d.backupUrl = cddisOf(origUrl);
            d.attempts = 3;
            return d;
        }

        void addUnique(std::vector<Dir> &out, std::set<std::string> &seen, Dir d) {
            if (d.url.empty() || !seen.insert(d.url).second) return;
            out.push_back(std::move(d));
        }

        /// 周目录
        std::vector<Dir> weekDirs(const int year, const int doy) {
            std::vector<Dir> out;
            std::set<std::string> seen;
            const long prevOrd = YearMonthDay{year, 1, 1}.toDays() + doy - 2;
            const YearMonthDay prevYmd = YearMonthDay::fromDays(prevOrd);
            const int prevDoy = static_cast<int>(prevOrd - YearMonthDay{prevYmd.year, 1, 1}.toDays()) + 1;

            char buf[256];
            const std::pair<int, int> days[2] = {{year, doy}, {prevYmd.year, prevDoy}};
            for (const auto &[y, d]: days) {
                const int week = ProductMeta::GPSWeek(y, d);
                std::snprintf(buf, sizeof(buf), "%s/products/%04d/", kIgsRoot, week);
                const std::string plain = buf;
                addUnique(out, seen, mirrored(plain));

                std::snprintf(buf, sizeof(buf), "%s/products/mgex/%04d/", kIgsRoot, week);
                const std::string mgex = buf;
                addUnique(out, seen, mirrored(mgex));
            }
            return out;
        }

        /// Swarm 的约化动力轨道在 swarm-diss 的 POD/RD 下，按星分目录、整目录不按日分页。
        /// 目录里混着建过档的全部日期，只能列出来按名字挑（跨度秒数与版本号随天/基线变，拼不死文件名），
        /// 名字前缀把同目录的 KIN 轨道挡在外面。
        std::string swarmPodDir(const char sat) {
            char buf[192];
            std::snprintf(buf, sizeof(buf), "%s%s%c", kSwarmDissRoot, kSwarmPodQuery, sat);
            return buf;
        }

        std::string swarmSp3Prefix(const char sat) {
            return std::string("SW_OPER_SP3") + sat + "COM";
        }
    }

    std::vector<Dir> IGS(const std::set<ProductType> &types, const int year, const int doy) {
        std::vector<Dir> out;
        std::set<std::string> seen;
        if (year < 1980 || doy < 1) return out;

        const bool wantWeek =
                types.count(ProductType::ORB) || types.count(ProductType::CLK) ||
                types.count(ProductType::ERP) || types.count(ProductType::OSB) ||
                types.count(ProductType::TRO) || types.count(ProductType::ATT);
        if (wantWeek)
            for (auto &d: weekDirs(year, doy)) addUnique(out, seen, std::move(d));

        char buf[256];
        if (types.count(ProductType::NAV)) {
            std::snprintf(buf, sizeof(buf), "%s/data/daily/%04d/brdc/", kIgsRoot, year);
            const std::string u = buf;
            addUnique(out, seen, mirrored(u));
        }
        if (types.count(ProductType::ION)) {
            std::snprintf(buf, sizeof(buf), "%s/products/ionex/%04d/%03d/", kIgsRoot, year, doy);
            const std::string u = buf;
            addUnique(out, seen, mirrored(u));
        }
        // 观测目录按年积日再分 <yy>d / <yy>o 两层，两层的文件名尾段不同
        if (types.count(ProductType::OBS)) {
            for (const char *kind: {"d", "o"}) {
                std::snprintf(buf, sizeof(buf), "%s/data/daily/%04d/%03d/%02d%s/", kIgsRoot, year, doy,
                              year % 100, kind);
                const std::string u = buf;
                addUnique(out, seen, mirrored(u));
            }
        }
        if (types.count(ProductType::VMF)) {
            std::snprintf(buf, sizeof(buf), "%s/%04d/", kVmfRoot, year);
            addUnique(out, seen, {buf, ProductType::Unknown, {}, {}, {}});
        }
        return out;
    }

    std::vector<Dir> daily(const Query &q) {
        std::vector<Dir> out = IGS(q.types, q.year, q.doy);
        std::set<std::string> seen;
        for (const auto &d: out) seen.insert(d.url);
        for (auto &d: codeDir(q.types, q.year)) addUnique(out, seen, std::move(d));
        return out;
    }

    Dir snxWeekDir(const int week) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s/products/%04d/", kIgsRoot, week);
        const std::string u = buf;
        return mirrored(u, ProductType::SNX);
    }

    std::vector<Dir> LEOObs(const int year, const int doy) {
        std::vector<Dir> out;
        std::set<std::string> seen;

        // Sentinel-6A 的日 RINEX 首历元踩在 GPS 日的最后一秒（…前一日 23:59:59.996 GPS 起），
        // CMR 按粒子首历元分 temporal 页，于是 D 日的页面列的是覆盖 D+1 的文件；
        // DOY D 真正要的文件（名字戳 YYYYDDDHHMM=D）在 D-1 日的页面。
        {
            const long first = YearMonthDay{1980, 1, 1}.toDays();
            const long ord = YearMonthDay{year, 1, 1}.toDays() + doy - 1;
            for (const long o: {ord, ord - 1})
                if (o >= first)
                    addUnique(out, seen, {cmrTemporal(YearMonthDay::fromDays(o)), ProductType::LEO_OBS, {}, {}, {}});
        }

        if (const auto ymd = YearMonthDay::fromDOY(year, doy)) {
            for (const char sat: kSwarmSats)
                for (const char *ver: kSwarmVersions) {
                    const std::string name = swarmName(sat, ymd->year, ymd->month, ymd->day, ver);
                    addUnique(out, seen,
                              {std::string(kSwarmRoot) + sat + "/" + name, ProductType::LEO_OBS, name, {}, {}});
                }
        }

        addUnique(out, seen, {ucarLEO(year, doy, "level1a"), ProductType::LEO_OBS, {}, "podCrx", {}});
        return out;
    }

    std::vector<Dir> refTruth(const int year, const int doy) {
        std::vector<Dir> out;
        std::set<std::string> seen;

        // 一条低轨轨道的真值源
        const auto addTruth = [&](const std::string &url, const std::string &mission, const char sat,
                                  const bool autoPick, const std::string &namePrefix = {},
                                  const std::string &backupUrl = {}) {
            Dir d;
            d.url = url;
            d.type = ProductType::LEO_SP3;
            d.namePrefix = namePrefix;
            d.backupUrl = backupUrl;
            d.mission = mission;
            d.sat = sat;
            d.autoPick = autoPick;
            if (!backupUrl.empty()) d.attempts = 3; // 镜像那一路走 CDDIS 认证
            addUnique(out, seen, std::move(d));
        };

        // GFZ RSO：L64/L65 是 GRACE-FO
        for (const char *lv: kGfzLevels) {
            const std::string level = lv;
            const bool grace = level == "L64" || level == "L65";
            addTruth(gzfRSO(lv, year, doy), grace ? "GRACE-FO" : "", 0, level == "L64");
        }

        // COSMIC-2
        {
            const std::string ucar = ucarLEO(year, doy, "level1b");
            char orbName[96];
            std::snprintf(orbName, sizeof(orbName), "leoOrb_nrt_%04d_%03d.tar.gz", year, doy);
            addUnique(out, seen, {ucar + orbName, ProductType::LEO_SP3, orbName, {}, {}, "COSMIC-2", 0, true});
            addTruth(ucar, "COSMIC-2", 0, false, "leoOrb");
        }

        // Sentinel-6A/6B 的 DORIS 弧
        const auto addDoris = [&](const char *center, const char *arc, const char sat,
                                  const bool autoPick) {
            addTruth(std::string(kDorisRoot) + center + '/' + arc + '/', "Sentinel-6", sat, autoPick, {},
                     std::string(kCddisDorisRoot) + center + '/' + arc + '/');
        };
        addDoris("gsc", "s6a", 'A', true);
        addDoris("ssa", "s6a", 'A', false);
        addDoris("ssa", "s6b", 'B', true);

        // Swarm：三颗星各一个整目录，名字前缀挡掉同目录的 KIN 轨道
        for (const char sat: kSwarmSats)
            addTruth(swarmPodDir(sat), "Swarm", sat, true, swarmSp3Prefix(sat));

        for (auto &d: weekDirs(year, doy)) {
            if (d.url.find("/mgex/") != std::string::npos) continue;
            d.type = ProductType::SNX;
            addUnique(out, seen, std::move(d));
        }
        for (auto &d: codeDir({ProductType::ORB}, year)) addUnique(out, seen, std::move(d));
        return out;
    }

    std::vector<Dir> codeDir(const std::set<ProductType> &types, const int year) {
        const bool wanted = types.count(ProductType::ORB) || types.count(ProductType::CLK) ||
                            types.count(ProductType::ERP) || types.count(ProductType::OSB) ||
                            types.count(ProductType::TRO) || types.count(ProductType::ATT) ||
                            types.count(ProductType::ION) || types.count(ProductType::SNX);
        if (!wanted || year < 1980) return {};
        return {{aiubCode(year), ProductType::Unknown, {}, kCodePrefix, {}}};
    }

    char SwarmSat(const std::string &obsName) {
        const std::string n = Str::toUpper(File::baseName(obsName));
        for (const char sat: kSwarmSats)
            if (n.find(std::string("GPS") + sat + "_RO") != std::string::npos) return sat;
        if (const std::size_t at = n.find("SWARM"); at != std::string::npos)
            for (std::size_t i = at + 5; i < n.size() && i <= at + 8; ++i)
                if (n[i] == 'A' || n[i] == 'B' || n[i] == 'C') return n[i];
        return 0;
    }

    std::string LEOMission(const std::string &obsName) {
        std::string n = Str::toUpper(File::baseName(obsName));
        // 分片合并后按头里的 MARKER NAME 命名（如 C2E2_L77_POD1），前缀只为与原始分片区分。
        if (n.rfind("MERGED_", 0) == 0) n = n.substr(7);
        if (n.rfind("S6A", 0) == 0 || n.rfind("S6B", 0) == 0 || n.find("SENTINEL-6") != std::string::npos)
            return "Sentinel-6";
        if (n.rfind("SW_", 0) == 0 || n.find("SWARM") != std::string::npos) return "Swarm";
        if (n.find("PODCRX") != std::string::npos || n.rfind("C2", 0) == 0) return "COSMIC-2";
        if (n.find("GRACE") != std::string::npos) return "GRACE-FO";
        if (n.find("TSX") != std::string::npos || n.find("TANDEM") != std::string::npos) return "TerraSAR-X";
        return {};
    }

    char LEOSat(const std::string &obsName) {
        const std::string n = Str::toUpper(File::baseName(obsName));
        if (n.find("S6B") != std::string::npos) return 'B';
        if (n.find("S6A") != std::string::npos) return 'A';
        return SwarmSat(obsName);
    }

    bool orbitIsFor(const std::string &mission, const char sat, const std::string &fileName) {
        const std::string n = Str::toUpper(File::baseName(fileName));
        if (mission == "Sentinel-6") return n.find(sat == 'B' ? "S6B" : "S6A") != std::string::npos;
        if (mission == "Swarm") {
            return sat != 0 && n.find(std::string("SP3") + sat + "COM") != std::string::npos;
        }
        if (mission == "GRACE-FO") return n.find("GRACE") != std::string::npos;
        return false;
    }

    std::vector<Dir> LEOTruths(const std::string &obsName, const int year, const int doy) {
        const std::string mission = LEOMission(obsName);
        if (mission.empty()) return {};
        const char sat = LEOSat(obsName);
        std::vector<Dir> out;
        for (auto &d: refTruth(year, doy))
            if (d.mission == mission && d.autoPick && (d.sat == 0 || d.sat == sat))
                out.push_back(std::move(d));
        return out;
    }
}
