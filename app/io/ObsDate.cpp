#include "io/ObsDate.h"

#include "FileUtils.h"
#include "StringUtils.h"
#include "core/TimeStruct.h"
#include "util/Clock.h"

#include <algorithm>
#include <cctype>
#include <regex>

namespace {
    /// 下载库布局 <任意字母目录>/<年>/<日序>/：只认形状，不认目录名是哪种产品，
    /// 免得为一个兜底去依赖产品词表。
    std::optional<ObsDate> dateFromStoreLayout(const std::string &path) {
        static const std::regex re(R"((?:^|[\\/])([A-Za-z_]+)[\\/](\d{4})[\\/](\d{3})[\\/])");
        std::smatch m;
        if (!std::regex_search(path, m, re)) return std::nullopt;
        const int year = Str::toInt(m[2].str());
        const int doy = Str::toInt(m[3].str());
        if (year < 1980 || doy < 1 || doy > 366) return std::nullopt;
        return ObsDate{year, doy, GPSWeek(year, doy), 0.0, ObsDateFrom::Name};
    }
}

std::optional<ObsDate> parseObsDate(const std::string &obspath) {
    std::string f = Str::stripSuffixIC(File::baseName(obspath), ".gz");
    if (f.empty()) return std::nullopt;

    // RINEX3 长名：_YYYYDDDHHMM_
    static const std::regex re3(R"(_(\d{4})(\d{3})\d{4}_)");
    if (std::smatch m; std::regex_search(f, m, re3)) {
        const int year = Str::toInt(m[1].str());
        const int doy = Str::toInt(m[2].str());
        return ObsDate{year, doy, GPSWeek(year, doy), 0.0, ObsDateFrom::Name};
    }

    if (f.rfind("merged_", 0) == 0) {
        static const std::regex reMerged(R"(_(\d{4})_(\d{3})(?:\.|$))");
        if (std::smatch m; std::regex_search(f, m, reMerged)) {
            const int year = Str::toInt(m[1].str());
            const int doy = Str::toInt(m[2].str());
            if (year >= 1980 && doy >= 1 && doy <= 366)
                return ObsDate{year, doy, GPSWeek(year, doy), 0.0, ObsDateFrom::Name};
        }
    }

    // RINEX2 短名：ssssDDD0.YYO
    if (const auto dot = f.find_last_of('.'); dot != std::string::npos && dot + 3 <= f.size()) {
        if (const std::string suf = f.substr(dot + 1); suf.size() >= 3 &&
                                                       std::isdigit(static_cast<unsigned char>(suf[0])) &&
                                                       std::isdigit(static_cast<unsigned char>(suf[1])) &&
                                                       std::isalpha(static_cast<unsigned char>(suf[2]))) {
            const int yy = Str::toInt(suf.substr(0, 2));
            const int year = yy < 80 ? 2000 + yy : 1900 + yy;
            const std::string base = f.substr(0, dot);
            for (const std::size_t back: {static_cast<std::size_t>(4), static_cast<std::size_t>(3)}) {
                if (base.size() < back) continue;
                if (const std::string d3 = base.substr(base.size() - back, 3); std::all_of(d3.begin(), d3.end(),
                                                                                           [](const unsigned char c) {
                                                                                               return std::isdigit(c);
                                                                                           })) {
                    if (const int doy = Str::toInt(d3); doy >= 1 && doy <= 366)
                        return ObsDate{year, doy, GPSWeek(year, doy), 0.0, ObsDateFrom::Name};
                }
            }
        }
    }

    // 路径里任意位置的 YYYYMMDD（观测目录常按年月日分层）
    static const std::regex reDate(R"((\d{8}))");
    for (auto it = std::sregex_iterator(obspath.begin(), obspath.end(), reDate);
         it != std::sregex_iterator(); ++it) {
        const std::string s = (*it)[1].str();
        const int y = Str::toInt(s.substr(0, 4));
        const int mo = Str::toInt(s.substr(4, 2));
        const int da = Str::toInt(s.substr(6, 2));
        if (const int doy = YearMonthDay{y, mo, da}.toDOY(); y >= 1980 && doy > 0)
            return ObsDate{y, doy, GPSWeek(y, doy), 0.0, ObsDateFrom::Name};
    }

    // 名字里认不出日期：读文件内容（头里的 TIME OF FIRST OBS → 首历元），GPS 周也一并拿到
    if (ObsDate d; Observer::probeObsDate(obspath, d))
        return ObsDate{d.year, d.doy, d.week > 0 ? d.week : GPSWeek(d.year, d.doy), d.sow, d.from};

    return dateFromStoreLayout(obspath);
}

ObsDate defaultObsDate() {
    const std::tm tm = Clock::localTime(Clock::wallNow() - Clock::hours(24));
    const int year = tm.tm_year + 1900;
    const int doy = YearMonthDay{year, tm.tm_mon + 1, tm.tm_mday}.toDOY();
    return ObsDate{year, doy > 0 ? doy : 1, GPSWeek(year, doy > 0 ? doy : 1), 0.0, ObsDateFrom::Name};
}
