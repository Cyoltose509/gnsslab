#include "ProductMeta.h"
#include "TimeConvert.h"
#include "StringUtils.h"

#include <regex>
#include <cstdio>
#include <chrono>
#include <ctime>
#include <cctype>
#include <algorithm>

namespace {
    std::string stripArchiveSuffix(const std::string &s) {
        for (const char *sfx: {".gz", ".zip", ".bz2", ".z"})
            if (const std::string t = Str::stripSuffixIC(s, sfx); t.size() != s.size()) return t;
        return s;
    }

    std::optional<ProductMeta> parseRINEX2ShortName(const std::string &filename) {
        std::string base = Str::baseName(filename);
        for (;;) {
            const std::string cut = stripArchiveSuffix(base);
            if (cut == base) break;
            base = cut;
        }
        if (base.size() < 12 || base[8] != '.') return std::nullopt;

        const auto isAlnum = [](const char c) {
            return std::isalnum(static_cast<unsigned char>(c)) != 0;
        };
        const auto isDig = [](const char c) {
            return std::isdigit(static_cast<unsigned char>(c)) != 0;
        };
        for (int i = 0; i < 4; ++i)
            if (!isAlnum(base[i])) return std::nullopt;
        for (int i = 4; i <= 6; ++i)
            if (!isDig(base[i])) return std::nullopt;
        if (!isAlnum(base[7]) || !isDig(base[9]) || !isDig(base[10])) return std::nullopt;

        const int doy = (base[4] - '0') * 100 + (base[5] - '0') * 10 + (base[6] - '0');
        if (doy < 1 || doy > 366) return std::nullopt;
        const int yy = (base[9] - '0') * 10 + (base[10] - '0');
        const int year = yy < 80 ? 2000 + yy : 1900 + yy;

        const char t = static_cast<char>(std::tolower(static_cast<unsigned char>(base[11])));
        ProductType type;
        if (t == 'o' || t == 'd') type = ProductType::OBS;
        else if (t == 'm') type = ProductType::MET;
        else if (t == 'n' || t == 'g') type = ProductType::NAV;
        else return std::nullopt;

        ProductMeta m;
        m.filename = filename;
        m.type = type;
        m.year = year;
        m.doy = doy;
        m.spanDays = 1;
        m.ext = std::string(1, t);
        return m;
    }
}

std::optional<ProductMeta> ProductMeta::parse(const std::string &filename) {
    ProductMeta m;
    m.filename = filename;

    if (const std::string lowAll = Str::toLower(filename); lowAll.find("vmf") != std::string::npos) {
        std::regex reYmd(R"((\d{4})(\d{2})(\d{2}))");
        if (std::smatch sm; std::regex_search(filename, sm, reYmd)) {
            const int y = std::stoi(sm[1].str());
            const int mo = std::stoi(sm[2].str());
            if (const int d = std::stoi(sm[3].str()); mo >= 1 && mo <= 12 && d >= 1 && d <= 31) {
                m.year = y;
                m.doy = Time::YMDToDOY({y, mo, d});
                m.type = ProductType::VMF;
                m.spanDays = 1;
                if (std::smatch hs; std::regex_search(filename, hs, std::regex(R"(\.H(\d{2}))")))
                    m.slot = "H" + hs[1].str();
                if (std::smatch ds; std::regex_search(filename, ds, std::regex(R"(^([A-Za-z]+))")))
                    m.datacenter = ds[1].str();
                if (const auto dot = filename.find_last_of('.'); dot != std::string::npos)
                    m.ext = filename.substr(dot + 1);
                m.interval = "06H";
                return m;
            }
        }
    }


    if (auto r2 = parseRINEX2ShortName(filename); r2) return r2;

    std::regex reDate(R"((\d{4})(\d{3}))");
    auto it = std::sregex_iterator(filename.begin(), filename.end(), reDate);
    if (it == std::sregex_iterator()) return std::nullopt;
    m.year = std::stoi((*it)[1].str());
    m.doy = std::stoi((*it)[2].str());


    {
        const auto afterDate = static_cast<std::size_t>(it->position(0) + it->length(0));
        if (const auto u = filename.find('_', afterDate); u != std::string::npos && u + 3 < filename.size()) {
            if (const std::string tok = filename.substr(u + 1, 3); std::isdigit(static_cast<unsigned char>(tok[0])) &&
                                                                   std::isdigit(static_cast<unsigned char>(tok[1]))) {
                if (tok[2] == 'D') {
                    m.spanDays = (tok[0] - '0') * 10 + (tok[1] - '0');
                    if (const auto v = filename.find('_', u + 1); v != std::string::npos && v + 3 < filename.size())
                        m.interval = filename.substr(v + 1, 3);
                } else {
                    m.interval = tok;
                }
            }
        }
    }


    std::regex reDc(R"(^([A-Za-z]+))");
    if (std::smatch dm; std::regex_search(filename, dm, reDc)) m.datacenter = dm[1].str();


    std::string base = Str::stripSuffixIC(filename, ".gz");
    auto us = base.find_last_of('_');
    std::string tail = us != std::string::npos ? base.substr(us + 1) : base;
    auto dot = tail.find_last_of('.');
    std::string typeTok = dot != std::string::npos ? tail.substr(0, dot) : tail;
    std::string ext = dot != std::string::npos ? tail.substr(dot + 1) : "";
    m.ext = ext;

    const std::string low = Str::toLower(filename);

    if (typeTok == "ATT" || ext == "OBX") m.type = ProductType::ATT;
    else if (typeTok == "MM") m.type = ProductType::MET;
    else if (typeTok == "MO" ||
             (typeTok.size() == 3 && std::isdigit(static_cast<unsigned char>(typeTok[0])) &&
              std::isdigit(static_cast<unsigned char>(typeTok[1])) &&
              (typeTok[2] == 'd' || typeTok[2] == 'o' || typeTok[2] == 'O')))
        m.type = ProductType::OBS;
    else if (typeTok == "ORB" || ext == "SP3") m.type = ProductType::ORB;
    else if (typeTok == "CLK" || ext == "CLK") m.type = ProductType::CLK;
    else if (typeTok == "ERP" || ext == "ERP") m.type = ProductType::ERP;
    else if (typeTok == "ION" || ext == "INX" || ext == "IONEX") m.type = ProductType::ION;
    else if (typeTok == "TRO" || ext == "TRO") m.type = ProductType::TRO;
    else if (typeTok == "OSB" || typeTok == "BIA" ||
             ext == "OSB" || ext == "BIA")
        m.type = ProductType::OSB;

    else if (typeTok == "MN" || ext == "rnx" || ext == "NAV" ||
             low.find("brdc") != std::string::npos ||
             (ext.size() == 3 && std::isdigit(static_cast<unsigned char>(ext[0])) &&
              std::isdigit(static_cast<unsigned char>(ext[1])) &&
              (ext[2] == 'n' || ext[2] == 'N')))
        m.type = ProductType::NAV;
    else m.type = ProductType::Unknown;
    return m;
}

std::string ProductMeta::typeDir(const ProductType t) {
    switch (t) {
        case ProductType::NAV: return "nav";
        case ProductType::ORB: return "sp3";
        case ProductType::CLK: return "clk";
        case ProductType::ERP: return "erp";
        case ProductType::ION: return "ionex";
        case ProductType::TRO: return "tro";
        case ProductType::OSB: return "osb";
        case ProductType::VMF: return "vmf";
        case ProductType::ATT: return "att";
        case ProductType::MET: return "met";
        case ProductType::OBS: return "rinex";
        default: return "other";
    }
}

ProductType ProductMeta::typeFromIniKey(const std::string &iniKey) {
    const std::string s = Str::toLower(iniKey);
    if (s.find("sp3") != std::string::npos) return ProductType::ORB;
    if (s.find("clk") != std::string::npos) return ProductType::CLK;
    if (s.find("erp") != std::string::npos) return ProductType::ERP;
    if (s.find("ionex") != std::string::npos ||
        s.find("ion") != std::string::npos)
        return ProductType::ION;
    if (s.find("tro") != std::string::npos) return ProductType::TRO;
    if (s.find("osb") != std::string::npos ||
        s.find("bia") != std::string::npos)
        return ProductType::OSB;
    if (s.find("brdc") != std::string::npos ||
        s.find("nav") != std::string::npos)
        return ProductType::NAV;
    return ProductType::Unknown;
}

std::string ProductMeta::doyToDateStr(const int year, const int doy) {
    const auto ymd = Time::DOYToYMD(year, doy);
    if (!ymd) return "";
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d", ymd->year, ymd->month, ymd->day);
    return buf;
}

std::optional<std::pair<int, int> > ProductMeta::parseObsDate(const std::string &obsPath) {
    std::string f = Str::stripSuffixIC(Str::baseName(obsPath), ".gz");
    if (f.empty()) return std::nullopt;


    std::regex re3(R"(_(\d{4})(\d{3})\d{4}_)");
    if (std::smatch m; std::regex_search(f, m, re3))
        return std::make_pair(std::stoi(m[1].str()), std::stoi(m[2].str()));


    if (const auto dot = f.find_last_of('.'); dot != std::string::npos && dot + 3 <= f.size()) {
        if (const std::string suf = f.substr(dot + 1); suf.size() >= 3 &&
                                                       std::isdigit(static_cast<unsigned char>(suf[0])) &&
                                                       std::isdigit(static_cast<unsigned char>(suf[1])) &&
                                                       std::isalpha(static_cast<unsigned char>(suf[2]))) {
            const int yy = std::stoi(suf.substr(0, 2));
            const int year = yy < 80 ? 2000 + yy : 1900 + yy;
            const std::string base = f.substr(0, dot);
            for (const std::size_t back: {static_cast<std::size_t>(4), static_cast<std::size_t>(3)}) {
                if (base.size() < back) continue;
                if (const std::string d3 = base.substr(base.size() - back, 3); std::all_of(d3.begin(), d3.end(),
                                                                                           [](const unsigned char c) {
                                                                                               return std::isdigit(c);
                                                                                           })) {
                    if (const int doy = std::stoi(d3); doy >= 1 && doy <= 366) return std::make_pair(year, doy);
                }
            }
        }
    }


    std::regex reDate(R"((\d{8}))");
    for (auto it = std::sregex_iterator(obsPath.begin(), obsPath.end(), reDate);
         it != std::sregex_iterator(); ++it) {
        const std::string s = (*it)[1].str();
        const int y = std::stoi(s.substr(0, 4));
        const int mo = std::stoi(s.substr(4, 2));
        const int da = std::stoi(s.substr(6, 2));
        if (const int doy = Time::YMDToDOY({y, mo, da}); y >= 1980 && doy > 0) return std::make_pair(y, doy);
    }
    return std::nullopt;
}

std::pair<int, int> ProductMeta::defaultObsDate() {
    const auto now = std::chrono::system_clock::now() - std::chrono::hours(24);
    const std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tm, &tt);
#endif
    const int year = tm.tm_year + 1900;
    const int doy = Time::YMDToDOY({year, tm.tm_mon + 1, tm.tm_mday});
    return {year, doy > 0 ? doy : 1};
}


const char *ProductMeta::mirrorRoot() {
    return "ftp://igs.gnsswhu.cn/pub/gps";
}

const char *ProductMeta::vmfRoot() {
    return "https://vmf.geo.tuwien.ac.at/trop_products/GRID/5x5/VMF3/VMF3_OP";
}

int ProductMeta::gpsWeek(const int year, const int doy) {
    if (doy < 1 || doy > 366) return 0;


    const long days = Time::civilToYMD({year, 1, 1}) + doy - 1 - Time::civilToYMD({1980, 1, 6});
    if (days < 0) return 0;
    return static_cast<int>(days / 7);
}

bool ProductMeta::covers(const int _year, const int _doy) const {
    if (this->year < 1980 || this->doy < 1 || this->spanDays < 1) return false;
    const long start = Time::civilToYMD({this->year, 1, 1}) + this->doy - 1;
    const long day = Time::civilToYMD({_year, 1, 1}) + _doy - 1;
    return day >= start && day <= start + spanDays - 1;
}

const std::vector<std::string> &ProductMeta::knownDatacenters() {
    static const std::vector<std::string> v = {
        "IGS", "COD", "EMR", "ESA", "GFZ", "GRG", "JGX",
        "JPL", "NGS", "SHA", "SIO", "USN", "WHU", "WUM"
    };
    return v;
}

std::vector<std::string> ProductMeta::scanDirs(const std::set<ProductType> &types, int year, int doy) {
    std::vector<std::string> out;
    const auto add = [&out](const std::string &u) {
        if (std::find(out.begin(), out.end(), u) == out.end()) out.push_back(u);
    };
    if (year < 1980 || doy < 1) return out;

    const bool wantWeek =
            types.count(ProductType::ORB) || types.count(ProductType::CLK) ||
            types.count(ProductType::ERP) || types.count(ProductType::OSB) ||
            types.count(ProductType::TRO) || types.count(ProductType::ATT);
    if (wantWeek) {
        const long prevOrd = Time::civilToYMD({year, 1, 1}) + doy - 2;
        const YearMonthDay prevYmd = Time::YMDToCivil(prevOrd);
        const int prevDoy = static_cast<int>(prevOrd - Time::civilToYMD({prevYmd.year, 1, 1})) + 1;

        char buf[256];
        const std::pair<int, int> days2[2] = {{year, doy}, {prevYmd.year, prevDoy}};
        for (const auto &[fst, snd]: days2) {
            std::snprintf(buf, sizeof(buf), "%s/products/%04d/", mirrorRoot(), gpsWeek(fst, snd));
            add(buf);

            std::snprintf(buf, sizeof(buf), "%s/products/mgex/%04d/", mirrorRoot(), gpsWeek(fst, snd));
            add(buf);
        }
    }
    if (types.count(ProductType::NAV)) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s/data/daily/%04d/brdc/", mirrorRoot(), year);
        add(buf);
    }
    if (types.count(ProductType::ION)) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s/products/ionex/%04d/%03d/", mirrorRoot(), year, doy);
        add(buf);
    }
    if (types.count(ProductType::MET)) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s/data/daily/%04d/%03d/%02dm/", mirrorRoot(), year, doy,
                      year % 100);
        add(buf);
        std::snprintf(buf, sizeof(buf), "%s/data/daily/%04d/%03d/%02dd/", mirrorRoot(), year, doy,
                      year % 100);
        add(buf);
    }
    if (types.count(ProductType::OBS)) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s/data/daily/%04d/%03d/%02dd/", mirrorRoot(), year, doy,
                      year % 100);
        add(buf);
        std::snprintf(buf, sizeof(buf), "%s/data/daily/%04d/%03d/%02do/", mirrorRoot(), year, doy,
                      year % 100);
        add(buf);
    }
    if (types.count(ProductType::VMF)) {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s/%04d/", vmfRoot(), year);
        add(buf);
    }

    return out;
}
