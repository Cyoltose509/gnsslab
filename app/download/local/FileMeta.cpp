#include "download/local/FileMeta.h"
#include "FileUtils.h"
#include "core/TimeStruct.h"
#include "StringUtils.h"

#include <regex>
#include <cstdio>
#include <cctype>
#include <algorithm>
#include <string_view>

namespace {
    std::string stripArchiveSuffix(const std::string &s) {
        for (const char *sfx: {".gz", ".zip", ".bz2", ".z"})
            if (const std::string t = Str::stripSuffixIC(s, sfx); t.size() != s.size()) return t;
        return s;
    }

    bool isSpanToken(const std::string &tok) {
        if (tok.size() != 3) return false;
        if (std::isdigit(static_cast<unsigned char>(tok[0])) == 0) return false;
        if (std::isdigit(static_cast<unsigned char>(tok[1])) == 0) return false;
        return tok[2] == 'D' || tok[2] == 'H' || tok[2] == 'M' || tok[2] == 'S' ||
               tok[2] == 'C' || tok[2] == 'Y';
    }

    bool isRINEX2Token(const std::string &tok, const std::string_view letters) {
        if (tok.size() != 3) return false;
        if (std::isdigit(static_cast<unsigned char>(tok[0])) == 0) return false;
        if (std::isdigit(static_cast<unsigned char>(tok[1])) == 0) return false;
        return letters.find(tok[2]) != std::string_view::npos;
    }

    std::optional<FileMeta> parseRINEX2ShortName(const std::string &filename) {
        std::string base = File::baseName(filename);
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
        else if (t == 'n' || t == 'g') type = ProductType::NAV;
        else return std::nullopt;

        FileMeta m;
        m.filename = filename;
        m.type = type;
        m.year = year;
        m.doy = doy;
        m.spanDays = 1;
        m.ext = std::string(1, t);
        return m;
    }

    std::optional<FileMeta> parseDorisSP3(const std::string &filename) {
        if (Str::toLower(filename).find(".sp3") == std::string::npos) return std::nullopt;

        static const std::regex re(R"(\.b(\d{2})(\d{3})\.e(\d{2})(\d{3})\.)");
        std::smatch sm;
        if (!std::regex_search(filename, sm, re)) return std::nullopt;

        const auto century = [](const int yy) { return yy < 80 ? 2000 + yy : 1900 + yy; };
        const int year = century(Str::toInt(sm[1].str()));
        const int doy = Str::toInt(sm[2].str());
        const int endYear = century(Str::toInt(sm[3].str()));
        const int endDoy = Str::toInt(sm[4].str());
        if (doy < 1 || doy > 366 || endDoy < 1 || endDoy > 366) return std::nullopt;

        const long start = YearMonthDay{year, 1, 1}.toDays() + doy - 1;
        const long end = YearMonthDay{endYear, 1, 1}.toDays() + endDoy - 1;
        if (end < start) return std::nullopt;

        FileMeta m;
        m.filename = filename;
        m.type = ProductType::LEO_SP3;
        m.year = year;
        m.doy = doy;
        m.spanDays = static_cast<int>(end - start) + 1;
        m.ext = "sp3";
        m.datacenter = Str::toUpper(filename.substr(0, 3));
        return m;
    }

    /// 按页面语境列目录时，同一页里混着导航链接、样式表与说明 PDF，只有这些后缀才是数据文件。
    bool acceptsAs(const ProductType dirType, const std::string &name) {
        switch (dirType) {
            case ProductType::LEO_OBS:
                return Str::endsWithIC(name, ".rnx") || Str::endsWithIC(name, ".crx") ||
                       Str::endsWithIC(name, ".zip") || Str::endsWithIC(name, ".gz") ||
                       Str::endsWithIC(name, ".tar") || Str::endsWithIC(name, ".z");
            case ProductType::LEO_SP3:
                return Str::endsWithIC(name, ".sp3") || Str::endsWithIC(name, "_sp3") ||
                       Str::endsWithIC(name, ".gz") || Str::endsWithIC(name, ".tar") ||
                       Str::endsWithIC(name, ".z") || Str::endsWithIC(name, ".zip");
            case ProductType::SNX:
                return Str::endsWithIC(name, ".snx") || Str::endsWithIC(name, ".snx.gz");
            default:
                return true;
        }
    }
}

std::optional<FileMeta> FileMeta::parse(const std::string &filename) {
    FileMeta m;
    m.filename = filename;

    if (const std::string lowAll = Str::toLower(filename); lowAll.find("vmf") != std::string::npos) {
        static const std::regex reYmd(R"((\d{4})(\d{2})(\d{2}))");
        static const std::regex reHour(R"(\.H(\d{2}))");
        if (std::smatch sm; std::regex_search(filename, sm, reYmd)) {
            const int y = Str::toInt(sm[1].str());
            const int mo = Str::toInt(sm[2].str());
            if (const int d = Str::toInt(sm[3].str()); mo >= 1 && mo <= 12 && d >= 1 && d <= 31) {
                m.year = y;
                m.doy = YearMonthDay{y, mo, d}.toDOY();
                m.type = ProductType::VMF;
                m.spanDays = 1;
                if (std::smatch hs; std::regex_search(filename, hs, reHour))
                    m.slot = "H" + hs[1].str();
                m.datacenter = Str::leadingAlpha(filename);
                if (const auto dot = filename.find_last_of('.'); dot != std::string::npos)
                    m.ext = filename.substr(dot + 1);
                m.interval = "06H";
                return m;
            }
        }
    }


    // ISO 双时标名（Swarm 的低轨产品）：_YYYYMMDDThhmmss_YYYYMMDDThhmmss_。
    {
        static const std::regex reIso(R"((\d{4})(\d{2})(\d{2})T\d{6})");
        long first = 0, last = 0;
        for (auto it = std::sregex_iterator(filename.begin(), filename.end(), reIso);
             it != std::sregex_iterator(); ++it) {
            const int y = Str::toInt((*it)[1].str());
            const int mo = Str::toInt((*it)[2].str());
            const int d = Str::toInt((*it)[3].str());
            if (mo < 1 || mo > 12 || d < 1 || d > 31) continue;
            const long day = YearMonthDay{y, mo, d}.toDays();
            if (first == 0) first = day;
            last = day;
        }
        if (first > 0) {
            const long span = last > first ? last - first : 0;
            const long anchor = span <= 1 ? last : first;
            const YearMonthDay ymd = YearMonthDay::fromDays(static_cast<int>(anchor));
            m.year = ymd.year;
            m.doy = static_cast<int>(anchor - YearMonthDay{ymd.year, 1, 1}.toDays()) + 1;
            m.spanDays = span <= 1 ? 1 : static_cast<int>(span) + 1;
            m.stamped = true;
            m.datacenter = Str::leadingAlpha(filename);
            if (const auto dot = filename.find_last_of('.'); dot != std::string::npos)
                m.ext = filename.substr(dot + 1);
            return m;
        }
    }

    if (auto r2 = parseRINEX2ShortName(filename); r2) return r2;
    if (auto doris = parseDorisSP3(filename); doris) return doris;

    static const std::regex reDate(R"((\d{4})(\d{3}))");
    auto it = std::sregex_iterator(filename.begin(), filename.end(), reDate);
    if (it == std::sregex_iterator()) return std::nullopt;
    m.year = Str::toInt((*it)[1].str());
    m.doy = Str::toInt((*it)[2].str());

    // RINEX3/IGS 长名的时刻戳是 11 位（YYYYDDDHHMM，如 S6AP00EUM_R_20262160000_01D）：
    {
        if (const auto pos = static_cast<std::size_t>(it->position(0));
            pos + 11 <= filename.size() && std::all_of(filename.begin() + static_cast<long>(pos) + 7,
                                                       filename.begin() + static_cast<long>(pos) + 11,
                                                       [](const unsigned char c) { return std::isdigit(c); }))
            m.stamped = true;
    }


    {
        // 日期后第一个「两位数字 + 单位」是周期，第二个才是采样；导航文件没有采样段，紧跟的是类型。
        const auto afterDate = static_cast<std::size_t>(it->position(0) + it->length(0));
        if (const auto periodAt = filename.find('_', afterDate); periodAt != std::string::npos && periodAt + 4 <= filename.size()) {
            if (const std::string period = filename.substr(periodAt + 1, 3); isSpanToken(period)) {
                if (period[2] == 'D') m.spanDays = (period[0] - '0') * 10 + (period[1] - '0');
                if (const auto sampleAt = filename.find('_', periodAt + 1);
                    sampleAt != std::string::npos && sampleAt + 4 <= filename.size()) {
                    if (const std::string sample = filename.substr(sampleAt + 1, 3); isSpanToken(sample))
                        m.interval = sample;
                }
            }
        }
    }


    m.datacenter = Str::leadingAlpha(filename);


    std::string base = Str::stripSuffixIC(filename, ".gz");
    auto us = base.find_last_of('_');
    std::string tail = us != std::string::npos ? base.substr(us + 1) : base;
    auto dot = tail.find_last_of('.');
    std::string typeTok = dot != std::string::npos ? tail.substr(0, dot) : tail;
    std::string ext = dot != std::string::npos ? tail.substr(dot + 1) : "";
    m.ext = ext;

    const std::string low = Str::toLower(filename);

    // 气象 MM 与 *.rnx 同为 RINEX3 长名，不先拦下会被下面的 NAV 分支当成广播星历
    if (typeTok == "MM") return m;

    if (typeTok == "ATT" || ext == "OBX") m.type = ProductType::ATT;
    else if (typeTok == "MO" || isRINEX2Token(typeTok, "doO")) m.type = ProductType::OBS;
    else if (typeTok == "ORB" || ext == "SP3") m.type = ProductType::ORB;
    else if (typeTok == "CLK" || ext == "CLK") m.type = ProductType::CLK;
    else if (typeTok == "ERP" || ext == "ERP") m.type = ProductType::ERP;
    else if (typeTok == "ION" || ext == "INX" || ext == "IONEX") m.type = ProductType::ION;
    else if (typeTok == "TRO" || ext == "TRO") m.type = ProductType::TRO;
    else if (typeTok == "OSB" || typeTok == "BIA" || ext == "OSB" || ext == "BIA")
        m.type = ProductType::OSB;
    else if (ext == "SNX") m.type = ProductType::SNX;
    else if (typeTok == "MN" || ext == "rnx" || ext == "NAV" ||
             low.find("brdc") != std::string::npos || isRINEX2Token(ext, "nN"))
        m.type = ProductType::NAV;
    return m;
}

std::optional<FileMeta> FileMeta::parseInDir(const std::string &filename, const ProductType dirType,
                                                   const int dateYear, const int dateDoy) {
    if (dirType == ProductType::Unknown) return parse(filename);
    if (!acceptsAs(dirType, filename)) return std::nullopt;

    FileMeta m = parse(filename).value_or(FileMeta{});
    m.filename = filename;
    m.type = dirType;
    if (m.spanDays > 1 && !m.covers(dateYear, dateDoy)) return std::nullopt;
    if (!m.covers(dateYear, dateDoy)) {
        if (m.stamped) return std::nullopt; // 自带起止时刻却不覆盖这天：不是这一天的文件
        m.year = dateYear;
        m.doy = dateDoy;
        m.spanDays = 1;
    }
    return m;
}

bool FileMeta::covers(const int y, const int d) const {
    if (year < 1980 || doy < 1 || spanDays < 1) return false;
    const long start = YearMonthDay{year, 1, 1}.toDays() + doy - 1;
    const long day = YearMonthDay{y, 1, 1}.toDays() + d - 1;
    return day >= start && day <= start + spanDays - 1;
}

std::string FileMeta::productLineOf(const std::string &filename) {
    const std::string name = Str::toUpper(Str::stripSuffixIC(File::baseName(filename), ".gz"));
    if (name.size() < 9 || name[3] != '0') return {};
    const std::string center = Str::leadingAlpha(name);
    const std::string line = name.substr(4, 3);
    for (const char c: line)
        if (!std::isupper(static_cast<unsigned char>(c))) return {};
    return center + "/" + line;
}
