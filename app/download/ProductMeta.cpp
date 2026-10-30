#include "ProductMeta.h"
#include "TimeConvert.h"

#include <regex>
#include <cstdio>
#include <cctype>
#include <algorithm>

std::optional<ProductMeta> ProductMeta::parse(const std::string &filename) {
    ProductMeta m;
    m.filename = filename;


    std::regex reDate(R"((\d{4})(\d{3}))");
    auto it = std::sregex_iterator(filename.begin(), filename.end(), reDate);
    if (it == std::sregex_iterator()) return std::nullopt;
    m.year = std::stoi((*it)[1].str());
    m.doy = std::stoi((*it)[2].str());



    {
        const auto afterDate = static_cast<size_t>(it->position(0) + it->length(0));
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


    std::string base = filename;
    {
        std::string l = base;
        for (auto &c: l) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (l.size() > 3 && l.compare(l.size() - 3, 3, ".gz") == 0) base.erase(base.size() - 3);
    }
    auto us = base.find_last_of('_');
    std::string tail = us != std::string::npos ? base.substr(us + 1) : base;
    auto dot = tail.find_last_of('.');
    std::string typeTok = dot != std::string::npos ? tail.substr(0, dot) : tail;
    std::string ext = dot != std::string::npos ? tail.substr(dot + 1) : "";
    m.ext = ext;

    std::string low = filename;
    for (auto &c: low) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    if (typeTok == "ORB" || ext == "SP3") m.type = ProductType::ORB;
    else if (typeTok == "CLK" || ext == "CLK") m.type = ProductType::CLK;
    else if (typeTok == "ERP" || ext == "ERP") m.type = ProductType::ERP;
    else if (typeTok == "ION" || ext == "INX" || ext == "IONEX") m.type = ProductType::ION;
    else if (typeTok == "TRO" || ext == "TRO") m.type = ProductType::TRO;
    else if (typeTok == "OSB" || typeTok == "BIA" ||
             ext == "OSB" || ext == "BIA")
        m.type = ProductType::OSB;
    else if (ext == "ATX") m.type = ProductType::ATX;

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
        case ProductType::ATX: return "atx";
        default: return "other";
    }
}

ProductType ProductMeta::typeFromIniKey(const std::string &iniKey) {
    std::string s = iniKey;
    std::transform(s.begin(), s.end(), s.begin(),
                   [](const unsigned char c) { return static_cast<char>(std::tolower(c)); });
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
    if (s.find("atx") != std::string::npos) return ProductType::ATX;
    if (s.find("brdc") != std::string::npos ||
        s.find("nav") != std::string::npos)
        return ProductType::NAV;
    return ProductType::Unknown;
}

std::string ProductMeta::doyToDateStr(const int year, const int doy) {
    int y = 0, m = 0, d = 0;
    if (!doyToYmd(year, doy, y, m, d)) return "";
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d", y, m, d);
    return buf;
}

bool ProductMeta::doyToYmd(const int year, const int doy, int &oy, int &om, int &od) {
    if (doy < 1 || doy > 366) return false;
    int y = 0;
    unsigned m = 0, d = 0;
    civilFromDays(daysFromCivil(year, 1, 1) + doy - 1, y, m, d);
    oy = y;
    om = static_cast<int>(m);
    od = static_cast<int>(d);
    return true;
}

int ProductMeta::dateToDoy(const int year, const int month, const int day) {
    if (month < 1 || month > 12 || day < 1 || day > 31) return 0;
    const long doy = daysFromCivil(year, static_cast<unsigned>(month),
                                   static_cast<unsigned>(day)) -
                     daysFromCivil(year, 1, 1) + 1;
    return doy >= 1 && doy <= 366 ? static_cast<int>(doy) : 0;
}

std::optional<std::pair<int, int> > ProductMeta::parseObsDate(const std::string &obsPath) {
    std::string f = obsPath;
    if (auto ps = f.find_last_of("\\/"); ps != std::string::npos) f = f.substr(ps + 1);
    if (f.empty()) return std::nullopt;
    {
        std::string l = f;
        for (auto &c: l) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (l.size() > 3 && l.compare(l.size() - 3, 3, ".gz") == 0) f.erase(f.size() - 3);
    }


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
            for (const size_t back: {static_cast<size_t>(4), static_cast<size_t>(3)}) {

                if (base.size() < back) continue;
                if (const std::string d3 = base.substr(base.size() - back, 3); std::all_of(d3.begin(), d3.end(),
                                                                                           [](const unsigned char c) { return std::isdigit(c); })) {
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
        if (const int doy = dateToDoy(y, mo, da); y >= 1980 && doy > 0) return std::make_pair(y, doy);
    }
    return std::nullopt;
}



const char *ProductMeta::mirrorRoot() {
    return "ftp://igs.gnsswhu.cn/pub/gps";
}

int ProductMeta::gpsWeek(const int year, const int doy) {
    if (doy < 1 || doy > 366) return 0;


    const long days = daysFromCivil(year, 1, 1) + doy - 1 - daysFromCivil(1980, 1, 6);
    if (days < 0) return 0;
    return static_cast<int>(days / 7);
}

bool ProductMeta::covers(const int _year, const int _doy) const {
    if (this->year < 1980 || this->doy < 1 || this->spanDays < 1) return false;
    const long start = daysFromCivil(this->year, 1, 1) + this->doy - 1;
    const long day = daysFromCivil(_year, 1, 1) + _doy - 1;
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
            types.count(ProductType::TRO);
    if (wantWeek) {

        const long prevOrd = daysFromCivil(year, 1, 1) + doy - 2;
        int py = 0;
        unsigned pm = 0, pd = 0;
        civilFromDays(prevOrd, py, pm, pd);
        const int prevDoy = static_cast<int>(prevOrd - daysFromCivil(py, 1, 1)) + 1;

        char buf[256];
        const std::pair<int, int> days2[2] = {{year, doy}, {py, prevDoy}};
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

    return out;
}
