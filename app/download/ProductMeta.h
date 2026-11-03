#pragma once

#include <string>
#include <optional>
#include <utility>
#include <set>
#include <vector>


enum class ProductType {
    NAV,
    ORB,
    CLK,
    ERP,
    ION,
    TRO,
    OSB,
    VMF,
    ATT,
    MET,
    OBS,
    Unknown
};


struct ProductMeta {
    ProductType type = ProductType::Unknown;
    std::string datacenter;
    int year = 0;
    int doy = 0;
    int spanDays = 1;
    std::string interval;
    std::string ext;
    std::string filename;

    std::string slot;


    static std::optional<ProductMeta> parse(const std::string &filename);


    static std::string typeDir(ProductType t);


    static ProductType typeFromIniKey(const std::string &iniKey);


    static std::string doyToDateStr(int year, int doy);

    static std::optional<std::pair<int, int> > parseObsDate(const std::string &obsPath);

    static std::pair<int, int> defaultObsDate();

    [[nodiscard]] bool covers(int _year, int _doy) const;

    static const char *mirrorRoot();

    static const char *vmfRoot();

    static int gpsWeek(int year, int doy);

    static const std::vector<std::string> &knownDatacenters();

    static std::vector<std::string> scanDirs(const std::set<ProductType> &types, int year, int doy);
};
