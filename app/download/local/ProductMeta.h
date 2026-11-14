#pragma once

#include <iterator>
#include <string>
#include <optional>

#include "Observer.h"


enum class ProductType {
    NAV, ORB, CLK, ERP, ION, TRO, OSB, VMF, ATT,
    OBS, LEO_OBS, LEO_SP3, SNX,
    Unknown
};

inline constexpr int kStoredTypeCount = static_cast<int>(ProductType::Unknown);
inline constexpr int kDateTypeCount = static_cast<int>(ProductType::OBS);

inline constexpr const char *kTypeLabels[] = {
    "广播星历", "精密轨道", "精密钟差", "地球自转", "电离层",
    "对流层", "码偏差", "映射函数", "卫星姿态",
    "观测文件", "低轨观测", "低轨轨道", "基站坐标",
};
static_assert(std::size(kTypeLabels) == kStoredTypeCount, "显示名表与 ProductType 不同步");

inline int dateTypeIndex(const ProductType t) {
    const int i = static_cast<int>(t);
    return i < kDateTypeCount ? i : -1;
}

inline const char *typeLabel(const ProductType t) {
    const int i = static_cast<int>(t);
    return i < kStoredTypeCount ? kTypeLabels[i] : "未识别";
}


using ObsDate = Observer::ObsDate;
using ObsDateFrom = Observer::DateFrom;


struct ProductMeta {
    ProductType type = ProductType::Unknown;
    std::string datacenter;
    int year = 0;
    int doy = 0;
    int spanDays = 1;
    bool stamped = false;
    std::string interval;
    std::string ext;
    std::string filename;

    std::string slot;


    static std::optional<ProductMeta> parse(const std::string &filename);

    static std::optional<ProductMeta> parseInDir(const std::string &filename, ProductType dirType,
                                                 int dateYear, int dateDoy);


    static std::string typeDir(ProductType t);

    static ProductType typeFromDir(const std::string &dir);

    static ProductType typeFromIniKey(const std::string &iniKey);

    static std::string doyToDateStr(int year, int doy);

    static std::optional<ObsDate> parseObsDate(const std::string &obsPath);

    static ObsDate defaultObsDate();
    [[nodiscard]] bool covers(int _year, int _doy) const;

    static int GPSWeek(int year, int doy);

    static std::string productLineOf(const std::string &filename);
};
