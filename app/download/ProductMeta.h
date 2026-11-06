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

    static const char *VMFRoot();

    static int GPSWeek(int year, int doy);

    static const std::vector<std::string> &knownDatacenters();

    static std::vector<std::string> scanDirs(const std::set<ProductType> &types, int year, int doy);
};

namespace Download {
    /// 按日期检索的产品类型个数（与 ProductType 前 9 个枚举值一一对应）
    inline constexpr int kDateTypeCount = 9;

    inline constexpr const char *kTypeLabels[kDateTypeCount] = {
        "广播星历", "精密轨道", "精密钟差", "地球自转",
        "电离层", "对流层", "码偏差", "映射函数",
        "卫星姿态"
    };

    /// 产品类型的显示名；观测文件与未识别类型不在日期检索表里，单独给。
    inline const char *typeLabel(const ProductType t) {
        if (const int i = static_cast<int>(t); i >= 0 && i < kDateTypeCount) return kTypeLabels[i];
        if (t == ProductType::OBS) return "观测文件";
        return "未识别";
    }
}
