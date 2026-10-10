#pragma once

#include <string>
#include <optional>
#include <utility>
#include <set>
#include <vector>

// 产品类型
enum class ProductType {
    NAV,
    ORB,
    CLK,
    ERP,
    ION,
    TRO,
    OSB,
    ATX,
    Unknown
};

// IGS 长文件名解析 + GPS 年积日 ↔ 日历日期桥接。
// 仅依赖文件名（Reader 都从内容读时间，无人解析文件名，这里收口成唯一真源）。
struct ProductMeta {
    ProductType type = ProductType::Unknown;
    std::string  datacenter;   // 文件名开头连续字母（COD / BRDC ...）
    int         year = 0;
    int         doy  = 0;      // 起始年积日（用户说的"GPS 天"）
    int         spanDays = 1;  // 覆盖天数：日期段后第一个 "NN D"（01D/02D/03D）；ULT 是 2D
    std::string interval;      // 01D / 05M ...（采样/时段段）
    std::string ext;           // SP3 / CLK / rnx ...
    std::string filename;

    // IGS 长文件名 → 元数据；非 IGS 命名或解析失败返回 nullopt
    static std::optional<ProductMeta> parse(const std::string &filename);

    // 产品类型 → downloads/ 子目录名
    static std::string typeDir(ProductType t);

    // 由 fileRow 的 iniKey 后缀推导产品类型（观测文件行返回 Unknown，不出"获取"按钮）
    static ProductType typeFromIniKey(const std::string &iniKey);

    // doy → 日历 "YYYY-MM-DD"（仅展示用）
    static std::string doyToDateStr(int year, int doy);
    // doy → 日历 (year, month, day)；越界返回 false
    static bool doyToYmd(int year, int doy, int &oy, int &om, int &od);
    // 日历 → doy
    static int dateToDoy(int year, int month, int day);

    // 从 RINEX 观测文件名（或路径里的 YYYYMMDD 日期目录）推断 (year, doy)
    static std::optional<std::pair<int, int>> parseObsDate(const std::string &obsPath);

    // 本产品覆盖区间 [起始日, 起始日 + spanDays - 1] 是否包含 (year, doy)
    [[nodiscard]] bool covers(int _year, int _doy) const;

    // ---- 远程镜像----
    static const char *mirrorRoot();

    // (年, 年积日) → GPS 周（products/<week> 目录名）
    static int gpsWeek(int year, int doy);

    // 实测出现过的 IGS 分析中心码（数据中心标签候选）
    static const std::vector<std::string> &knownDatacenters();

    // 按 (类型集合, 日期) 展开出需要列目录的远程 URL（去重、按需有序）。
    // 时段产品（02D/03D）可能起于上一周而覆盖本日 ⇒ 上周与本周都扫；IONEX 目录本身按日分。
    static std::vector<std::string> scanDirs(const std::set<ProductType> &types, int year, int doy);
};
