#pragma once

#include "GnssStruct.h"
#include "TimeConvert.h"
#include <map>
#include <string>
#include <vector>

/// 对流层产品 TRO（SINEX TRO，如 CODE `COD..._01H_TRO.TRO`）读取器。
/// 每测站逐时给出天顶总延迟与水平梯度：
///   *STATION__ _____EPOCH____ TROTOT STDDEV  TGNTOT STDDEV  TGETOT STDDEV
/// 文件内单位 mm（TROPO PARAMETER UNITS 1e+03），读入时统一转为 m。
///
/// **本产品是可选的**：未加载（empty()==true 或该测站无数据）时 getZtd() 返回 false，
/// 调用方应走原有的"ZWD 拉向 0"先验分支，行为与接入本产品之前完全一致。
///
/// 模型一致性提示：CODE 该产品的解算配置为 WET_VMF3 映射 + FES2014b 海洋潮负荷 +
/// Chen-Herring 梯度映射，而本工程默认用 NMF、无 OTL、Bar-Sever 梯度。
/// 故把它的 ZTD 当约束时会残留 mm 级模型差，约束 σ 不宜照搬其 1.6 mm 的形式误差。
class TroReader {
public:
    struct Record {
        CommonTime t;
        double mjd = 0.0;    // 由 t 换算的修约儒略日，内插用（避开 CommonTime 时系统差异）
        double ztd = 0.0;    // 天顶总延迟 TROTOT (m)
        double ztdStd = 0.0; // TROTOT 形式误差 (m)
    };

    bool read(const std::string &path);

    [[nodiscard]] bool empty() const { return data.empty(); }

    [[nodiscard]] int stationCount() const { return static_cast<int>(data.size()); }

    [[nodiscard]] bool hasStation(const std::string &station) const;

    /// 取该测站时刻 t 的 ZTD(线性内插)与形式误差。返回 false 表示无该站/无数据，调用方按"无 TRO"处理。
    /// 测站名先精确匹配，再退化为 4 字符短名匹配（如 HKWS00HKG ↔ HKWS）。
    [[nodiscard]] bool getZtd(const std::string &station, const CommonTime &t,
                              double &ztd, double &sigma) const;

private:
    [[nodiscard]] const std::vector<Record> *findStation(const std::string &station) const;

    std::map<std::string, std::vector<Record> > data;
};
