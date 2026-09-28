#pragma once

#include "GnssStruct.h"
#include <string>
#include <vector>

/// 地球自转参数 ERP（IGS ERP v2 文本格式）读取器。
class ErpReader {
public:
    struct Record {
        double mjd = 0.0;    // 修约儒略日（文件原值，UTC 基准）
        double xp = 0.0;     // 极移 x（rad）
        double yp = 0.0;     // 极移 y（rad）
        double ut1Utc = 0.0; // UT1-UTC（s）
        double lod = 0.0;    // 日长变化 LOD（s）
    };

    bool read(const std::string &path);

    [[nodiscard]] bool empty() const { return data.empty(); }

    [[nodiscard]] int size() const { return static_cast<int>(data.size()); }

    [[nodiscard]] bool getErp(const CommonTime &t, double &xp, double &yp, double &ut1Utc) const;

private:
    std::vector<Record> data;
};
