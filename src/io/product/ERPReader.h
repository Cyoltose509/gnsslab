#pragma once

#include "TimeStruct.h"
#include <string>
#include <vector>


class ERPReader {
public:
    struct Record {
        double mjd = 0.0;
        double xp = 0.0;
        double yp = 0.0;
        double ut1Utc = 0.0;
        double lod = 0.0;
    };

    bool read(const std::string &path);

    /// append=true：把 path 里的 ERP 并入已有 data（按 MJD 合并、重排），
    /// 用于跨日弧加载相邻多日 ERP。append=false 与 read(path) 等价。
    bool read(const std::string &path, bool append);

    [[nodiscard]] bool empty() const { return data.empty(); }

    [[nodiscard]] int size() const { return static_cast<int>(data.size()); }

    [[nodiscard]] bool getERP(const CommonTime &t, double &xp, double &yp, double &ut1Utc) const;

private:
    std::vector<Record> data;
};
