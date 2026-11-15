#pragma once

#include <functional>
#include <string>

#include "core/geom/EphemerisTable.h"
#include "Observer.h"

struct Products;

namespace Solvers::RTK {
    /// 逐历元配对双站观测；星历取自 prod.eph（广播星历已由任务装好，本函数不再读盘）。
    int pairFiles(const std::string &basePath, const std::string &roverPath,
                  const std::string &navPath, double sampleInterval,
                  Products &prod,
                  const std::function<bool()> &shouldStop,
                  const std::function<void(const ObsData &, const ObsData &,
                                           const std::shared_ptr<EphemerisTable> &)> &onPair,
                  std::string &err);
}
