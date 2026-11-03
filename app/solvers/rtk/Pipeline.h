#pragma once

#include <functional>
#include <string>

#include "EphemerisTable.h"
#include "Observer.h"

namespace Solvers::RTK {
    /// 双站文件配对管线：格式识别 → RINEX 模式加载广播星历 → 双源对齐 → DualPair 逐配对回调。
    /// onPair 对每个配对历元回调（未经抽稀），eph 为该对已选星历快照（RINEX 模式为空，
    /// 调用方回退到 navTable，即传入的同一对象）。返回配对数，失败返回 -1 并给出 err。
    int pairFiles(const std::string &basePath, const std::string &roverPath,
                  const std::string &navPath, double sampleInterval,
                  EphemerisTable &navTable,
                  const std::function<bool()> &shouldStop,
                  const std::function<void(const ObsData &, const ObsData &,
                                           const std::shared_ptr<EphemerisTable> &,
                                           EphemerisTable *)> &onPair,
                  std::string &err);
}
