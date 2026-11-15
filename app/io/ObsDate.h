#pragma once

#include <optional>
#include <string>

#include "io/Observer.h"

using ObsDate = Observer::ObsDate;
using ObsDateFrom = Observer::DateFrom;

/// 从已有的观测文件猜日期：名字 → 路径 → 文件头 → 下载库目录四层回落，认 .gz。
/// 这是应用层的取巧，所以留在 app；src 侧只认文件头（`Observer::probeObsDate`）。
std::optional<ObsDate> parseObsDate(const std::string &obspath);

/// 默认取昨天：产品当天多半还没发全。
ObsDate defaultObsDate();
