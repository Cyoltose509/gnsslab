#pragma once
// 无 GUI CLI 的共享辅助（被各个 *_cli.cpp 复用）。
// 设计约定：本文件只放"被多个 cli 共用"的辅助；每个 solver 的 run* 入口
// 各自写在对应的 cli.cpp 里（如 runPpp 在 ppp_cli.cpp），保持归属清晰、
// 避免把 runRtk 之类也编进 spp_cli.exe。
#include <string>
#include <vector>

#include "SolverTask.h"   // GnssTask::EpochData / SolverMode
#include "GnssStruct.h"   // ObsData / PVT / SatID / Vector3d / Result
#include "core/SolveConfig.h"    // SolveConfig

namespace Cli {

// 判断是否为 OEM7 日志（按扩展名）；其余一律按 RINEX 观测处理
bool isOem7Path(const std::string &p);

// 扫描 RINEX obs 同目录下的伴生导航文件（与 SolverTask::ScanNavFiles 同规则）
std::vector<std::string> scanNavFiles(const std::string &obsPath);

std::string defaultCsvName(GnssTask::SolverMode m);

// 解出 CSV（按 cfg.csvPath 或默认名）
void dumpCsv(const std::vector<GnssTask::EpochData> &epochs,
             GnssTask::SolverMode mode, const Vector3d &ref,
             bool hasRefOrbit, const SolveConfig &cfg);

// 解析失败/异常时给个 CSV 兜底，避免空文件
void dumpEmpty(GnssTask::SolverMode mode, const SolveConfig &cfg);

} // namespace Cli
