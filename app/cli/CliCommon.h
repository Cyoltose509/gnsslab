#pragma once
#include <string>
#include <vector>

#include "solvers/common/Task.h"
#include "SolverOptions.h"
#include "core/IniFile.h"

SolverOptions loadConfig(const IniFile &ini, Solvers::Mode mode);

/// ini 的 mode 键 → 解算模式（spp / ppp_static / ppp_kin / rtk / leo）。
Solvers::Mode modeFromKey(const std::string &s);

namespace Cli {
    std::string defaultCSVName(Solvers::Mode m);

    /// 把任务结果写成 CSV（cfg 指定路径或按模式取默认名），并打印历元数。
    void dumpTask(Solvers::Task &task, const SolverOptions &cfg);

    /// 统一入口：cfg.source 决定文件/实时，跑完写 CSV；返回进程退出码。
    int runTask(Solvers::Task &task, const SolverOptions &cfg);
}
