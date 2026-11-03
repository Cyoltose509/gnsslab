#pragma once
#include <string>
#include <vector>

#include "solvers/common/Core.h"
#include "SolverOptions.h"
#include "core/IniFile.h"

SolverOptions loadConfig(const IniFile &ini, Solvers::Mode mode);

namespace Cli {
    std::string defaultCSVName(Solvers::Mode m);

    /// 按路径认出 NovAtel / RINEX 并打开，返回可用观测源；失败时已向 stderr 打印原因并返回 nullptr。
    /// 四个 CLI 的「开观测文件」样板（空路径 / NovAtel / RINEX / 认不出）本来就是同一份东西。
    /// tag 只进错误信息前缀（SPP/PPP/LEO）。两个源对象的生命周期由调用方持有。
    Observer::Source *openObsSource(const std::string &path, const char *tag,
                                    Observer::RINEXFile &rinex,
                                    Observer::NovAtelFile &oem);

    void dumpCSV(const std::vector<Solvers::Epoch> &epochs,
                 Solvers::Mode mode, const Eigen::Vector3d &ref,
                 bool hasRefOrbit, const SolverOptions &cfg);

    void dumpEmpty(Solvers::Mode mode, const SolverOptions &cfg);
}
