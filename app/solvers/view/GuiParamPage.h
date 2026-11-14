#pragma once

#include <set>
#include <string>

#include "core/PathMemory.h"
#include "SolverOptions.h"
#include "solvers/common/Core.h"

namespace Solvers::Helpers {
    void renderAdvancedParams(SolverOptions &opts, Mode mode);

    void saveSolverParams(Mode mode, const SolverOptions &opts);

    void loadSolverParams(Mode mode, SolverOptions &opts);

    /// INS 文件页用的初始位置/速度两行输入；参数页里同名条目只做存档、不显示。
    void renderInsInitialState(SolverOptions &opts);
}
