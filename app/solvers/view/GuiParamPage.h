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
}
