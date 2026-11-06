#include "Task.h"

#include <memory>

#include "PPPKinematic.h"
#include "SPP.h"

namespace Solvers::PPPKinematic {
    std::unique_ptr<::SPP> Task::makeSolver() {
        return std::make_unique<::PPPKinematic>();
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->applyConfig(cfg);
        return task;
    }
}
