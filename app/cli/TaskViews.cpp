#include "solvers/leo/Task.h"
#include "solvers/ppp_kinematic/Task.h"
#include "solvers/ppp_static/Task.h"
#include "solvers/rtk/Task.h"
#include "solvers/spp/Task.h"

/// CLI 无界面，补齐不含 Panel.cpp 的链接单元所需 vtable 槽。
namespace Solvers {
    void Task::renderResult() {}
}

namespace Solvers::SPP {
    void Task::renderConfigPanel() {}
    void Task::loadFromMemory() {}
}

namespace Solvers::PPPStatic {
    void Task::renderConfigPanel() {}
    void Task::loadFromMemory() {}
}

namespace Solvers::PPPKinematic {
    void Task::renderConfigPanel() {}
    void Task::loadFromMemory() {}
}

namespace Solvers::LEO {
    void Task::renderConfigPanel() {}
    void Task::loadFromMemory() {}
}

namespace Solvers::RTK {
    void Task::renderConfigPanel() {}
}
