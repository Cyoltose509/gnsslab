#include "GuiInsPanel.h"

#include "GuiInsCommon.h"
#include "GuiHelpers.h"
#include "SolverTask.h"
#include "core/PathMemory.h"
#include "Const.h"

#include <sstream>
#include <string>

namespace Tools::InsPanel {
    std::shared_ptr<GnssTask::SolverTaskBase> MakeSixPosCalibTask() {
        auto t = std::make_shared<SixPosCalibTask>();
        const auto &pm = AppConfig::PathMemory::instance();
        if (const std::string files = pm.get("ins_cal_files"); !files.empty()) {
            size_t pos = 0;
            while (pos < files.size()) {
                size_t nxt = files.find('|', pos);
                if (nxt == std::string::npos) nxt = files.size();
                if (std::string p = files.substr(pos, nxt - pos); !p.empty()) t->filePaths.push_back(p);
                pos = nxt + 1;
            }
        }

        loadInsParams(t->par);
        return t;
    }

    std::shared_ptr<GnssTask::SolverTaskBase> MakeStaticAlignTask() {
        auto t = std::make_shared<StaticAlignTask>();
        t->alignPath = AppConfig::PathMemory::instance().get("ins_align");
        loadInsParams(t->par);
        return t;
    }

    bool RenderInsResult(const std::shared_ptr<GnssTask::SolverTaskBase> &job) {
        if (auto *c = dynamic_cast<SixPosCalibTask *>(job.get())) {
            c->renderResult();
            return true;
        }
        if (auto *a = dynamic_cast<StaticAlignTask *>(job.get())) {
            a->renderResult();
            return true;
        }
        return false;
    }
}
