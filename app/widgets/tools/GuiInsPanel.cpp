#include "GuiInsPanel.h"

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
        t->g = pm.getNum("ins_g", SETTING_G, 1.0, 12.0);
        t->turnAngleDeg = pm.getNum("ins_turn", 360.0, 1.0, 3600.0);
        return t;
    }

    std::shared_ptr<GnssTask::SolverTaskBase> MakeStaticAlignTask() {
        auto t = std::make_shared<StaticAlignTask>();
        t->alignPath = AppConfig::PathMemory::instance().get("ins_align");
        t->latDeg = AppConfig::PathMemory::instance().getNum("ins_lat", SETTING_LAT_DEG, -90.0, 90.0);
        t->g = AppConfig::PathMemory::instance().getNum("ins_g", SETTING_G, 1.0, 12.0);
        t->win = AppConfig::PathMemory::instance().getInt("ins_win", 400, 2, 20000);
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
