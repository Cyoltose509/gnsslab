#include "GuiInsPanel.h"

#include "GuiInsCommon.h"
#include "solvers/common/Core.h"
#include "core/PathMemory.h"

#include <string>

namespace Tools::Ins {
    std::shared_ptr<Solvers::Task> MakeSixPosCalibTask() {
        auto t = std::make_shared<SixPosCalibTask>();
        const auto &pm = AppConfig::PathMemory::instance();
        if (const std::string files = pm.get("ins_cal_files"); !files.empty()) {
            std::size_t pos = 0;
            while (pos < files.size()) {
                std::size_t nxt = files.find('|', pos);
                if (nxt == std::string::npos) nxt = files.size();
                if (std::string p = files.substr(pos, nxt - pos); !p.empty()) t->filePaths.push_back(p);
                pos = nxt + 1;
            }
        }

        loadInsParams(t->par);
        return t;
    }

    std::shared_ptr<Solvers::Task> MakeStaticAlignTask() {
        auto t = std::make_shared<StaticAlignTask>();
        t->alignPath = AppConfig::PathMemory::instance().get("ins_align");
        loadInsParams(t->par);
        return t;
    }

}
