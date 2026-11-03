#pragma once

#include <string>
#include <memory>
#include <algorithm>

#include "GnssStruct.h"
#include "solvers/common/Core.h"
#include "view/GuiHelpers.h"
#include "view/GuiParamPage.h"
#include "core/PathMemory.h"
#include "core/Products.h"
#include "product/SP3OrbitReader.h"

namespace Solvers::LEO {
    struct Task : Solvers::Task {

        std::string leoObsPathBuf;
        std::string refSp3PathBuf;

        std::vector<std::string> auxFiles;
        int refPrn = 49;

        std::shared_ptr<Solvers::Core> core;

        std::shared_ptr<Solvers::Core> solveResult() override { return core; }
        void renderConfigPanel() override;

        bool initSolver(void *solver) override;
        bool afterSolveEpoch(void *solver, Solvers::Epoch *data,
                             Observer::Item *item, int index) override;

        SP3OrbitReader refRdr;
        SatID refSat;
        bool hasRefOrbit = false;

    protected:
        ::SPP *makeSolver() override;

    public:
        Task() {
            core = std::make_shared<Solvers::Core>();
            const auto &cfg = AppConfig::PathMemory::instance();
            leoObsPathBuf = cfg.get("leo_obs");
            refSp3PathBuf = cfg.get("leo_ref_sp3");
            for (auto &r : cfg.getRecent("leo_aux")) {
                if (std::find(auxFiles.begin(), auxFiles.end(), r) == auxFiles.end())
                    auxFiles.push_back(r);
            }
            if (const std::string prnStr = cfg.get("leo_ref_prn"); !prnStr.empty()) {
                try { refPrn = std::stoi(prnStr); } catch (...) {}
            }
            Helpers::loadCommonCfg("leo", Solvers::Mode::LEO, &core->opts,
                                   &core->cutoffDeg, &core->sampleInterval, &core->enabledSystems);
            AppConfig::Products::instance().load();
            refSat = SatID('L', refPrn);
        }

    };

}
