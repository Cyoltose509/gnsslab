#pragma once

#include <memory>
#include <string>
#include <vector>

#include "GnssStruct.h"
#include "product/SP3OrbitReader.h"
#include "solvers/common/Core.h"

namespace Solvers::LEO {
    /// LEO 定轨：配置与结果在 core，界面在 Panel.cpp。
    struct Task : Solvers::Task {
        std::string leoObsPathBuf;
        std::string refSp3PathBuf;

        std::vector<std::string> auxFiles;
        int refPrn = 49;

        std::shared_ptr<Solvers::Core> core;

        SP3OrbitReader refRdr;
        SatID refSat;
        bool hasRefOrbit = false;

        std::shared_ptr<Solvers::Core> solveResult() override { return core; }

        void applyConfig(const SolverOptions &cfg);

        void loadFromMemory();

        void renderConfigPanel();

        bool initSolver(void *solver) override;

        bool afterSolveEpoch(void *solver, Solvers::Epoch *data,
                             Observer::Item *item, int index) override;

        void exportCSV(std::ostream &out) const override;

        Task() {
            core = std::make_shared<Solvers::Core>();
            core->mode = Mode::LEO;
            refSat = SatID('L', refPrn);
        }

    protected:
        ::SPP *makeSolver() override;
    };

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg);
}
