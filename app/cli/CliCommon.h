#pragma once
#include <string>
#include <vector>

#include "SolverTask.h"
#include "core/SolveConfig.h"

namespace Cli {
    std::string defaultCsvName(GnssTask::SolverMode m);

    void dumpCsv(const std::vector<GnssTask::EpochData> &epochs,
                 GnssTask::SolverMode mode, const Eigen::Vector3d &ref,
                 bool hasRefOrbit, const SolveConfig &cfg);

    void dumpEmpty(GnssTask::SolverMode mode, const SolveConfig &cfg);
}
