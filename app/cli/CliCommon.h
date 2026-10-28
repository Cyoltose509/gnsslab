#pragma once
#include <string>
#include <vector>

#include "SolverTask.h"
#include "SolverOptions.h"
#include "core/IniFile.h"

SolverOptions loadConfig(const AppConfig::IniFile &ini);

namespace Cli {
    std::string defaultCsvName(GnssTask::SolverMode m);

    void dumpCsv(const std::vector<GnssTask::EpochData> &epochs,
                 GnssTask::SolverMode mode, const Eigen::Vector3d &ref,
                 bool hasRefOrbit, const SolverOptions &cfg);

    void dumpEmpty(GnssTask::SolverMode mode, const SolverOptions &cfg);
}
