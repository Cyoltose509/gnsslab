#pragma once

#include <functional>
#include <string>

#include "EphemerisTable.h"
#include "Observer.h"

namespace Solvers::RTK {
    int pairFiles(const std::string &basePath, const std::string &roverPath,
                  const std::string &navPath, double sampleInterval,
                  EphemerisTable &navTable,
                  const std::function<bool()> &shouldStop,
                  const std::function<void(const ObsData &, const ObsData &,
                                           const std::shared_ptr<EphemerisTable> &,
                                           EphemerisTable *)> &onPair,
                  std::string &err);
}
