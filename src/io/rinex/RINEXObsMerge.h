#pragma once

#include <string>
#include <vector>

namespace RINEX {
    struct ObsGroup {
        std::string key;
        std::vector<std::string> files;
    };

    std::vector<ObsGroup> groupObsFiles(const std::vector<std::string> &files);

    int mergeObsFiles(const std::vector<std::string> &files, const std::string &outPath, std::string &err);
}
