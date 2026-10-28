#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "CliCommon.h"
#include "SolverTask.h"
#include "SolveDriver.h"

namespace Cli {
    std::string defaultCsvName(const GnssTask::SolverMode m) {
        switch (m) {
            case GnssTask::SolverMode::Leo: return "gnss_leo.csv";
            case GnssTask::SolverMode::Rtk: return "gnss_rtk.csv";
            case GnssTask::SolverMode::Ppp: return "gnss_ppp.csv";
            default: return "gnss_spp.csv";
        }
    }

    void dumpCsv(const std::vector<GnssTask::EpochData> &epochs,
                 const GnssTask::SolverMode mode, const Eigen::Vector3d &ref,
                 const bool hasRefOrbit, const SolveConfig &cfg) {
        const std::string path = cfg.csvPath.empty() ? defaultCsvName(mode) : cfg.csvPath;
        std::ofstream out(path);
        if (!out) {
            std::cerr << "无法打开输出文件: " << path << "\n";
            return;
        }
        GnssTask::exportCsv(out, epochs, mode, ref, hasRefOrbit);
        std::cout << "已写出 " << epochs.size() << " 历元 -> " << path << "\n";
    }

    void dumpEmpty(const GnssTask::SolverMode mode, const SolveConfig &cfg) {
        dumpCsv({}, mode, Eigen::Vector3d(0, 0, 0), false, cfg);
    }
}
