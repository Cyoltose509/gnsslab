#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "CliCommon.h"
#include "CliCsv.h"
#include "SolveDriver.h"   // 共享回填实现（fillFromObs/fillFromSPP/feedRtk）与 ObsSource

namespace Cli {

// 判断是否为 OEM7 日志（按扩展名）；其余一律按 RINEX 观测处理
bool isOem7Path(const std::string &p) {
    const auto dp = p.rfind('.');
    if (dp == std::string::npos) return false;
    const std::string ext = p.substr(dp + 1);
    return ext == "log" || ext == "LOG" || ext == "oem7" || ext == "OEM7";
}

// 扫描 RINEX obs 同目录下的伴生导航文件（与 SolverTask::ScanNavFiles 同规则）
std::vector<std::string> scanNavFiles(const std::string &obsPath) {
    std::vector<std::string> result;
    if (obsPath.size() < 4) return result;
    const std::string base = obsPath.substr(0, obsPath.size() - 1);
    for (const char *ext : {"N", "G", "C", "F", "E", "J", "I", "L"}) {
        std::string np = base + ext;
        std::ifstream t(np);
        if (t.good()) result.push_back(np);
    }
    return result;
}

std::string defaultCsvName(GnssTask::SolverMode m) {
    switch (m) {
        case GnssTask::SolverMode::Leo: return "gnss_leo.csv";
        case GnssTask::SolverMode::Rtk: return "gnss_rtk.csv";
        case GnssTask::SolverMode::Ppp: return "gnss_ppp.csv";
        default: return "gnss_spp.csv";
    }
}

void dumpCsv(const std::vector<GnssTask::EpochData> &epochs,
             GnssTask::SolverMode mode, const Vector3d &ref,
             bool hasRefOrbit, const SolveConfig &cfg) {
    std::string path = cfg.csvPath.empty() ? defaultCsvName(mode) : cfg.csvPath;
    std::ofstream out(path);
    if (!out) {
        std::cerr << "无法打开输出文件: " << path << "\n";
        return;
    }
    CliCsv::writeHeader(out, mode, hasRefOrbit);
    for (const auto &ep : epochs)
        CliCsv::writeRow(out, ep, mode, ref, hasRefOrbit);
    std::cout << "已写出 " << epochs.size() << " 历元 -> " << path << "\n";
}

// 解析失败/异常时给个 CSV 兜底，避免空文件
void dumpEmpty(GnssTask::SolverMode mode, const SolveConfig &cfg) {
    dumpCsv({}, mode, Vector3d(0, 0, 0), false, cfg);
}

} // namespace Cli
