#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include <atomic>
#include <chrono>

#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "Const.h"

#include "GnssStruct.h"
#include "SolverTask.h"
#include "SPP.h"
#include "ObsFormat.h"      // classifyObsPath：RINEX/OEM7 三值判别（文件模式分发唯一入口）
#include "OEM7SocketReader.h"
#include "Log.h"
#include "core/SolveConfig.h"
#include "CliCommon.h"
#include "SolveDriver.h"
#include "core/IniFile.h"

namespace {
    // ---------------------------------------------------------------------------
    // SPP（文件模式：RINEX 或 OEM7）—— 单站解算由 SolveDriver::solveAll 统一驱动
    // ---------------------------------------------------------------------------
    void runSppFile(const SolveConfig &cfg) {
        std::vector<GnssTask::EpochData> epochs;
        SPP spp;
        spp.setCutoffElevDeg(cfg.cutoffDeg);
        spp.setEnabledSystems(cfg.systems);
        spp.setIsbProcNoise(cfg.isbProcNoise, cfg.isbInitVar); // ISB：非参考系统钟差当常数/随机游走估(对齐老师)

        // 广播星历：优先用 nav，空则扫伴生文件
        std::vector<std::string> navs;
        if (!cfg.navPath.empty()) navs.push_back(cfg.navPath);
        else navs = GnssTask::ScanNavFiles(cfg.obsPath);
        for (const auto &np: navs) {
            try { spp.loadBrdc(np); } catch (const std::exception &e) {
                std::cerr << "SPP 读取广播星历失败 " << np << ": " << e.what() << "\n";
            }
        }

        Eigen::Vector3d ref(0, 0, 0);

        const auto fmt = GnssTask::classifyObsPath(cfg.obsPath);
        if (fmt == GnssTask::ObsFormat::Oem7) {
            SolveDriver::Oem7FileSource src;
            if (!src.open(cfg.obsPath)) {
                std::cerr << "SPP 无法打开 OEM7: " << cfg.obsPath << "\n";
                Cli::dumpEmpty(GnssTask::SolverMode::Spp, cfg);
                return;
            }
            epochs = SolveDriver::solveAll(src, spp);
        } else if (fmt == GnssTask::ObsFormat::Rinex) {
            SolveDriver::RinexFileSource src;
            if (!src.open(cfg.obsPath)) {
                std::cerr << "SPP 无法打开 RINEX: " << cfg.obsPath << "\n";
                Cli::dumpEmpty(GnssTask::SolverMode::Spp, cfg);
                return;
            }
            epochs = SolveDriver::solveAll(src, spp);
        } else {
            std::cerr << "SPP 无法识别观测文件格式（支持 RINEX .??O/.rnx/.obs 与 OEM7 .log/.oem7）: "
                      << cfg.obsPath << "\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Spp, cfg);
            return;
        }

        if (cfg.hasRef) ref = Eigen::Vector3d(cfg.refX, cfg.refY, cfg.refZ);
        else SolveDriver::firstSolvedRef(epochs, ref);
        Cli::dumpCsv(epochs, GnssTask::SolverMode::Spp, ref, false, cfg);
    }

    // ---------------------------------------------------------------------------
    // SPP（实时流模式：OEM7 socket）—— 连接/重连由本函数管理，逐历元核心复用 SolveDriver 回填
    // ---------------------------------------------------------------------------
    void runSppRealtime(const SolveConfig &cfg) {
        std::vector<GnssTask::EpochData> epochs;
        SPP spp;
        spp.setCutoffElevDeg(cfg.cutoffDeg);
        spp.setEnabledSystems(cfg.systems);
        spp.setIsbProcNoise(cfg.isbProcNoise, cfg.isbInitVar); // ISB：非参考系统钟差当常数/随机游走估(对齐老师)

        std::atomic stop{false};
        std::thread waiter;
        if (cfg.realtimeSec <= 0 && cfg.maxEpochs <= 0) {
            // 无计时/计数限制时，等待用户回车停止
            waiter = std::thread([&] {
                std::cin.get();
                stop = true;
            });
        } else if (cfg.realtimeSec > 0) {
            std::thread([&] {
                std::this_thread::sleep_for(std::chrono::seconds(cfg.realtimeSec));
                stop = true;
            }).detach();
        }

        bool refInit = false;
        Eigen::Vector3d ref(0, 0, 0);
        bool loggedFail = false;

        while (!stop) {
            try {
                OEM7SocketReader reader;
                if (!reader.connect(cfg.ip, static_cast<unsigned short>(cfg.port))) {
                    if (!loggedFail) {
                        std::cerr << "[实时] 连接失败 " << cfg.ip << ":" << cfg.port << "，每 2s 重试\n";
                        loggedFail = true;
                    }
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    continue;
                }
                loggedFail = false;
                std::cerr << "[实时] 已连接 " << cfg.ip << ":" << cfg.port << "\n";
                reader.setReceiveTimeout(200);

                ObsData obs;
                while (!stop) {
                    bool got = false;
                    try { got = reader.getNextEpoch(obs); } catch (...) { got = false; }
                    if (!got) {
                        if (!reader.isConnected()) break;
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        continue;
                    }
                    auto eph = reader.ephSnapshot();
                    if (eph) spp.ephTable = *eph;
                    try { spp.preprocess(obs); } catch (...) {
                    }
                    GnssTask::EpochData data;
                    SolveDriver::fillFromObs(data, obs);
                    bool ok = false;
                    try { ok = spp.processEpoch(obs); } catch (...) { ok = false; }
                    if (ok) SolveDriver::fillFromSPP(data, spp);
                    else data.solved = false;
                    if (data.solved && !refInit) {
                        ref = data.sppResult.xyz;
                        refInit = true;
                    }
                    epochs.push_back(data);
                    if (cfg.maxEpochs > 0 && static_cast<int>(epochs.size()) >= cfg.maxEpochs) stop = true;
                }
                reader.close();
            } catch (const std::exception &e) {
                std::cerr << "[实时] 异常: " << e.what() << "\n";
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }

        if (waiter.joinable()) waiter.join();
        if (cfg.hasRef) ref = Eigen::Vector3d(cfg.refX, cfg.refY, cfg.refZ);
        Cli::dumpCsv(epochs, GnssTask::SolverMode::Spp, ref, false, cfg);
    }
} // namespace

int main(int argc, char **argv) {
    std::string ini = argc > 1 ? argv[1] : "spp.ini";
    AppConfig::IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    const SolveConfig cfg = loadConfig(f);
    Log::init("cli_gnsslab.log");
    std::cout << "[SPP] 模式=" << cfg.source << " obs=" << cfg.obsPath
            << " nav=" << (cfg.navPath.empty() ? "(伴生自动扫描)" : cfg.navPath) << "\n";
    if (cfg.source == "realtime") runSppRealtime(cfg);
    else runSppFile(cfg);
    return 0;
}
