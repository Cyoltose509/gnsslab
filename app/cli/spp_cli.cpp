
#include <atomic>
#include <chrono>

#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "Const.h"

#include "GnssStruct.h"
#include "solvers/common/Core.h"
#include "SPP.h"
#include "Observer.h"
#include "novatel/SocketReader.h"
#include "Log.h"
#include "SolverOptions.h"
#include "CliCommon.h"
#include "solvers/common/Driver.h"
#include "io/ObsSource.h"
#include "solvers/spp/Realtime.h"
#include "io/ProductLoad.h"
#include "core/IniFile.h"

namespace {
    void runSppFile(const SolverOptions &cfg) {
        std::vector<Solvers::Epoch> epochs;
        SPP spp;
        spp.applyOptions(cfg);
        ProductLoad::loadSPPGrids(spp, cfg.vmf, cfg.gpt3);
        ProductLoad::warnTropoFallback("SPP", cfg.spp.tropoMap, spp.tropoGridReady());

        std::vector<std::string> navs;
        if (!cfg.navPath.empty()) navs.push_back(cfg.navPath);
        else navs = ObsSource::scanNavFiles(cfg.obsPath);
        for (const auto &np: navs) {
            try { spp.loadBRDC(np); } catch (const std::exception &e) {
                std::cerr << "SPP 读取广播星历失败 " << np << ": " << e.what() << "\n";
            }
        }

        Eigen::Vector3d ref(0, 0, 0);

        Observer::RINEXFile rsrc;
        Observer::NovAtelFile osrc;
        Observer::Source *src = Cli::openObsSource(cfg.obsPath, "SPP", rsrc, osrc);
        if (!src) {
            Cli::dumpEmpty(Solvers::Mode::SPP, cfg);
            return;
        }
        epochs = Solvers::Driver::solveAll(*src, spp, cfg.sampleInterval);

        if (cfg.hasRef) ref = Eigen::Vector3d(cfg.refX, cfg.refY, cfg.refZ);
        else Solvers::Driver::firstSolvedRef(epochs, ref);
        Cli::dumpCSV(epochs, Solvers::Mode::SPP, ref, false, cfg);
    }

    void runSppRealtime(const SolverOptions &cfg) {
        std::vector<Solvers::Epoch> epochs;
        SPP spp;
        spp.applyOptions(cfg);
        ProductLoad::loadSPPGrids(spp, cfg.vmf, cfg.gpt3);
        ProductLoad::warnTropoFallback("SPP", cfg.spp.tropoMap, spp.tropoGridReady());

        std::atomic stop{false};
        std::thread waiter;
        if (cfg.realtimeSec <= 0 && cfg.maxEpochs <= 0) {
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
                NovAtel::SocketReader reader;
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

                const EpochDecimator dec(cfg.sampleInterval);
                Solvers::SPP::pump(reader, spp, dec,
                                             [&] { return stop.load(); },
                                             [&](ObsData &obs, const EphemerisTable &, const bool accepted) {
                                                 if (!accepted) return;
                                                 Solvers::Epoch data;
                                                 data.getFromObs(obs);
                                                 bool ok = false;
                                                 try { ok = spp.processEpoch(obs); } catch (...) { ok = false; }
                                                 if (ok) data.getFromSPP(spp);
                                                 else data.solved = false;
                                                 if (data.solved && !refInit) {
                                                     ref = data.sppResult.xyz;
                                                     refInit = true;
                                                 }
                                                 epochs.push_back(data);
                                                 if (cfg.maxEpochs > 0 && static_cast<int>(epochs.size()) >= cfg.maxEpochs)
                                                     stop = true;
                                             });
                reader.close();
            } catch (const std::exception &e) {
                std::cerr << "[实时] 异常: " << e.what() << "\n";
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }

        if (waiter.joinable()) waiter.join();
        if (cfg.hasRef) ref = Eigen::Vector3d(cfg.refX, cfg.refY, cfg.refZ);
        Cli::dumpCSV(epochs, Solvers::Mode::SPP, ref, false, cfg);
    }
}

int main(const int argc, char **argv) {
    const std::string ini = argc > 1 ? argv[1] : "spp.ini";
    IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    const SolverOptions cfg = loadConfig(f, Solvers::Mode::SPP);
    Log::init("cli_gnsslab.log");
    std::cout << "[SPP] 模式=" << cfg.source << " obs=" << cfg.obsPath
            << " nav=" << (cfg.navPath.empty() ? "(伴生自动扫描)" : cfg.navPath) << "\n";
    if (cfg.source == "realtime") runSppRealtime(cfg);
    else runSppFile(cfg);
    return 0;
}
