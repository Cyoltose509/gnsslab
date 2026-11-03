#include "Core.h"

#include <fstream>
#include <string>

#include "Log.h"
#include "rinex/RINEXObsReader.h"
#include "novatel/LogReader.h"

namespace Solvers {
    bool waitNavReady(const std::shared_ptr<Core> &core) {
        std::unique_lock lk(core->queueMutex);
        core->queueCv.wait(lk, [&] { return core->navReady.load() || core->stop.load(); });
        return !core->stop.load();
    }

    bool nextObsQueue(const std::shared_ptr<Core> &core, Observer::Item &out) {
        std::unique_lock lk(core->queueMutex);
        for (;;) {
            core->queueCv.wait(lk, [&] {
                return !core->obsQueue.empty() || core->readDone.load() || core->stop.load();
            });
            if (!core->obsQueue.empty()) {
                out = std::move(core->obsQueue.front());
                core->obsQueue.pop();
                return !core->stop.load();
            }
            return false;
        }
    }

    void commitEpoch(const std::shared_ptr<Core> &core, Epoch &&item,
                     const std::function<void(int, Epoch &)> &onStored) {
        std::lock_guard lk(core->mutex);
        const int idx = static_cast<int>(core->epochs.size());
        core->epochs.push_back(std::move(item));
        if (onStored) onStored(idx, core->epochs.back());
        try {
            std::lock_guard plk(core->plotMutex);
            core->plotData.insert(idx, core->epochs.back(), core->refECEF);
        } catch (...) {
        }
        if (core->selectedEpoch == -1 || core->selectedEpoch == idx - 1) core->selectedEpoch = idx;
        core->solvingProgress = idx + 1;
        core->solvedCount = static_cast<int>(core->epochs.size());
    }

    int readRINEXObsToQueue(const std::shared_ptr<Core> &core, const std::atomic<bool> &stopFlag) {
        const std::string &path = core->filePath;
        {
            std::lock_guard lk(core->qcInputMutex);
            core->qcInput.clear();
        }

        {
            std::lock_guard lk(core->queueMutex);
            core->navReady = true;
        }
        core->queueCv.notify_all();

        std::fstream obsFile(path.c_str(), std::ios::in);
        if (!obsFile) {
            LOG_ERROR << "无法打开 RINEX: " << path;
            core->setError("无法打开 RINEX");
            core->readDone = true;
            core->queueCv.notify_all();
            return -1;
        }
        const auto obsReader = Observer::makeRINEXReader(obsFile);
        int safety = 0, ok = 0;
        while (++safety < 100000) {
            if (stopFlag) break;
            try {
                ObsData o = obsReader->parseRINEXObs();
                {
                    std::lock_guard lk(core->qcInputMutex);
                    core->qcInput.push_back(QC::makeQCObsEpoch(o, o.weekSecond.sow));
                }
                {
                    std::lock_guard lk(core->queueMutex);
                    core->obsQueue.push({std::move(o), nullptr});
                }
                core->queueCv.notify_one();
                ok++;
            } catch (const EndOfFile &) { break; } catch (const std::exception &e) {
                LOG_ERROR << "RINEX parse err (" << path << "): " << e.what();
                core->setError("RINEX parse err: " + std::string(e.what()));
                core->readDone = true;
                core->queueCv.notify_all();
                return -1;
            }
        }
        if (ok == 0) {
            LOG_ERROR << "RINEX 无历元: " << path << "（文件是否被取消/解压失败/选错文件？）";
            core->setError("RINEX 无历元");
        }
        core->totalEpochs = ok;
        core->readDone = true;
        core->queueCv.notify_all();
        return ok;
    }

    int readNovAtelObsToQueue(const std::shared_ptr<Core> &core, const std::atomic<bool> &stopFlag) {
        const std::string &path = core->filePath;
        {
            std::lock_guard lk(core->qcInputMutex);
            core->qcInput.clear();
        }

        const auto reader = Observer::makeNovAtelReader(path); // 二进制 / ASC 由内容决定
        if (!reader || !reader->open(path)) {
            LOG_ERROR << "无法打开 NovAtel 日志: " << path;
            core->setError("无法打开 NovAtel 日志");
            core->readDone = true;
            core->queueCv.notify_all();
            return -1;
        }
        {
            std::lock_guard lk(core->queueMutex);
            core->navReady = true;
        }
        core->queueCv.notify_all();

        ObsData obs;
        int ok = 0;
        while (reader->getNextEpoch(obs)) {
            if (stopFlag) break;
            auto eph = reader->ephSnapshot();
            {
                std::lock_guard lk(core->qcInputMutex);
                core->qcInput.push_back(QC::makeQCObsEpoch(obs, obs.weekSecond.sow));
            }
            {
                std::lock_guard lk(core->queueMutex);
                core->obsQueue.push({std::move(obs), std::move(eph)});
            }
            core->queueCv.notify_one();
            ok++;
        }
        if (ok == 0) {
            LOG_ERROR << "NovAtel 日志无历元: " << path;
            core->setError("NovAtel 日志无历元");
        }
        core->totalEpochs = ok;
        core->hasNav = ok > 0;
        core->readDone = true;
        core->queueCv.notify_all();
        return ok;
    }

    int readObsToQueue(const std::shared_ptr<Core> &core, const std::atomic<bool> &stopFlag) {
        if (core->isRINEX) return readRINEXObsToQueue(core, stopFlag);
        return readNovAtelObsToQueue(core, stopFlag);
    }
}
