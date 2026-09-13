#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "GuiLeoProcessor.h"
#include "GuiHelpers.h"   // fileRow / renderCutoffAndSystems / beginConfigWindow / renderCancelButton
#include "core/AppConfig.h"
#include "ui/Gui.h"
#include "imgui.h"
#include "SPP.h"          // LEO 继承自 PPP/SPP，getFromSPP 复用
#include "LEO.h"          // 运动学定轨（卫星测卫星）
#include "Sp3OrbitReader.h"  // 参考轨道读取（近似位置 + 轨迹/RTN）
#include "RinexObsReader.h"
#include "QualityControl.h"   // QC::makeQCObsEpoch / QualityReport
#include "Log.h"

#include <fstream>
#include <filesystem>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <set>
#include <string>
#include <vector>

namespace GuiLeoProcessor {
    using GuiFileProcessor::SppEpochData;
    using GuiFileProcessor::SppTask;

    static void ReaderThread(const std::shared_ptr<LeoTask> &task) {
        try {
            const auto &path = task->core->filePath;
            LOG_INFO << "LEO 读取文件: " << path;
            task->core->qcInput.clear();

            // LEO 广播星历回退由解算线程 leo.loadBrdc 在类内加载（与 PPP 一致），读取线程不再预载
            task->core->hasNav = !task->gnssBrdcPathBuf.empty();
            {
                std::lock_guard lk(task->core->queueMutex);
                task->core->navReady = true;
            }
            task->core->queueCv.notify_all();

            RinexObsReader obsReader;
            std::fstream obsFile(path.c_str(), std::ios::in);
            if (!obsFile) {
                task->core->hasError = true;
                task->core->errorMsg = "无法打开 RINEX";
                task->core->readDone = true;
                task->core->queueCv.notify_all();
                task->core->loading = false;
                return;
            }
            obsReader.pFileStream = &obsFile;
            int safety = 0, ok = 0;
            while (++safety < 100000) {
                if (task->stop) break;
                try {
                    ObsData o = obsReader.parseRinexObs();
                    task->core->qcInput.push_back(QC::makeQCObsEpoch(o, o.weekSecond.sow));
                    {
                        std::lock_guard lk(task->core->queueMutex);
                        task->core->obsQueue.push({std::move(o), nullptr});
                    }
                    task->core->queueCv.notify_one();
                    ok++;
                } catch (const EndOfFile &) { break; } catch (const std::exception &e) {
                    task->core->hasError = true;
                    task->core->errorMsg = "RINEX parse err: " + std::string(e.what());
                    task->core->readDone = true;
                    task->core->queueCv.notify_all();
                    task->core->loading = false;
                    return;
                }
            }
            if (ok == 0) {
                task->core->hasError = true;
                task->core->errorMsg = "RINEX 无历元";
            }
            task->core->totalEpochs = ok;
            task->core->readDone = true;
            task->core->queueCv.notify_all();
            if (!task->core->qcInput.empty())
                GuiFileProcessor::LaunchQC(task->core);
            else
                task->core->qcReady = true;
        } catch (const std::exception &e) {
            task->core->hasError = true;
            task->core->errorMsg = e.what();
        }
        task->core->loading = false;
    }

    static void SolverThread(const std::shared_ptr<LeoTask> &task) {
        try {
            {
                std::unique_lock lk(task->core->queueMutex);
                task->core->queueCv.wait(lk, [&] { return task->core->navReady.load() || task->stop.load(); });
            }
            if (task->stop) {
                task->core->solvingDone = true;
                return;
            }

            LEO leo;
            leo.setCutoffElevDeg(task->core->cutoffDeg);
            leo.enabledSystems = task->core->enabledSystems;

            bool loadOk = true;
            if (!task->gnssSp3PathBuf.empty() && !leo.loadSp3(task->gnssSp3PathBuf)) {
                LOG_ERROR << "LEO loadSp3 失败";
                loadOk = false;
            }
            if (!task->gnssClkPathBuf.empty() && !leo.loadClk(task->gnssClkPathBuf)) {
                LOG_ERROR << "LEO loadClk 失败";
                loadOk = false;
            }
            if (!task->osbPathBuf.empty() && !leo.loadOsb(task->osbPathBuf)) {
                LOG_ERROR << "LEO loadOsb 失败";
                loadOk = false;
            }
            if (!task->atxPathBuf.empty() && !leo.loadAtx(task->atxPathBuf)) {
                LOG_ERROR << "LEO loadAtx 失败";
                loadOk = false;
            }
            if (!task->gnssBrdcPathBuf.empty()) leo.loadBrdc(task->gnssBrdcPathBuf); // 回退，非致命

            Sp3OrbitReader refRdr;
            SatID refSat('L', task->refPrn); // 兜底
            bool hasRefOrbit = false; // 参考轨道为可选
            if (!task->refSp3PathBuf.empty()) {
                refRdr.read(task->refSp3PathBuf);
                const auto &rdata = refRdr.getData();
                for (const auto &[sat, _]: rdata)
                    if (sat.system == 'L') {
                        task->refPrn = sat.id;
                        break;
                    }
                if (!rdata.empty()) task->refPrn = rdata.begin()->first.id;
                refSat = SatID('L', task->refPrn);
                if (!leo.loadLeoReference(task->refSp3PathBuf, refSat)) {
                    LOG_ERROR << "LEO loadLeoReference 失败";
                    loadOk = false;
                } else hasRefOrbit = true;
            } else {
                LOG_INFO << "LEO: 未提供参考轨道 SP3，跳过 RTN 评估与参考轨迹（解算仍依赖精密星历/钟差）";
            }
            task->core->hasRefOrbit = hasRefOrbit; // 供导出 CSV 判断是否输出 REF/RTN 列

            if (!task->auxFiles.empty()) {
                for (const auto &f: task->auxFiles) leo.loadAuxFile(f); // 多 HDR：后者覆盖 mLeoAntOffset
            } else {
                LOG_INFO << "LEO: 未加载辅助文件(.HDR 杠杆臂)，CoG→ARP 偏移为 0";
            }

            if (!loadOk) {
                task->core->hasError = true;
                task->core->errorMsg = "产品加载失败（检查 SP3/CLK/OSB/ATX/参考轨道/辅助文件路径）";
                task->core->solvingDone = true;
                return;
            }

            bool firstEpoch = true;
            while (true) {
                GuiFileProcessor::ObsItem oi;
                {
                    std::unique_lock lk(task->core->queueMutex);
                    task->core->queueCv.wait(lk, [&] {
                        return !task->core->obsQueue.empty() || task->core->readDone.load() || task->stop.load();
                    });
                    if (task->core->obsQueue.empty() && (task->core->readDone.load() || task->stop.load())) break;
                    if (task->core->obsQueue.empty()) continue;
                    oi = std::move(task->core->obsQueue.front());
                    task->core->obsQueue.pop();
                }
                if (task->stop) break;

                if (firstEpoch) firstEpoch = false;

                SppEpochData item;
                bool ok = false;
                Vector3d leoPos = Vector3d::Zero();
                Vector3d refPos = Vector3d::Zero();
                std::vector<GuiCharts::SatVis> gnssVisLocal;
                std::map<SatID, Vector3d> gnssTrajLocal;
                bool haveRtn = false;
                double R = 0, T = 0, N = 0, d3 = 0, sod = 0;
                try {
                    item.getFromObs(oi.obs);
                    ok = leo.process(oi.obs);
                    if (ok) {
                        item.getFromSPP(leo);
                        leoPos = leo.result.xyz;
                        ok = leoPos.squaredNorm() > 1e12; // 合理 ECEF 位置(>1e6 m)
                        if (ok) {
                            for (size_t si = 0; si < item.satIds.size(); ++si) {
                                if (item.rejected[si]) continue;
                                const PVT &sp = item.satPVTs[si];
                                if (sp.p.squaredNorm() <= 1.0) continue;
                                gnssVisLocal.push_back(
                                    GuiCharts::SatVis{item.satIds[si], item.satIds[si].system, sp.p, true});
                                gnssTrajLocal[item.satIds[si]] = sp.p;
                            }
                            refPos = leoPos;
                            if (hasRefOrbit) {
                                const PVT ref = refRdr.getPVT(refSat, oi.obs.epoch);
                                refPos = ref.p.squaredNorm() > 1e12 ? ref.p : leoPos;
                                const Vector3d err = leoPos - refPos;
                                const Vector3d ur = refPos.normalized();
                                if (ref.v.squaredNorm() > 1.0) {
                                    const Vector3d ut = ref.v.normalized();
                                    const Vector3d uc = ur.cross(ut).normalized();
                                    R = err.dot(ur);
                                    T = err.dot(ut);
                                    N = err.dot(uc);
                                } else { R = err.dot(ur); }
                                d3 = err.norm();
                                sod = oi.obs.epoch.m_sod;
                                haveRtn = true;
                            }
                            item.refECEF = refPos;
                            item.rtnR = R;
                            item.rtnT = T;
                            item.rtnN = N;
                        }
                    }
                } catch (const std::exception &e) {
                    ok = false;
                    LOG_ERROR << "LEO 历元解算异常 (sod=" << oi.obs.epoch.m_sod << "): " << e.what();
                } catch (...) {
                    ok = false;
                    LOG_ERROR << "LEO 历元解算未知异常 (sod=" << oi.obs.epoch.m_sod << ")";
                }
                if (!ok) item.solved = false;

                {
                    std::lock_guard lk(task->core->mutex);
                    const int idx = static_cast<int>(task->core->epochs.size());
                    task->core->epochs.push_back(std::move(item));
                    if (ok) {
                        task->core->leoPos3d = leoPos;
                        task->core->leoTraj.push_back({idx, leoPos});
                        task->core->hasLeo = true;
                        task->core->gnssVis = std::move(gnssVisLocal);
                        for (auto &kv: gnssTrajLocal) {
                            task->core->gnssTraj[kv.first].push_back({idx, kv.second});
                        }
                        task->core->leoRef3d = refPos;
                        if (haveRtn) {
                            task->core->leoRefTraj.push_back({idx, refPos});
                            task->core->rtnTimes.push_back(sod);
                            task->core->rtn_r.push_back(R);
                            task->core->rtn_t.push_back(T);
                            task->core->rtn_n.push_back(N);
                            task->core->rtn_d3.push_back(d3);
                        }
                        if (!task->core->initializedRefECEF) {
                            task->core->refECEF = refPos;
                            task->core->initializedRefECEF = true;
                        }
                    }
                    try {
                        std::lock_guard plk(task->core->plotMutex);
                        task->core->plotData.insert(idx, task->core->epochs.back(), task->core->refECEF);
                    } catch (...) {
                    }
                    if (task->core->selectedEpoch == -1 || task->core->selectedEpoch == idx - 1)
                        task->core->selectedEpoch = idx;
                    task->core->solvingProgress = idx + 1;
                    task->core->solvedCount = static_cast<int>(task->core->epochs.size());
                }
            }
        } catch (const std::exception &e) {
            task->core->hasError = true;
            task->core->errorMsg = e.what();
        }
        task->core->solvingDone = true;
    }

    void SolveThread(const std::shared_ptr<LeoTask> &task) {
        std::thread thRead(ReaderThread, task);
        std::thread thSolve(SolverThread, task);
        thRead.join();
        thSolve.join();
        task->core->state = SppTask::State::Done;
        task->state = LeoTask::State::Done;
        task->done = true;
        task->loading = false;
        task->core->done = true;
        task->core->loading = false;
    }

    void RenderConfigPanel(const std::shared_ptr<LeoTask> &task) {
        GuiHelpers::beginConfigWindow("LEO 处理配置###leo_cfg_", task.get());

        GuiHelpers::fileRow("观测文件", &task->leoObsPathBuf, "leo_obs", GuiHelpers::fObs, 3, {});
        GuiHelpers::fileRow("广播星历 BRDC", &task->gnssBrdcPathBuf, "leo_gnss_brdc", GuiHelpers::fRnx, 2, {});
        GuiHelpers::fileRow("精密轨道 SP3", &task->gnssSp3PathBuf, "leo_gnss_sp3", GuiHelpers::fSp3, 2, {});
        GuiHelpers::fileRow("精密钟差 CLK", &task->gnssClkPathBuf, "leo_gnss_clk", GuiHelpers::fClk, 2, {});
        GuiHelpers::fileRow("天线文件 ATX", &task->atxPathBuf, "leo_atx", GuiHelpers::fAtx, 2, {});
        GuiHelpers::fileRow("码偏差 OSB/BIA", &task->osbPathBuf, "leo_osb", GuiHelpers::fOsb, 2, {});
        GuiHelpers::fileRow("参考轨道 SP3 (可选)", &task->refSp3PathBuf, "leo_ref_sp3", GuiHelpers::fSp3, 2, {});
        ImGui::Text("辅助文件 (.HDR 等):");
        if (!task->auxFiles.empty()) {
            for (size_t i = 0; i < task->auxFiles.size(); i++) {
                ImGui::Bullet();
                std::string shortName = task->auxFiles[i];
                if (const auto pos = shortName.find_last_of("\\/"); pos != std::string::npos)
                    shortName = shortName.substr(pos + 1);
                ImGui::Text("%s", shortName.c_str());
                ImGui::SameLine();
                if (ImGui::SmallButton(("删除##aux" + std::to_string(i)).c_str()))
                    task->auxFiles.erase(task->auxFiles.begin() + i);
            }
        }
        if (ImGui::Button("添加...")) {
            std::vector<std::wstring> wpaths;
            if (ShowOpenFilesDialog(wpaths, GuiHelpers::fHdr, 2)) {
                for (auto &wp: wpaths) {
                    std::string p = wideToAcp(wp);
                    if (std::find(task->auxFiles.begin(), task->auxFiles.end(), p) == task->auxFiles.end())
                        task->auxFiles.push_back(p);
                    AppConfig::instance().addRecent("leo_aux", p);
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("从记录添加")) ImGui::OpenPopup("##leo_aux_hist_pop");
        if (ImGui::BeginPopup("##leo_aux_hist_pop")) {
            const auto recent = AppConfig::instance().getRecent("leo_aux");
            if (recent.empty()) ImGui::TextDisabled("(暂无记录)");
            for (auto &r: recent) {
                if (ImGui::Selectable(r.c_str())) {
                    if (std::find(task->auxFiles.begin(), task->auxFiles.end(), r) == task->auxFiles.end())
                        task->auxFiles.push_back(r);
                    AppConfig::instance().addRecent("leo_aux", r);
                }
            }
            ImGui::EndPopup();
        }
        ImGui::Separator();
        GuiHelpers::renderCutoffAndSystems(&task->core->cutoffDeg, &task->core->enabledSystems);
        ImGui::Separator();
        if (ImGui::Button("开始解算", ImVec2(200, 40))) {
            if (task->leoObsPathBuf.empty()) {
                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "请先选择观测文件");
            } else {
                const auto &cfg = AppConfig::instance();
                cfg.set("leo_obs", task->leoObsPathBuf);
                cfg.set("leo_gnss_brdc", task->gnssBrdcPathBuf);
                cfg.set("leo_gnss_sp3", task->gnssSp3PathBuf);
                cfg.set("leo_gnss_clk", task->gnssClkPathBuf);
                cfg.set("leo_atx", task->atxPathBuf);
                cfg.set("leo_osb", task->osbPathBuf);
                cfg.set("leo_ref_sp3", task->refSp3PathBuf);
                cfg.set("leo_ref_prn", std::to_string(task->refPrn));

                task->core->filePath = task->leoObsPathBuf;
                task->core->fileName = GuiHelpers::baseName(task->leoObsPathBuf);
                task->fileName = task->core->fileName;
                task->core->obsPathBuf = task->leoObsPathBuf;
                task->core->isRinex = true;

                task->state = LeoTask::State::Running;
                task->core->state = SppTask::State::Running;
                task->loading = true;
                task->core->loading = true;
                task->worker = std::thread(SolveThread, task);
            }
        }
        ImGui::SameLine();
        GuiHelpers::renderCancelButton(task);

        ImGui::End();
    }

    void RenderTask(const std::shared_ptr<LeoTask> &task) {
        if (task->state == LeoTask::State::Config) {
            RenderConfigPanel(task);
            return;
        }
        task->core->loading.store(task->loading.load());
        task->core->done.store(task->done.load());
        task->core->hasError = task->hasError;
        task->core->errorMsg = task->errorMsg;
        task->core->processorLabel = "LEO"; // 导出文件名 / 标签用
        if (task->fileName.empty()) task->fileName = task->core->fileName;
        task->core->fileName = task->fileName;
        task->core->state = SppTask::State::Running;
        GuiFileProcessor::RenderTask(task->core, false);
    }
}
