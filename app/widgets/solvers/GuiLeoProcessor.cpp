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
#include "CoordConvert.h"  // computeRTN：RTN 误差分量（原为 GUI 内手算）
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
#include "GuiSolverPanel.h"

namespace GuiLeoProcessor {
    using GnssTask::EpochData;
    using GnssTask::SolveTask;
    using GnssTask::SolverMode;

    static void ReaderThread(const std::shared_ptr<LeoTask> &task) {
        try {
            const auto &path = task->core->filePath;
            LOG_INFO << "LEO 读取文件: " << path;
            // LEO 广播星历回退由解算线程 leo.loadBrdc 在类内加载（与 PPP 一致），读取线程不再预载
            task->core->hasNav = !task->gnssBrdcPathBuf.empty();
            // 读取主体（清 QC 输入 → 发 navReady 信号 → 逐历元入队 → 置 totalEpochs/readDone/notify_all）
            // 原与 SPP/PPP 逐行雷同，已抽到 GnssTask::ReadRinexObsToQueue。
            // 返回值：-1 = 打开或解析失败（已置 hasError/errorMsg/readDone 并 notify_all）；
            //        0  = 无历元（已置 hasError/"RINEX 无历元"，仍继续走下面 QC 收尾分支）。
            const int ok = GnssTask::ReadRinexObsToQueue(task->core, task->core->stop);
            if (ok < 0) {
                task->core->loading = false;
                return;
            }
            bool hasQcInput = false;
            {
                std::lock_guard<std::mutex> lk(task->core->qcInputMutex);
                hasQcInput = !task->core->qcInput.empty();
            }
            if (hasQcInput)
                GuiSolverPanel::LaunchQC(task->core);
            else
                task->core->qcReady = true;
        } catch (const std::exception &e) {
            task->core->setError(e.what());
        }
        task->core->loading = false;
    }

    static void SolverThread(const std::shared_ptr<LeoTask> &task) {
        // 参考轨道读取器与判定结果在循环外构造，供 init 与逐历元 body 共用。
        Sp3OrbitReader refRdr;
        SatID refSat('L', task->refPrn); // 兜底
        bool hasRefOrbit = false;         // 参考轨道为可选，加载失败不致命
        bool loadOk = true;

        const auto init = [&](LEO &leo) {
            // 超快/最终 SP3 自带钟差列时 CLK 可缺省（解算器自行判定，此处只负责加载与日志）
            if (!task->gnssSp3PathBuf.empty()) {
                const bool ok = leo.loadSp3(task->gnssSp3PathBuf);
                LOG_INFO << "LEO 读取精密轨道 SP3: " << task->gnssSp3PathBuf << (ok ? "" : "[失败]");
                if (!ok) loadOk = false;
            }
            if (!task->gnssClkPathBuf.empty()) {
                const bool ok = leo.loadClk(task->gnssClkPathBuf);
                LOG_INFO << "LEO 读取精密钟差 CLK: " << task->gnssClkPathBuf << (ok ? "" : "[失败]");
                if (!ok) loadOk = false;
            }
            if (!task->osbPathBuf.empty()) {
                const bool ok = leo.loadOsb(task->osbPathBuf);
                LOG_INFO << "LEO 读取码偏差 OSB: " << task->osbPathBuf << (ok ? "" : "[失败]");
                if (!ok) loadOk = false;
            }
            if (!task->atxPathBuf.empty()) {
                const bool ok = leo.loadAtx(task->atxPathBuf);
                LOG_INFO << "LEO 读取天线文件 ATX: " << task->atxPathBuf << (ok ? "" : "[失败]");
                if (!ok) loadOk = false;
            }
            if (!task->gnssBrdcPathBuf.empty()) {
                const bool ok = leo.loadBrdc(task->gnssBrdcPathBuf);
                LOG_INFO << "LEO 读取广播星历 BRDC: " << task->gnssBrdcPathBuf << (ok ? "" : "[失败]");
            }

            if (!task->refSp3PathBuf.empty()) {
                LOG_INFO << "LEO 读取参考轨道 SP3: " << task->refSp3PathBuf;
                refRdr.read(task->refSp3PathBuf);
                const auto &rdata = refRdr.getData();
                bool refFound = false; // 是否已在 SP3 中找到 L 系统卫星
                for (const auto &[sat, _]: rdata)
                    if (sat.system == 'L') {
                        task->refPrn = sat.id;
                        refFound = true;
                        break;
                    }
                // 仅当没找到 L 系统卫星时才用首条记录兜底；否则会无条件覆盖掉上面（或用户 UI）选中的 PRN
                if (!refFound && !rdata.empty()) task->refPrn = rdata.begin()->first.id;
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
                task->core->setError("产品加载失败（检查 SP3/CLK/OSB/ATX/参考轨道/辅助文件路径）");
                return false;
            }
            return true;
        };

        GnssTask::RunSingleStationSolve<LEO>(task->core, init,
            [&](LEO &leo, GnssTask::ObsItem &oi, GnssTask::EpochData &data, int idx) {
                if (!leo.processEpoch(oi.obs)) return false;
                data.getFromSPP(leo);
                const Vector3d leoPos = leo.result.xyz;
                if (leoPos.squaredNorm() <= 1e12) return false;   // 非合理 ECEF 位置(<=1e6 m)

                // 下方写入的 leoTraj/gnssTraj/rtn*/gnssVis 等都是**共享状态**：渲染侧(GuiSolverPanel)
                // 会在 core->mutex 下整份拷贝它们，而这里此前一把锁都没拿。push_back 触发重分配时，
                // 渲染侧并发拷贝就会读到已释放内存，或读到"size 与 data 指针不一致的半更新向量" →
                // Debug 下迭代器断言 / abort(0x80000003)。LEO 是唯一按历元 push 大向量的解算器，
                // 故只有它崩。锁顺序与 CommitEpoch 一致(先 core->mutex，后 plotMutex)，不会死锁。
                std::lock_guard lk(task->core->mutex);
                std::vector<SatVis> gnssVis;
                for (size_t si = 0; si < data.satIds.size(); ++si) {
                    if (data.rejected[si]) continue;
                    const PVT &sp = data.satPVTs[si];
                    if (sp.p.squaredNorm() <= 1.0) continue;
                    gnssVis.push_back(
                        SatVis{data.satIds[si], data.satIds[si].system, sp.p, true});
                    task->core->gnssTraj[data.satIds[si]].push_back({idx, sp.p});
                }
                task->core->gnssVis = std::move(gnssVis);   // 只留当前历元的可见星
                task->core->leoPos3d = leoPos;
                task->core->leoTraj.push_back({idx, leoPos});
                task->core->hasLeo = true;

                Vector3d refPos = leoPos;   // 无参考轨道时自比，RTN 全零
                if (hasRefOrbit) {
                    const PVT ref = refRdr.getPVT(refSat, oi.obs.epoch);
                    refPos = ref.p.squaredNorm() > 1e12 ? ref.p : leoPos;
                    // RTN 分量（径向/沿迹/法向）由 CoordConvert::computeRTN 统一提供
                    const RtnComponents rtn = computeRTN(leoPos, refPos, ref.v);
                    data.refECEF = refPos;
                    data.rtnR = rtn.R;
                    data.rtnT = rtn.T;
                    data.rtnN = rtn.N;
                    task->core->leoRef3d = refPos;
                    task->core->leoRefTraj.push_back({idx, refPos});
                    task->core->rtnTimes.push_back(oi.obs.epoch.m_sod);
                    task->core->rtnR.push_back(rtn.R);
                    task->core->rtnT.push_back(rtn.T);
                    task->core->rtnN.push_back(rtn.N);
                    task->core->rtnD3.push_back(rtn.d3);
                }
                if (!task->core->initializedRefECEF) {
                    task->core->refECEF = refPos;
                    task->core->initializedRefECEF = true;
                }
                return true;
            });
    }

    void SolveThread(const std::shared_ptr<LeoTask> &task) {
        std::thread thRead(ReaderThread, task);
        std::thread thSolve(SolverThread, task);
        thRead.join();
        thSolve.join();
        task->state = LeoTask::State::Done;
        task->done = true;
        task->loading = false;
        GnssTask::SyncTaskStatus(task, task->core);
    }

    void LeoTask::renderConfigPanel() {
        GuiHelpers::beginConfigWindow("LEO 处理配置###leo_cfg_", *this);

        GuiHelpers::fileRow("观测文件", &this->leoObsPathBuf, "leo_obs", GuiHelpers::fObs, 3, {});
        GuiHelpers::fileRow("广播星历 BRDC", &this->gnssBrdcPathBuf, "leo_gnss_brdc", GuiHelpers::fRnx, 2, {});
        GuiHelpers::fileRow("精密轨道 SP3", &this->gnssSp3PathBuf, "leo_gnss_sp3", GuiHelpers::fSp3, 2, {});
        GuiHelpers::fileRow("精密钟差 CLK", &this->gnssClkPathBuf, "leo_gnss_clk", GuiHelpers::fClk, 2, {});
        GuiHelpers::fileRow("天线文件 ATX", &this->atxPathBuf, "leo_atx", GuiHelpers::fAtx, 2, {});
        GuiHelpers::fileRow("码偏差 OSB/BIA", &this->osbPathBuf, "leo_osb", GuiHelpers::fOsb, 2, {});
        GuiHelpers::fileRow("参考轨道 SP3 (可选)", &this->refSp3PathBuf, "leo_ref_sp3", GuiHelpers::fSp3, 2, {});
        ImGui::Text("辅助文件 (.HDR 等):");
        if (!this->auxFiles.empty()) {
            for (size_t i = 0; i < this->auxFiles.size(); i++) {
                ImGui::Bullet();
                std::string shortName = this->auxFiles[i];
                if (const auto pos = shortName.find_last_of("\\/"); pos != std::string::npos)
                    shortName = shortName.substr(pos + 1);
                ImGui::Text("%s", shortName.c_str());
                ImGui::SameLine();
                if (ImGui::SmallButton(("删除##aux" + std::to_string(i)).c_str()))
                    this->auxFiles.erase(this->auxFiles.begin() + i);
            }
        }
        if (ImGui::Button("添加...")) {
            std::vector<std::wstring> wpaths;
            if (ShowOpenFilesDialog(wpaths, GuiHelpers::fHdr, 2)) {
                for (auto &wp: wpaths) {
                    std::string p = wideToAcp(wp);
                    if (std::find(this->auxFiles.begin(), this->auxFiles.end(), p) == this->auxFiles.end())
                        this->auxFiles.push_back(p);
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
                    if (std::find(this->auxFiles.begin(), this->auxFiles.end(), r) == this->auxFiles.end())
                        this->auxFiles.push_back(r);
                    AppConfig::instance().addRecent("leo_aux", r);
                }
            }
            ImGui::EndPopup();
        }
        ImGui::Separator();
        GuiHelpers::renderCutoffAndSystems(&this->core->cutoffDeg, &this->core->enabledSystems);
        ImGui::Separator();
        if (ImGui::Button("开始解算", ImVec2(200, 40))) {
            if (this->leoObsPathBuf.empty()) {
                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "请先选择观测文件");
            } else {
                const auto &cfg = AppConfig::instance();
                cfg.set("leo_obs", this->leoObsPathBuf);
                cfg.set("leo_gnss_brdc", this->gnssBrdcPathBuf);
                cfg.set("leo_gnss_sp3", this->gnssSp3PathBuf);
                cfg.set("leo_gnss_clk", this->gnssClkPathBuf);
                cfg.set("leo_atx", this->atxPathBuf);
                cfg.set("leo_osb", this->osbPathBuf);
                cfg.set("leo_ref_sp3", this->refSp3PathBuf);
                cfg.set("leo_ref_prn", std::to_string(this->refPrn));

                // 同 PPP：上一轮被取消/关闭过会残留 stop=true，不复位则本轮 0 历元且无报错。
                this->stop = false;
                this->core->stop = false;

                this->core->filePath = this->leoObsPathBuf;
                this->core->fileName = GuiHelpers::baseName(this->leoObsPathBuf);
                this->core->obsPathBuf = this->leoObsPathBuf;
                this->core->isRinex = true;

                this->state = LeoTask::State::Running;
                this->loading = true;
                this->core->loading = true;
                this->worker = std::thread(SolveThread, std::static_pointer_cast<LeoTask>(shared_from_this()));
            }
        }
        ImGui::SameLine();
        GuiHelpers::renderCancelButton(this);

        ImGui::End();
    }

}
