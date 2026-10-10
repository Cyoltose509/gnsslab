#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "GuiLeoProcessor.h"
#include "GuiHelpers.h"   // fileRow / renderCutoffAndSystems / beginConfigBody / renderCancelButton
#include "core/PathMemory.h"
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

    // ---- LeoTask 的解算钩子（骨架见 RunSingleStationSolve）----

    bool LeoTask::initSolver(void *solverPtr) {
        LEO &leo = *static_cast<LEO *>(solverPtr);
        // 参考轨道读取器与判定结果放在 LeoTask 成员里：initSolver 与 afterSolveEpoch 都要用。
        hasRefOrbit = false;         // 参考轨道为可选，加载失败不致命
        bool loadOk = true;

        // 超快/最终 SP3 自带钟差列时 CLK 可缺省（解算器自行判定，此处只负责加载与日志）
        if (!gnssSp3PathBuf.empty()) {
            const bool ok = leo.loadSp3(gnssSp3PathBuf);
            LOG_INFO << "LEO 读取精密轨道 SP3: " << gnssSp3PathBuf << (ok ? "" : "[失败]");
            if (!ok) loadOk = false;
        }
        if (!gnssClkPathBuf.empty()) {
            // 同 PPP：只加载参与解算的星座，跳过其余系统的整行解析
            const bool ok = leo.loadClk(gnssClkPathBuf, core->enabledSystems);
            LOG_INFO << "LEO 读取精密钟差 CLK: " << gnssClkPathBuf << (ok ? "" : "[失败]");
            if (!ok) loadOk = false;
        }
        if (!osbPathBuf.empty()) {
            const bool ok = leo.loadOsb(osbPathBuf);
            LOG_INFO << "LEO 读取码偏差 OSB: " << osbPathBuf << (ok ? "" : "[失败]");
            if (!ok) loadOk = false;
        }
        if (!atxPathBuf.empty()) {
            const bool ok = leo.loadAtx(atxPathBuf);
            LOG_INFO << "LEO 读取天线文件 ATX: " << atxPathBuf << (ok ? "" : "[失败]");
            if (!ok) loadOk = false;
        }
        if (!gnssBrdcPathBuf.empty()) {
            const bool ok = leo.loadBrdc(gnssBrdcPathBuf);
            LOG_INFO << "LEO 读取广播星历 BRDC: " << gnssBrdcPathBuf << (ok ? "" : "[失败]");
            // 广播星历对 LEO **不是可选的**：LEO 构造里 mFirstEpochSPP=true，首历元定位靠 SPP，
            // 而 SPP 的卫星位置只能来自广播星历（精密星历是 5min 间隔、且不具备电文参数）。
            // 缺失时现象极隐蔽：每个历元都解不出、日志无任何 ERROR，用户只会看到"全程无解"。
            if (!ok) loadOk = false;
        } else {
            LOG_ERROR << "LEO 未填广播星历 BRDC：首历元 SPP 定位需要它，缺则全历元无解";
            loadOk = false;
        }

        if (!refSp3PathBuf.empty()) {
            LOG_INFO << "LEO 读取参考轨道 SP3: " << refSp3PathBuf;
            refRdr.read(refSp3PathBuf);
            const auto &rdata = refRdr.getData();
            bool refFound = false; // 是否已在 SP3 中找到 L 系统卫星
            for (const auto &[sat, _]: rdata)
            if (sat.system == 'L') {
                refPrn = sat.id;
                refFound = true;
                break;
            }
            // 仅当没找到 L 系统卫星时才用首条记录兜底；否则会无条件覆盖掉上面（或用户 UI）选中的 PRN
            if (!refFound && !rdata.empty()) refPrn = rdata.begin()->first.id;
            refSat = SatID('L', refPrn);
            if (!leo.loadLeoReference(refSp3PathBuf, refSat)) {
                LOG_ERROR << "LEO loadLeoReference 失败";
                loadOk = false;
            } else hasRefOrbit = true;
        } else {
            LOG_INFO << "LEO: 未提供参考轨道 SP3，跳过 RTN 评估与参考轨迹（解算仍依赖精密星历/钟差）";
        }
        core->hasRefOrbit = hasRefOrbit; // 供导出 CSV 判断是否输出 REF/RTN 列

        if (!auxFiles.empty()) {
            for (const auto &f: auxFiles) leo.loadAuxFile(f); // 多 HDR：后者覆盖 mLeoAntOffset
        } else {
            LOG_INFO << "LEO: 未加载辅助文件(.HDR 杠杆臂)，CoG→ARP 偏移为 0";
        }
        if (!loadOk) {
            core->setError("产品加载失败（检查 SP3/CLK/OSB/ATX/参考轨道/辅助文件路径）");
            return false;
        }
            return true;
    }

    bool LeoTask::afterSolveEpoch(void *, GnssTask::EpochData *data,
                                  GnssTask::ObsItem *item, int idx) {
        if (!data->solved) return false;
        const Vector3d leoPos = data->sppResult.xyz;
        if (leoPos.squaredNorm() <= 1e12) return false;   // 非合理 ECEF 位置(<=1e6 m)

        // 下方写入的 leoTraj/gnssTraj/rtn*/gnssVis 等都是**共享状态**：渲染侧(GuiSolverPanel)
        // 会在 core->mutex 下整份拷贝它们，而这里此前一把锁都没拿。push_back 触发重分配时，
        // 渲染侧并发拷贝就会读到已释放内存，或读到"size 与 data 指针不一致的半更新向量" →
        // Debug 下迭代器断言 / abort(0x80000003)。LEO 是唯一按历元 push 大向量的解算器，
        // 故只有它崩。锁顺序与 CommitEpoch 一致(先 core->mutex，后 plotMutex)，不会死锁。
        std::lock_guard lk(core->mutex);
        std::vector<SatVis> gnssVis;
        for (size_t si = 0; si < data->satIds.size(); ++si) {
                if (data->rejected[si]) continue;
                const PVT &sp = data->satPVTs[si];
                if (sp.p.squaredNorm() <= 1.0) continue;
                gnssVis.push_back(
                SatVis{data->satIds[si], data->satIds[si].system, sp.p, true});
                core->gnssTraj[data->satIds[si]].push_back({idx, sp.p});
        }
        core->gnssVis = std::move(gnssVis);   // 只留当前历元的可见星
        core->leoPos3d = leoPos;
        core->leoTraj.push_back({idx, leoPos});
        core->hasLeo = true;

        Vector3d refPos = leoPos;   // 无参考轨道时自比，RTN 全零
        if (hasRefOrbit) {
                const PVT ref = refRdr.getPVT(refSat, item->obs.epoch);
                refPos = ref.p.squaredNorm() > 1e12 ? ref.p : leoPos;
                // RTN 分量（径向/沿迹/法向）由 CoordConvert::computeRTN 统一提供
                const RtnComponents rtn = computeRTN(leoPos, refPos, ref.v);
                data->refECEF = refPos;
                data->rtnR = rtn.R;
                data->rtnT = rtn.T;
                data->rtnN = rtn.N;
                core->leoRef3d = refPos;
                core->leoRefTraj.push_back({idx, refPos});
                core->rtnTimes.push_back(item->obs.epoch.m_sod);
                core->rtnR.push_back(rtn.R);
                core->rtnT.push_back(rtn.T);
                core->rtnN.push_back(rtn.N);
                core->rtnD3.push_back(rtn.d3);
        }
        if (!core->initializedRefECEF) {
                core->refECEF = refPos;
                core->initializedRefECEF = true;
        }
        return true;
    }

    static void SolverThread(const std::shared_ptr<LeoTask> &task) {
        LEO leo;
        GnssTask::RunSingleStationSolve<LEO>(task->core, leo, task);
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
        GuiHelpers::beginConfigBody("LEO 处理配置###leo_cfg_", *this);

        GuiHelpers::fileRow("观测文件", &this->leoObsPathBuf, "leo_obs", GuiHelpers::fObs, 3, {}, "leo_obs");
        GuiHelpers::fileRow("广播星历 BRDC", &this->gnssBrdcPathBuf, "leo_gnss_brdc", GuiHelpers::fRnx, 2, {}, "leo_obs");
        GuiHelpers::fileRow("精密轨道 SP3", &this->gnssSp3PathBuf, "leo_gnss_sp3", GuiHelpers::fSp3, 2, {}, "leo_obs");
        GuiHelpers::fileRow("精密钟差 CLK", &this->gnssClkPathBuf, "leo_gnss_clk", GuiHelpers::fClk, 2, {}, "leo_obs");
        GuiHelpers::fileRow("天线文件 ATX", &this->atxPathBuf, "leo_atx", GuiHelpers::fAtx, 2, {}, "leo_obs");
        GuiHelpers::fileRow("码偏差 OSB/BIA", &this->osbPathBuf, "leo_osb", GuiHelpers::fOsb, 2, {}, "leo_obs");
        GuiHelpers::fileRow("参考轨道 SP3 (可选)", &this->refSp3PathBuf, "leo_ref_sp3", GuiHelpers::fSp3, 2, {}, "leo_obs");
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
                    GnssCore::PathMemory::instance().addRecent("leo_aux", p);
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("从记录添加")) ImGui::OpenPopup("##leo_aux_hist_pop");
        if (ImGui::BeginPopup("##leo_aux_hist_pop")) {
            const auto recent = GnssCore::PathMemory::instance().getRecent("leo_aux");
            if (recent.empty()) ImGui::TextDisabled("(暂无记录)");
            for (auto &r: recent) {
                if (ImGui::Selectable(r.c_str())) {
                    if (std::find(this->auxFiles.begin(), this->auxFiles.end(), r) == this->auxFiles.end())
                        this->auxFiles.push_back(r);
                    GnssCore::PathMemory::instance().addRecent("leo_aux", r);
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
                const auto &cfg = GnssCore::PathMemory::instance();
                cfg.set("leo_obs", this->leoObsPathBuf);
                cfg.set("leo_gnss_brdc", this->gnssBrdcPathBuf);
                cfg.set("leo_gnss_sp3", this->gnssSp3PathBuf);
                cfg.set("leo_gnss_clk", this->gnssClkPathBuf);
                cfg.set("leo_atx", this->atxPathBuf);
                cfg.set("leo_osb", this->osbPathBuf);
                cfg.set("leo_ref_sp3", this->refSp3PathBuf);
                cfg.set("leo_ref_prn", std::to_string(this->refPrn));

                {
                    const std::string refInfo =
                        this->refSp3PathBuf.empty() ? std::string("无") : this->refSp3PathBuf;
                    LOG_INFO << "[LEO] 开始定轨: 观测=" << this->leoObsPathBuf
                             << ", 参考轨道=" << refInfo;
                }

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

        GuiHelpers::endConfigBody();
    }

}
