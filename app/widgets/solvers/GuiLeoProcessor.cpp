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
#include "CoordConvert.h"  // XYZToRTN：RTN 误差分量（原为 GUI 内手算）
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
#include "ObsFormat.h"      // classifyObsPath：RINEX/OEM7 三值判别（Unknown 显式报错）

namespace Solvers::LeoProcessor {
    using GnssTask::EpochData;
    using GnssTask::SolveTask;
    using GnssTask::SolverMode;

    static void ReaderThread(const std::shared_ptr<LeoTask> &task) {
        try {
            const auto &path = task->core->filePath;
            LOG_INFO << "LEO 读取文件: " << path;
            // RINEX / OEM7 分派已下沉 GnssTask::ReadObsToQueue（SPP/PPP 共用同一份）：
            // 返回值 -1 = 打开或解析失败；0 = 无历元；>0 = 入队历元数。
            // 卫星位置以 SP3 优先、BRDC 仅作回退，故 RINEX 模式 hasNav 恒为 true
            // （不依赖 BRDC 是否填）；OEM7 内含星历，hasNav 由 ReadOem7ObsToQueue 置位。
            task->core->hasNav = true;
            const int ok = GnssTask::ReadObsToQueue(task->core, task->core->stop);
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
                Solvers::SolverPanel::LaunchQC(task->core);
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
        // 广播星历 BRDC 对 LEO 是「可选回退」，不是必需。
        // LEO 继承 PPPStatic/PPPKinematic，卫星位置由 computeSatPos 优先取 SP3（精密轨道），
        // 仅在 SP3 不覆盖某星时才回退广播星历；重锚定 reanchorSPP / 主滤波同样用 SP3 优先的 satPVTRecTime。
        // 历史上 FALSE 断言"LEO 必须广播星历（首历元 SPP 需要 mFirstEpochSPP）"——其实该成员已从
        // PPPStatic 删除（从未被任何解算逻辑读取，是死标志），且 LEO 重锚定走的是 PPP 的 satPVTRecTime
        // 而非 SPP::computeSatPos。这里与 PPP 对齐：BRDC 空则跳过，不强制。
        if (!gnssBrdcPathBuf.empty()) {
            const bool ok = leo.loadBrdc(gnssBrdcPathBuf);
            LOG_INFO << "LEO 读取广播星历 BRDC(回退用): " << gnssBrdcPathBuf << (ok ? "" : "[失败]");
            if (!ok) loadOk = false;
        } else {
            LOG_INFO << "LEO 未填广播星历 BRDC：将以精密轨道 SP3 提供卫星位置（与 PPP 一致，BRDC 仅作回退）";
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
        // 卫星位置来源：SP3（精密）或 BRDC（广播回退）至少其一。两者皆空则全程无解。
        if (gnssSp3PathBuf.empty() && gnssBrdcPathBuf.empty()) {
            LOG_ERROR << "LEO 既未填精密轨道 SP3 也未填广播星历 BRDC：无法获取 GNSS 卫星位置，全历元无解";
            loadOk = false;
        }
        if (!loadOk) {
            core->setError("产品加载失败：需(精密轨道SP3 或 广播星历BRDC)至少其一提供卫星位置；"
                           "另检查 SP3/CLK/OSB/ATX/参考轨道/辅助文件路径是否正确");
            return false;
        }
            return true;
    }

    bool LeoTask::afterSolveEpoch(void *, GnssTask::EpochData *data,
                                  GnssTask::ObsItem *item, int idx) {
        if (!data->solved) return false;
        const Eigen::Vector3d leoPos = data->sppResult.xyz;
        if (leoPos.squaredNorm() <= 1e12) return false;   // 非合理 ECEF 位置(<=1e6 m)

        // 下方写入的 leoTraj/gnssTraj/rtn*/gnssVis 等都是**共享状态**：渲染侧(GuiSolverPanel)
        // 会在 core->mutex 下整份拷贝它们，而这里此前一把锁都没拿。push_back 触发重分配时，
        // 渲染侧并发拷贝就会读到已释放内存，或读到"size 与 data 指针不一致的半更新向量" →
        // Debug 下迭代器断言 / abort(0x80000003)。LEO 是唯一按历元 push 大向量的解算器，
        // 故只有它崩。锁顺序与 CommitEpoch 一致(先 core->mutex，后 plotMutex)，不会死锁。
        std::lock_guard lk(core->mutex);
        std::vector<GnssTask::SatVis> gnssVis;
        for (size_t si = 0; si < data->satIds.size(); ++si) {
                if (data->rejected[si]) continue;
                const PVT &sp = data->satPVTs[si];
                if (sp.p.squaredNorm() <= 1.0) continue;
                gnssVis.push_back(
                GnssTask::SatVis{data->satIds[si], data->satIds[si].system, sp.p, true});
                core->gnssTraj[data->satIds[si]].push_back({idx, sp.p});
        }
        core->gnssVis = std::move(gnssVis);   // 只留当前历元的可见星
        core->leoPos3d = leoPos;
        core->leoTraj.push_back({idx, leoPos});
        core->hasLeo = true;

        Eigen::Vector3d refPos = leoPos;   // 无参考轨道时自比，RTN 全零
        if (hasRefOrbit) {
                const PVT ref = refRdr.getPVT(refSat, item->obs.epoch);
                refPos = ref.p.squaredNorm() > 1e12 ? ref.p : leoPos;
                // RTN 分量（径向/沿迹/法向）由 CoordConvert::XYZToRTN 统一提供
                const Eigen::Vector3d rtn = XYZToRTN(leoPos, refPos, ref.v);
                data->refECEF = refPos;
                data->rtnR = rtn(0);
                data->rtnT = rtn(1);
                data->rtnN = rtn(2);
                core->leoRef3d = refPos;
                core->leoRefTraj.push_back({idx, refPos});
                core->rtnTimes.push_back(item->obs.epoch.m_sod);
                core->rtnR.push_back(rtn(0));
                core->rtnT.push_back(rtn(1));
                core->rtnN.push_back(rtn(2));
                // 3D 模长与 RTN 分量同源（正交框架下恒等于 √(R²+T²+N²)），由调用方自算
                // （XYZToRTN 只返回 (R,T,N)，不捎带与坐标系无关的模长）。
                core->rtnD3.push_back((leoPos - refPos).norm());
        }
        if (!core->initializedRefECEF) {
                core->refECEF = refPos;
                core->initializedRefECEF = true;
        }
        return true;
    }

    // OEM7 观测文件自带广播星历快照（RINEX 模式 item.eph 为 nullptr，星历来自
    // initSolver 加载的 SP3/BRDC）：逐历元注入 solver.ephTable，与 SPP GUI 同一钩子语义。
    // 快照为空时保留上一帧非空表（toe 不变时各历元本就共享同一 shared_ptr）。
    void LeoTask::beforeSolveEpoch(void *solverPtr, GnssTask::ObsItem *itemPtr) {
        if (!core->isRinex && itemPtr && itemPtr->eph) {
            auto &leo = *static_cast<SPP *>(solverPtr); // LEO→PPPKinematic→PPPStatic→SPP，ephTable 在 SPP
            const EphemerisTable *tbl = itemPtr->eph.get();
            if (tbl->gps.empty() && tbl->bds.empty() && lastGoodEph) tbl = lastGoodEph.get();
            else if (!tbl->gps.empty() || !tbl->bds.empty()) lastGoodEph = itemPtr->eph;
            leo.ephTable = *tbl;
            core->hasNav = true;
        }
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
        Solvers::Helpers::beginConfigBody("LEO 处理配置###leo_cfg_", *this);

        Solvers::Helpers::fileRow("观测文件 (RINEX .??O / OEM7 .log)", &this->leoObsPathBuf, "leo_obs", Solvers::Helpers::fObs, 3,
                            [&](const std::string &p) {
                                const std::string d = GnssTask::AutoDetectNav(p);
                                if (!d.empty()) this->gnssBrdcPathBuf = d;
                            }, "leo_obs");
        Solvers::Helpers::fileRow("广播星历 BRDC", &this->gnssBrdcPathBuf, "leo_gnss_brdc", Solvers::Helpers::fRnx, 2, {}, "leo_obs");
        Solvers::Helpers::fileRow("精密轨道 SP3", &this->gnssSp3PathBuf, "leo_gnss_sp3", Solvers::Helpers::fSp3, 2, {}, "leo_obs");
        Solvers::Helpers::fileRow("精密钟差 CLK", &this->gnssClkPathBuf, "leo_gnss_clk", Solvers::Helpers::fClk, 2, {}, "leo_obs");
        Solvers::Helpers::fileRow("天线文件 ATX", &this->atxPathBuf, "leo_atx", Solvers::Helpers::fAtx, 2, {}, "leo_obs");
        Solvers::Helpers::fileRow("码偏差 OSB/BIA", &this->osbPathBuf, "leo_osb", Solvers::Helpers::fOsb, 2, {}, "leo_obs");
        Solvers::Helpers::fileRow("参考轨道 SP3 (可选)", &this->refSp3PathBuf, "leo_ref_sp3", Solvers::Helpers::fSp3, 2, {}, "leo_obs");
        ImGui::Text("辅助文件 (.HDR 等):");
        if (!this->auxFiles.empty()) {
            for (size_t i = 0; i < this->auxFiles.size(); i++) {
                ImGui::Bullet();
                std::string shortName = Solvers::Helpers::baseName(this->auxFiles[i]);
                ImGui::Text("%s", shortName.c_str());
                ImGui::SameLine();
                if (ImGui::SmallButton(("删除##aux" + std::to_string(i)).c_str()))
                    this->auxFiles.erase(this->auxFiles.begin() + i);
            }
        }
        if (ImGui::Button("添加...")) {
            std::vector<std::wstring> wpaths;
            if (ShowOpenFilesDialog(wpaths, Solvers::Helpers::fHdr, 2)) {
                for (auto &wp: wpaths) {
                    std::string p = wideToAcp(wp);
                    if (std::find(this->auxFiles.begin(), this->auxFiles.end(), p) == this->auxFiles.end())
                        this->auxFiles.push_back(p);
                    AppConfig::PathMemory::instance().addRecent("leo_aux", p);
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("从记录添加")) ImGui::OpenPopup("##leo_aux_hist_pop");
        if (ImGui::BeginPopup("##leo_aux_hist_pop")) {
            const auto recent = AppConfig::PathMemory::instance().getRecent("leo_aux");
            if (recent.empty()) ImGui::TextDisabled("(暂无记录)");
            for (auto &r: recent) {
                if (ImGui::Selectable(r.c_str())) {
                    if (std::find(this->auxFiles.begin(), this->auxFiles.end(), r) == this->auxFiles.end())
                        this->auxFiles.push_back(r);
                    AppConfig::PathMemory::instance().addRecent("leo_aux", r);
                }
            }
            ImGui::EndPopup();
        }
        ImGui::Separator();
        Solvers::Helpers::renderCutoffAndSystems(&this->core->cutoffDeg, &this->core->enabledSystems);
        ImGui::Separator();
        if (ImGui::Button("开始解算", ImVec2(200, 40))) {
            const std::string path = this->leoObsPathBuf;
            // 格式判别是显式三值（与 SPP/RTK/PPP 同一入口 classifyObsPath）：无法识别时明确报错，
            // 不再默认落进 RINEX 读取器把 .log/.oem7 报成"无法打开 RINEX"误导用户
            const auto fmt = path.empty() ? GnssTask::ObsFormat::Unknown : GnssTask::classifyObsPath(path);
            if (path.empty()) {
                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "请先选择观测文件");
            } else if (fmt == GnssTask::ObsFormat::Unknown) {
                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1),
                                   "无法识别观测文件格式（支持 RINEX .??O/.rnx/.obs 与 OEM7 .log/.oem7）");
            } else {
                const auto &cfg = AppConfig::PathMemory::instance();
                cfg.set("leo_obs", path);
                cfg.set("leo_gnss_brdc", this->gnssBrdcPathBuf);
                cfg.set("leo_gnss_sp3", this->gnssSp3PathBuf);
                cfg.set("leo_gnss_clk", this->gnssClkPathBuf);
                cfg.set("leo_atx", this->atxPathBuf);
                cfg.set("leo_osb", this->osbPathBuf);
                cfg.set("leo_ref_sp3", this->refSp3PathBuf);
                cfg.set("leo_ref_prn", std::to_string(this->refPrn));
                cfg.setNum("leo_cutoff", this->core->cutoffDeg);
                cfg.set("leo_systems", GnssTask::sysSetToString(this->core->enabledSystems));

                {
                    const std::string refInfo =
                        this->refSp3PathBuf.empty() ? std::string("无") : this->refSp3PathBuf;
                    LOG_INFO << "[LEO] 开始定轨: 观测=" << path
                             << ", 格式=" << (fmt == GnssTask::ObsFormat::Rinex ? "RINEX" : "OEM7")
                             << ", 参考轨道=" << refInfo;
                }

                // 同 PPP：上一轮被取消/关闭过会残留 stop=true，不复位则本轮 0 历元且无报错。
                this->stop = false;
                this->core->stop = false;

                this->core->filePath = path;
                this->core->fileName = Solvers::Helpers::baseName(path);
                this->core->obsPathBuf = path;
                this->core->isRinex = (fmt == GnssTask::ObsFormat::Rinex);

                this->state = LeoTask::State::Running;
                this->loading = true;
                this->core->loading = true;
                this->worker = std::thread(SolveThread, std::static_pointer_cast<LeoTask>(shared_from_this()));
            }
        }
        ImGui::SameLine();
        Solvers::Helpers::renderCancelButton(this);

        Solvers::Helpers::endConfigBody();
    }

}
