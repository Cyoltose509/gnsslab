#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "GuiLeoProcessor.h"
#include "GuiHelpers.h"
#include "core/PathMemory.h"
#include "ui/Gui.h"
#include "imgui.h"
#include "SPP.h"
#include "LEO.h"
#include "Sp3OrbitReader.h"
#include "CoordConvert.h"
#include "Log.h"

#include <filesystem>
#include <thread>
#include <mutex>
#include <queue>
#include <string>
#include <vector>
#include <future>
#include "GuiSolverPanel.h"
#include "ObsFormat.h"

namespace Solvers::LeoProcessor {
    using GnssTask::EpochData;
    using GnssTask::SolveTask;
    using GnssTask::SolverMode;

    bool LeoTask::initSolver(void *solver) {
        LEO &leo = *static_cast<LEO *>(solver);
        hasRefOrbit = false;         // 参考轨道为可选，加载失败不致命
        bool loadOk = true;

        // 精密产品并行加载（与 PPP 共用 GuiHelpers::launchProductLoad：大 SP3/CLK 串行读取时首屏卡顿明显）
        {
            std::vector<std::future<bool>> jobs;
            Helpers::launchProductLoad(jobs, "LEO", "精密轨道 SP3", gnssSp3PathBuf, leo, &LEO::loadSp3);
            Helpers::launchProductLoad(jobs, "LEO", "精密钟差 CLK", gnssClkPathBuf, leo, &LEO::loadClk, core->enabledSystems);
            Helpers::launchProductLoad(jobs, "LEO", "码偏差 OSB", osbPathBuf, leo, &LEO::loadOsb);
            Helpers::launchProductLoad(jobs, "LEO", "天线文件 ATX", atxPathBuf, leo, &LEO::loadAtx);
            Helpers::launchProductLoad(jobs, "LEO", "广播星历 BRDC(回退用)", gnssBrdcPathBuf, leo, &LEO::loadBrdc);
            for (auto &j : jobs) {
                if (!j.get()) loadOk = false;
            }
        }
        if (gnssBrdcPathBuf.empty())
            LOG_INFO << "LEO 未填广播星历 BRDC：将以精密轨道 SP3 提供卫星位置（与 PPP 一致，BRDC 仅作回退）";

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

    bool LeoTask::afterSolveEpoch(void *, EpochData *data,
                                  GnssTask::ObsItem *item, const int index) {
        if (!data->solved) return false;
        const Eigen::Vector3d leoPos = data->sppResult.xyz;
        if (leoPos.squaredNorm() <= 1e12) return false;

        std::lock_guard lk(core->mutex);
        std::vector<GnssTask::SatVis> gnssVis;
        for (size_t si = 0; si < data->satIds.size(); ++si) {
                if (data->rejected[si]) continue;
                const PVT &sp = data->satPVTs[si];
                if (sp.p.squaredNorm() <= 1.0) continue;
                gnssVis.push_back(
                GnssTask::SatVis{data->satIds[si], data->satIds[si].system, sp.p, true});
                core->gnssTraj[data->satIds[si]].push_back({index, sp.p});
        }
        core->gnssVis = std::move(gnssVis);   // 只留当前历元的可见星
        core->leoPos3d = leoPos;
        core->leoTraj.push_back({index, leoPos});
        core->hasLeo = true;

        Eigen::Vector3d refPos = leoPos;   // 无参考轨道时自比，RTN 全零
        if (hasRefOrbit) {
                const PVT ref = refRdr.getPVT(refSat, item->obs.epoch);
                refPos = ref.p.squaredNorm() > 1e12 ? ref.p : leoPos;
                const Eigen::Vector3d rtn = XYZToRTN(leoPos, refPos, ref.v);
                data->refECEF = refPos;
                data->rtnR = rtn(0);
                data->rtnT = rtn(1);
                data->rtnN = rtn(2);
                core->leoRef3d = refPos;
                core->leoRefTraj.push_back({index, refPos});
                core->rtnTimes.push_back(item->obs.epoch.m_sod);
                core->rtnR.push_back(rtn(0));
                core->rtnT.push_back(rtn(1));
                core->rtnN.push_back(rtn(2));
                core->rtnD3.push_back((leoPos - refPos).norm());
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
        GnssTask::RunDualThreadJob(task, "LEO", SolverThread);
    }

    void LeoTask::renderConfigPanel() {
        Helpers::beginConfigBody("LEO 处理配置###leo_cfg_", *this);

        Helpers::fileRow("观测文件 (RINEX .??O / OEM7 .log)", &this->leoObsPathBuf, "leo_obs", Helpers::fObs, 3,
                            {}, "leo_obs", &this->gnssBrdcPathBuf);
        Helpers::fileRow("广播星历 BRDC", &this->gnssBrdcPathBuf, "leo_gnss_brdc", Helpers::fRnx, 2, {}, "leo_obs");
        Helpers::fileRow("精密轨道 SP3", &this->gnssSp3PathBuf, "leo_gnss_sp3", Helpers::fSp3, 2, {}, "leo_obs");
        Helpers::fileRow("精密钟差 CLK", &this->gnssClkPathBuf, "leo_gnss_clk", Helpers::fClk, 2, {}, "leo_obs");
        Helpers::fileRow("天线文件 ATX", &this->atxPathBuf, "leo_atx", Helpers::fAtx, 2, {}, "leo_obs");
        Helpers::fileRow("码偏差 OSB/BIA", &this->osbPathBuf, "leo_osb", Helpers::fOsb, 2, {}, "leo_obs");
        Helpers::fileRow("参考轨道 SP3 (可选)", &this->refSp3PathBuf, "leo_ref_sp3", Helpers::fSp3, 2, {}, "leo_obs");
        ImGui::Text("辅助文件 (.HDR 等):");
        if (!this->auxFiles.empty()) {
            for (size_t i = 0; i < this->auxFiles.size(); i++) {
                ImGui::Bullet();
                std::string shortName = Helpers::baseName(this->auxFiles[i]);
                ImGui::Text("%s", shortName.c_str());
                ImGui::SameLine();
                if (ImGui::SmallButton(("删除##aux" + std::to_string(i)).c_str()))
                    this->auxFiles.erase(this->auxFiles.begin() + i);
            }
        }
        if (ImGui::Button("添加...")) {
            if (std::vector<std::wstring> wpaths; ShowOpenFilesDialog(wpaths, Helpers::fHdr, 2)) {
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
        Helpers::renderCutoffAndSystems(&this->core->cutoffDeg, &this->core->enabledSystems);
        ImGui::Separator();
        if (ImGui::Button("开始解算", ImVec2(200, 40))) {
            // 校验/停旧任务/填 core/置运行态走公共 helper（与 SPP/PPP 同一份）；
            // ini 持久化键与日志文案各家不同，留在调用方。校验不过时红字已由 helper 绘出。
            if (Helpers::beginFileSolve(this, this->leoObsPathBuf)) {
                const std::string &path = this->leoObsPathBuf;
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
                             << ", 格式=" << (this->core->isRinex ? "RINEX" : "OEM7")
                             << ", 参考轨道=" << refInfo;
                }

                this->worker = std::thread(SolveThread, std::static_pointer_cast<LeoTask>(shared_from_this()));
            }
        }
        ImGui::SameLine();
        Helpers::renderCancelButton(this);

        Helpers::endConfigBody();
    }

}
