#include "GuiInsPanel.h"
#include "GuiInsCommon.h"

#include "GuiHelpers.h"
#include "SolverTask.h"
#include "core/PathMemory.h"
#include "io/AsciiLogReader.h"
#include "io/OEM7Reader.h"
#include "ins/ImuCalibrator.h"
#include "core/InsMath.h"
#include "Const.h"
#include "util/StringUtils.h"

#include <algorithm>
#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace Tools::InsPanel {
    using namespace insui;

    SixPosCalibTask::SixPosCalibTask() {
        mode = GnssTask::SolverMode::Ins;
        fileName = "六位置法标定";
    }


    void SixPosCalibTask::startCalib() {
        {
            // 实验室常量无条件持久化（校验失败也落盘，同四解算器铁律）
            const auto &cfg = AppConfig::PathMemory::instance();
            cfg.setNum("ins_g", g);
            cfg.setNum("ins_lat", latDeg);
            cfg.setNum("ins_we", earthRate);
            cfg.setNum("ins_turn", turnAngleDeg);
            cfg.setNum("ins_k_acc", kAccelLsb);
            cfg.setNum("ins_k_gyro", kGyroLsb);
            // 文件列表持久化（单一键，'|' 分隔）
            std::string joined;
            for (size_t i = 0; i < filePaths.size(); ++i) {
                if (i) joined += '|';
                joined += filePaths[i];
            }
            cfg.set("ins_cal_files", joined);
        }
        if (filePaths.empty()) {
            setError("请先添加至少一个 IMU 日志文件");
            return;
        }
        if (g <= 0.0 || turnAngleDeg <= 0.0) {
            setError("重力加速度 g 与转台转角 α 必须为正数");
            return;
        }
        if (!(kAccelLsb > 0.0) || !(kGyroLsb > 0.0)) {
            setError("解码当量 k_a、k_g 必须为正数");
            return;
        }
        state = State::Running;
        loading = true;
        if (worker.joinable()) worker.join();
        stop = false;
        hasResult.store(false);
        accelReady.store(false);
        sampleCount.store(0);
        segStatic.store(0);
        segRot.store(0);
        dupSkipped.store(0);
        detectDone.store(false);
        chartDirty.store(false);
        {
            std::lock_guard lk(resultMutex);
            currentFile.clear();
            detectLines.clear();
            detectSeq.clear();
            fileCharts.clear();
            accelMp.setIdentity();
            accelBp.setZero();
            accelScaleGp.setZero();
            accelRmsp = 0.0;
            report = CalibReport();
        }
        std::shared_ptr<SolverTaskBase> self = shared_from_this();
        worker = std::thread([self] { dynamic_cast<SixPosCalibTask *>(self.get())->calibWorker(); });
    }

    void SixPosCalibTask::calibWorker() {
        const double kAccel = kAccelLsb;
        const double kGyroRad = kGyroLsb * DEG_TO_RAD;
        ImuCalibrator::CalibSegments segs;
        std::vector<std::string> detLines;
        std::vector<std::pair<std::string, std::string> > detSeqLines;
        int dups = 0;

        auto parseOne = [&](const std::string &path, ImuLog &out) -> bool {
            out.clear();
            auto onSample = [&](const ImuSample &s) { out.push_back(s); };
            auto onBatch = [&](const long long n) { sampleCount.store(n); };
            if (endsWithIC(path, ".asc")) {
                AsciiLogReader r;
                if (!r.open(path)) return false;
                pumpImuReader(r, stop, onSample, onBatch);
            } else {
                OEM7Reader r;
                if (!r.open(path)) return false;
                pumpImuReader(r, stop, onSample, onBatch);
            }
            return !out.empty();
        };

        std::set<std::string> seenHash;
        int nStatic = 0, nRot = 0;
        std::vector<PerFileChart> fcs; // 逐文件的原始波形（任意数量文件）

        auto buildRaw = [&](const ImuLog &L, PerFileChart &fc) {
            if (L.empty()) return;
            const size_t stride = std::max<size_t>(1, (L.size() + 3999) / 4000);
            const double t0 = L[0].tow;
            const size_t cap = (L.size() + stride - 1) / stride;
            fc.t.reserve(cap);
            fc.ax.reserve(cap);
            fc.ay.reserve(cap);
            fc.az.reserve(cap);
            fc.gx.reserve(cap);
            fc.gy.reserve(cap);
            fc.gz.reserve(cap);
            for (size_t k = 0; k < L.size(); k += stride) {
                const auto &s = L[k];
                fc.t.push_back(s.tow - t0);
                fc.ax.push_back(s.ax);
                fc.ay.push_back(s.ay);
                fc.az.push_back(s.az);
                fc.gx.push_back(s.gx);
                fc.gy.push_back(s.gy);
                fc.gz.push_back(s.gz);
            }
        };

        for (const auto &path: filePaths) {
            if (stop) break;
            const std::string name = Solvers::Helpers::baseName(path);
            {
                std::lock_guard lk(resultMutex);
                currentFile = name;
            }
            ImuLog log;
            if (!parseOne(path, log) || log.empty()) {
                if (!stop) {
                    detLines.push_back(name + " : 无法解析或为空");
                    detSeqLines.emplace_back(name, "空/解析失败");
                }
                continue;
            }
            PerFileChart fc;
            fc.name = name;
            buildRaw(log, fc);

            // 重复文件去重：仅影响标定聚合，波形依旧展示
            const std::string hash = fileContentHash(path);
            if (seenHash.count(hash)) {
                ++dups;
                fc.dup = true;
                fc.det = "重复文件，已跳过（仅展示波形）";
                detLines.push_back(name + " : " + fc.det);
                detSeqLines.emplace_back(name, "重复文件(内容相同)已跳过");
                fcs.push_back(std::move(fc));
                continue;
            }
            seenHash.insert(hash);


            auto segsFile = ImuCalibrator::detect(log);
            int cPos[6] = {0, 0, 0, 0, 0, 0}, cRot[6] = {0, 0, 0, 0, 0, 0};
            std::string seq;
            for (const auto &[isStatic, pos, rot, i0, i1]: segsFile) {
                if (i1 <= i0) continue;
                ImuLog sub(log.begin() + i0, log.begin() + i1);
                if (isStatic) {
                    cPos[static_cast<int>(pos)]++;
                    ++nStatic;
                    segs.pos.emplace_back(pos, std::move(sub));
                    seq += std::string(seq.empty() ? "" : " ") + ImuCalibrator::posLabel(pos);
                } else {
                    cRot[static_cast<int>(rot)]++;
                    ++nRot;
                    segs.rot.emplace_back(rot, std::move(sub));
                    seq += std::string(seq.empty() ? "" : " ") + ImuCalibrator::rotLabel(rot);
                }
            }
            if (seq.empty()) seq = "未识别到有效段";

            std::string line;
            bool any = false;
            for (int p = 0; p < 6; ++p)
                if (cPos[p]) {
                    if (any) line += " ";
                    line += std::string(ImuCalibrator::posLabel(static_cast<ImuCalibrator::Pos>(p))) + "×" + std::to_string(cPos[p]);
                    any = true;
                }
            for (int r = 0; r < 6; ++r)
                if (cRot[r]) {
                    if (any) line += " ";
                    line += std::string(ImuCalibrator::rotLabel(static_cast<ImuCalibrator::Rot>(r))) + "×" + std::to_string(cRot[r]);
                    any = true;
                }
            if (!any) line = "未识别到静置/旋转段";
            fc.det = line;
            detLines.push_back(name + " : " + line);
            detSeqLines.emplace_back(name, seq);

            fcs.push_back(std::move(fc));
        }

        segStatic.store(nStatic);
        segRot.store(nRot);
        dupSkipped.store(dups);
        {
            std::lock_guard lk(resultMutex);
            detectLines = detLines;
            detectSeq = detSeqLines;
            detectDone.store(true);
        }

        if (stop) {
            loading = false;
            state = State::Done;
            return;
        }

        if (segs.pos.empty() && segs.rot.empty()) {
            setError("未检测到任何静置/旋转段，请检查文件内容");
            loading = false;
            state = State::Done;
            return;
        }
        CalibReport rep = ImuCalibrator::calibrate(segs, turnAngleDeg * DEG_TO_RAD, g);
        if (!rep.ok) {
            setError(rep.errorMsg);
            loading = false;
            state = State::Done;
            return;
        }
        ImuCalibrator::applyNominal(rep, kAccel, kGyroRad);

        {
            std::lock_guard lk(resultMutex);
            compSegs = segs;
            compSel = 0;
            for (int i = 0; i < static_cast<int>(compSegs.pos.size()); ++i)
                if (compSegs.pos[i].first == ImuCalibrator::XUP) {
                    compSel = i;
                    break;
                }
        }

        {
            std::lock_guard lk(resultMutex);
            fileCharts = std::move(fcs);
            accelMp = rep.calib.accelM;
            accelBp = rep.calib.accelB;
            accelScaleGp = rep.accelScaleG;
            accelRmsp = rep.accelRms;
            chartDirty.store(true);
        }
        accelReady.store(true);

        {
            std::lock_guard lk(resultMutex);
            report = std::move(rep);
        }
        hasResult.store(true);
        loading = false;
        state = State::Done;
    }

    void SixPosCalibTask::renderConfigPanel() {
        Solvers::Helpers::beginConfigBody("六位置法标定配置###cfg_", *this);

        if (ImGui::BeginTabBar("##cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                ImGui::Spacing();
                ImGui::Text("添加任意数量的 IMU 日志文件，程序自动检测每个文件包含静置段与旋转段");
                ImGui::Spacing();

                if (ImGui::Button("添加文件...", ImVec2(140, 0))) {
                    if (std::vector<std::wstring> wpaths; ShowOpenFilesDialog(wpaths, Solvers::Helpers::fIns,
                                                                              std::size(Solvers::Helpers::fIns))) {
                        for (auto &wp: wpaths) {
                            std::string p = wideToAcp(wp);
                            if (!p.empty() &&
                                std::find(filePaths.begin(), filePaths.end(), p) == filePaths.end())
                                filePaths.push_back(p);
                            AppConfig::PathMemory::instance().addRecent("ins_cal_files", p);
                        }
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("从记录添加", ImVec2(120, 0)))
                    ImGui::OpenPopup("##cal_hist_pop");
                ImGui::SameLine();
                if (ImGui::Button("清空", ImVec2(80, 0)) && !filePaths.empty())
                    filePaths.clear();

                if (ImGui::BeginPopup("##cal_hist_pop")) {
                    const auto recent =
                            AppConfig::PathMemory::instance().getRecent("ins_cal_files");
                    if (recent.empty()) ImGui::TextDisabled("(暂无记录)");
                    for (auto &r: recent) {
                        if (ImGui::Selectable(Solvers::Helpers::baseName(r).c_str())) {
                            if (std::find(filePaths.begin(), filePaths.end(), r) == filePaths.end())
                                filePaths.push_back(r);
                        }
                    }
                    ImGui::EndPopup();
                }

                ImGui::Spacing();
                ImGui::Separator();
                if (filePaths.empty()) {
                    ImGui::TextDisabled("尚未添加任何文件。点击「添加文件...」选择标定日志。");
                } else {
                    ImGui::Text("已选 %d 个文件：", static_cast<int>(filePaths.size()));
                    for (size_t i = 0; i < filePaths.size(); ++i) {
                        ImGui::Bullet();
                        ImGui::Text("%s", Solvers::Helpers::baseName(filePaths[i]).c_str());
                        ImGui::SameLine();
                        if (ImGui::SmallButton(("x##cal" + std::to_string(i)).c_str())) {
                            filePaths.erase(filePaths.begin() + i);
                            --i;
                        }
                    }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("参数")) {
                ImGui::Text("站点 / 仪器参数：");
                ImGui::PushItemWidth(260);
                ImGui::InputDouble("当地重力加速度 g (m/s²)", &g, 0.0, 0.0, "%.6f");
                ImGui::InputDouble("当地纬度 φ (°N)", &latDeg, 0.0, 0.0, "%.6f");
                ImGui::InputDouble("地球自转率 ωe (rad/s)", &earthRate, 0.0, 0.0, "%.12f");
                ImGui::InputDouble("转台精确转角 α (°)", &turnAngleDeg, 0.0, 0.0, "%.3f");
                ImGui::PopItemWidth();

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Text("解码当量：");
                ImGui::PushItemWidth(260);
                ImGui::InputDouble("加速度计当量 k_a ((m/s)/LSB)", &kAccelLsb, 0.0, 0.0, "%.12f");
                ImGui::InputDouble("陀螺当量 k_g (°/LSB)", &kGyroLsb, 0.0, 0.0, "%.12f");
                ImGui::PopItemWidth();
                if (kGyroLsb > 0.0 && kGyroLsb < 1e-8)
                    ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f),
                                       "本栏单位是 °/LSB，别填 rad/LSB 的值");
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        ImGui::Spacing();
        if (ImGui::Button("开始标定", ImVec2(200, 40))) startCalib();
        ImGui::SameLine();
        Solvers::Helpers::renderCancelButton(this);
        if (errorFlag() && !hasResult)
            ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "%s", errorMessage().c_str());

        Solvers::Helpers::endConfigBody();
    }

    namespace {
        enum ResultUnitRow {
            UR_ACC_SCALE = 0, // 加速度计比例因子
            UR_ACC_BIAS, // 加速度计零偏
            UR_ACC_DS, // 加速度计比例因子误差
            UR_GYRO_KEFF, // 陀螺有效比例因子
            UR_GYRO_SCALE, // 陀螺比例因子
            UR_GYRO_BIAS, // 陀螺零偏
            UR_GYRO_DS, // 陀螺比例因子误差
            UR_COUNT
        };

        int gResultUnit[UR_COUNT] = {0, 0, 1, 0, 0, 0, 1};

        int pmDecimals(const double s) {
            if (!(s > 0.0) || !std::isfinite(s)) return 3;
            const int e = static_cast<int>(std::floor(std::log10(s)));
            return std::clamp(1 - e, 0, 6);
        }
    }

    void SixPosCalibTask::renderResult() {
        const bool running = state == State::Running;
        if (errorFlag()) {
            ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "错误: %s", errorMessage().c_str());
        }
        const bool detReady = detectDone.load();
        {
            std::lock_guard lk(resultMutex);
            if (detReady) {
                ImGui::Spacing();
                //ImGui::Text("检测到的内容：");
                // for (const auto &l: detectLines) ImGui::Text("  %s", l.c_str());
                ImGui::Text("已识别 静置段 %d / 旋转段 %d，跳过重复 %d，已处理历元 %lld",
                            segStatic.load(), segRot.load(), dupSkipped.load(),
                            static_cast<long long>(sampleCount.load()));
            }
        }
        if (detReady && ImGui::Button("导出分段摘要 CSV", ImVec2(200, 0))) exportDetectCsv();
        if (errorFlag()) return; // 错误时只展示错误 + 检测摘要
        if (running) {
            std::string label;
            {
                std::lock_guard lk(resultMutex);
                label = currentFile;
            }
            ImGui::TextColored(ImVec4(0.6f, 0.6f, 1.0f, 1.0f),
                               "正在解析 [%s]：静置段 %d / 旋转段 %d / 重复 %d / %lld 历元",
                               label.empty() ? "—" : label.c_str(),
                               segStatic.load(), segRot.load(), dupSkipped.load(),
                               static_cast<long long>(sampleCount.load()));
            if (ImGui::Button("停止")) stop = true;
            ImGui::Spacing();
        }

        CalibReport rep;
        std::vector<PerFileChart> fcSnap;
        bool aReady = false, hResult = false;
        Eigen::Vector3d accS;
        double arm = 0.0;
        {
            std::lock_guard lk(resultMutex);
            rep = report;
            fcSnap = fileCharts;
            aReady = accelReady.load();
            hResult = hasResult.load();
            accS = accelScaleGp;
            arm = accelRmsp;
        }
        const bool chartNew = chartDirty.exchange(false);

        if (ImGui::BeginTable("##cal_tbl", 5,
                              ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("量", ImGuiTableColumnFlags_WidthStretch, 3.4f);
            ImGui::TableSetupColumn("X", ImGuiTableColumnFlags_WidthStretch, 1.7f);
            ImGui::TableSetupColumn("Y", ImGuiTableColumnFlags_WidthStretch, 1.7f);
            ImGui::TableSetupColumn("Z", ImGuiTableColumnFlags_WidthStretch, 1.7f);
            ImGui::TableSetupColumn("单位", ImGuiTableColumnFlags_WidthStretch, 1.7f);
            ImGui::TableHeadersRow();

            auto unitCombo = [](const int row, const char *const *units, const int nu) {
                ImGui::PushID(row);
                ImGui::SetNextItemWidth(-FLT_MIN);
                if (ImGui::BeginCombo("##unit", units[gResultUnit[row]])) {
                    for (int i = 0; i < nu; ++i) {
                        ImGui::PushID(i);
                        if (ImGui::Selectable(units[i], i == gResultUnit[row])) gResultUnit[row] = i;
                        ImGui::PopID();
                    }
                    ImGui::EndCombo();
                }
                ImGui::PopID();
            };

            auto unitFixed = [](const char *u) { ImGui::TextDisabled("%s", u); };
            auto rowUnit = [&](const char *label, const int row, const Eigen::Vector3d &v, const char *fmt,
                               const char *const *units, const int nu, const bool ready) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(label);
                for (int c = 0; c < 3; ++c) {
                    ImGui::TableSetColumnIndex(1 + c);
                    if (ready) {
                        char b[48];
                        std::snprintf(b, sizeof(b), fmt, v(c));
                        ImGui::TextUnformatted(b);
                    } else {
                        ImGui::TextDisabled("计算中");
                    }
                }
                ImGui::TableSetColumnIndex(4);
                if (nu > 1) unitCombo(row, units, nu);
                else unitFixed(units[0]);
            };

            static const char *kAccScaleU[3] = {"LSB/g", "LSB/(m/s²)", "LSB/Gal"};
            static const char *kAccBiasU[5] = {"LSB", "m/s²", "mg", "mGal", "Gal"};
            static const char *kDsU[3] = {"无量纲", "ppm", "%"};
            static const char *kGyroKeffU[2] = {"LSB/rad", "LSB/°"};
            static const char *kGyroScaleU[2] = {"LSB/(rad/s)", "LSB/(°/h)"};
            static const char *kGyroBiasU[3] = {"LSB/历元", "rad/s", "°/h"};

            static const char *kDsFmt[3] = {"%.6f", "%.2f", "%.4f"}; // 无量纲 / ppm / %
            static constexpr double kDsMul[3] = {1.0, 1e6, 100.0};


            {
                const int ua = gResultUnit[UR_ACC_SCALE];
                // LSB/g -> LSB/(m/s²): ÷g；LSB/Gal: 1 Gal = 0.01 m/s²，即 ×0.01/g
                const double f = ua == 0 ? 1.0 : (ua == 1 ? 1.0 / g : 0.01 / g);
                const Eigen::Vector3d v = accS * f;
                const Eigen::Vector3d sg = rep.accelScaleSigma * g * f;
                int d = 0;
                for (int c = 0; c < 3; ++c) d = std::max(d, pmDecimals(sg(c)));
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted("加速度计比例因子 K_a");
                for (int c = 0; c < 3; ++c) {
                    ImGui::TableSetColumnIndex(1 + c);
                    if (aReady) {
                        char b[48];
                        std::snprintf(b, sizeof(b), "%.*f ± %.*f", d, v(c), d, sg(c));
                        ImGui::TextUnformatted(b);
                    } else {
                        ImGui::TextDisabled("计算中");
                    }
                }
                ImGui::TableSetColumnIndex(4);
                unitCombo(UR_ACC_SCALE, kAccScaleU, 3);
            }
            {
                const int u = gResultUnit[UR_ACC_BIAS];
                static const char *fmtU[5] = {"%.2f", "%.6f", "%.4f", "%.2f", "%.4f"};
                Eigen::Vector3d v = rep.calib.accelB;
                if (u == 1) v = rep.accelBiasPhys;
                else if (u == 2) v = rep.accelBiasPhys / (g * 1e-3);
                else if (u == 3) v = rep.accelBiasPhys / 1e-5; // 1 mGal = 1e-5 m/s²
                else if (u == 4) v = rep.accelBiasPhys / 1e-2; // 1 Gal  = 1e-2 m/s²
                rowUnit("加速度计零偏 b_a", UR_ACC_BIAS, v, fmtU[u], kAccBiasU, 5, aReady);
            }
            {
                const int u = gResultUnit[UR_ACC_DS];
                rowUnit("加速度计比例因子误差 δs_a", UR_ACC_DS, rep.accelDeltaS * kDsMul[u], kDsFmt[u], kDsU, 3,
                        aReady);
            }
            {
                const bool deg = (gResultUnit[UR_GYRO_KEFF] == 1);
                const Eigen::Vector3d v = deg ? rep.gyroKeff * DEG_TO_RAD : rep.gyroKeff;
                rowUnit("陀螺有效比例因子 K_eff,g", UR_GYRO_KEFF, v, "%.1f", kGyroKeffU, 2, hResult);
            }
            {
                const bool degH = (gResultUnit[UR_GYRO_SCALE] == 1);
                const Eigen::Vector3d v = degH ? rep.calib.gyroScale * DEG_TO_RAD / 3600.0
                                               : rep.calib.gyroScale;
                rowUnit("陀螺比例因子 S_g", UR_GYRO_SCALE, v, degH ? "%.4f" : "%.1f", kGyroScaleU, 2, hResult);
            }
            {
                const int u = gResultUnit[UR_GYRO_BIAS];
                static const char *fmtU[3] = {"%.3f", "%.9f", "%.3f"};
                const Eigen::Vector3d bgDegHour = rep.gyroBiasPhys * RAD_TO_DEG * 3600.0;
                Eigen::Vector3d v = rep.calib.gyroBias;
                if (u == 1) v = rep.gyroBiasPhys;
                else if (u == 2) v = bgDegHour;
                rowUnit("陀螺零偏 b_g", UR_GYRO_BIAS, v, fmtU[u], kGyroBiasU, 3, hResult);
            }
            {
                const int u = gResultUnit[UR_GYRO_DS];
                rowUnit("陀螺比例因子误差 δs_g", UR_GYRO_DS, rep.gyroDeltaS * kDsMul[u], kDsFmt[u], kDsU, 3,
                        hResult);
            }
            ImGui::EndTable();
        }
        ImGui::Spacing();
        if (aReady) {
            ImGui::Spacing();
            ImGui::Text("加速度计交轴耦合矩阵 N_a（无量纲，对角为 0）");
            if (ImGui::BeginTable("##cross_tbl", 4,
                                  ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableSetupColumn("");
                ImGui::TableSetupColumn("输入 X");
                ImGui::TableSetupColumn("输入 Y");
                ImGui::TableSetupColumn("输入 Z");
                ImGui::TableHeadersRow();
                static const char *kOut[3] = {"输出 X", "输出 Y", "输出 Z"};
                for (int i = 0; i < 3; ++i) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(kOut[i]);
                    for (int j = 0; j < 3; ++j) {
                        ImGui::TableSetColumnIndex(1 + j);
                        char b[48];
                        std::snprintf(b, sizeof(b), "%.7f", rep.accelCross(i, j));
                        ImGui::TextUnformatted(b);
                    }
                }
                ImGui::EndTable();
            }
        }

        {
            const double meanG = accS.mean();
            const double spreadG = accS.maxCoeff() - accS.minCoeff();
            const double maxds = rep.gyroDeltaS.cwiseAbs().maxCoeff();
            ImGui::Spacing();
            ImGui::Text("质量判读");
            if (ImGui::BeginTable("##qc_tbl", 4,
                                  ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp)) {
                ImGui::TableSetupColumn("检查项", ImGuiTableColumnFlags_WidthStretch, 1.9f);
                ImGui::TableSetupColumn("判据", ImGuiTableColumnFlags_WidthStretch, 1.6f);
                ImGui::TableSetupColumn("实测", ImGuiTableColumnFlags_WidthStretch, 2.6f);
                ImGui::TableSetupColumn("结论", ImGuiTableColumnFlags_WidthStretch, 0.6f);
                ImGui::TableHeadersRow();
                auto qcRow = [](const char *item, const char *cond, const bool ready, const char *val, const bool ok) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(item);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextDisabled("%s", cond);
                    ImGui::TableSetColumnIndex(2);
                    if (ready) ImGui::TextUnformatted(val);
                    else ImGui::TextDisabled("计算中");
                    ImGui::TableSetColumnIndex(3);
                    if (!ready) ImGui::TextDisabled("—");
                    else ImGui::TextColored(ok ? kColPass : kColFail, "%s", ok ? "PASS" : "FAIL");
                };
                char v[128];
                const bool okG = aReady && meanG > 1.0 && spreadG / meanG < 0.05;
                std::snprintf(v, sizeof(v), "%.2f %%", meanG > 0.0 ? spreadG / meanG * 100.0 : 0.0);
                qcRow("加速度计三轴比例因子一致性", "(max−min)/mean < 5 %", aReady, v, okG);

                const bool okRms = aReady && arm < 0.02 * meanG;
                std::snprintf(v, sizeof(v), "%.2f LSB", arm);
                qcRow("六位置拟合残差 RMS", "RMS < 2 % · mean K_a", aReady, v, okRms);

                const bool okDs = hResult && maxds < 0.05;
                std::snprintf(v, sizeof(v), "%.3f %%", maxds * 100.0);
                qcRow("陀螺比例因子误差", "max|δs_g| < 5 %", hResult, v, okDs);
                ImGui::EndTable();
            }
        }

        ImGui::Spacing();
        ImGui::Text("比例因子一致性");
        const double scA[3] = {accS(0), accS(1), accS(2)};
        const double scG[3] = {rep.gyroKeff(0), rep.gyroKeff(1), rep.gyroKeff(2)};
        twoCol([&] { drawScaleBars("加速度计三轴比例因子 (LSB/g)", aReady, scA, kColZ, "%.1f"); },
               [&] { drawScaleBars("陀螺有效比例因子 K_eff,g (LSB/rad)", hResult, scG, kColZ, "%.0f"); });


        ImGui::Spacing();
        if (ImGui::Button("导出标定结果 CSV", ImVec2(180, 0))) exportCalibCsv();

        ImGui::SameLine();
        ImGui::TextDisabled("|  补偿序列");
        if (const auto segLabels = compSegLabels(); segLabels.empty()) {
            ImGui::SameLine();
            ImGui::TextDisabled("标定完成后可选择段导出");
        } else {
            compSel = std::clamp(compSel, 0, static_cast<int>(segLabels.size()) - 1);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(190.0f);
            if (ImGui::BeginCombo("##comp_seg", segLabels[compSel].c_str())) {
                for (int i = 0; i < static_cast<int>(segLabels.size()); ++i) {
                    const bool sel = i == compSel;
                    ImGui::PushID(i);
                    if (ImGui::Selectable(segLabels[i].c_str(), sel)) compSel = i;
                    if (sel) ImGui::SetItemDefaultFocus();
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            if (ImGui::Button("导出该段补偿序列", ImVec2(210, 0))) exportCompCsv();
        }

        ImGui::Spacing();
        ImGui::Text("各文件原始输出");
        for (const auto &fc: fcSnap) {
            ImGui::Spacing();
            ImGui::Text("%s   [%s]%s", fc.name.c_str(), fc.det.c_str(),
                        fc.dup ? "（重复，未计入标定）" : "");
            twoCol([&] { drawAccelChart((fc.name + " · 加速度计").c_str(), fc, chartNew); },
                   [&] { drawGyroChart((fc.name + " · 陀螺仪").c_str(), fc, chartNew); });
        }
    }

    // 导出前统一快照标定结果：无结果只报一次错，三个导出共用同一句提示。
    bool SixPosCalibTask::snapshotReport(CalibReport &r) {
        if (!hasResult.load()) {
            setError("尚无标定结果，请先点「开始标定」");
            return false;
        }
        std::lock_guard lk(resultMutex);
        r = report;
        return true;
    }

    void SixPosCalibTask::exportCalibCsv() {
        CalibReport r;
        if (!snapshotReport(r)) return;
        exportCsvDialog(*this, "calib_result.csv", "calib_result.csv", [&](Csv::Writer &w) {
            // 每个量只导出界面结果表当前所选单位（gResultUnit）的一套数值，单位写入行名括号
            static const char *kAccScaleU[3] = {"LSB/g", "LSB/(m/s^2)", "LSB/Gal"};
            static const char *kAccBiasU[5] = {"LSB", "m/s^2", "mg", "mGal", "Gal"};
            static const char *kDsU[3] = {"无量纲", "ppm", "%"};
            static const char *kGyroKeffU[2] = {"LSB/rad", "LSB/deg"};
            static const char *kGyroScaleU[2] = {"LSB/(rad/s)", "LSB/(deg/h)"};
            static const char *kGyroBiasU[3] = {"LSB/历元", "rad/s", "deg/h"};
            static constexpr double kDsMul[3] = {1.0, 1e6, 100.0}; // 无量纲 / ppm / %
            w.row("量", "X", "Y", "Z");

            const int ua = gResultUnit[UR_ACC_SCALE];
            const int ub = gResultUnit[UR_ACC_BIAS];
            const int uda = gResultUnit[UR_ACC_DS];
            const int uk = gResultUnit[UR_GYRO_KEFF];
            const int us = gResultUnit[UR_GYRO_SCALE];
            const int ug = gResultUnit[UR_GYRO_BIAS];
            const int udg = gResultUnit[UR_GYRO_DS];

            // 加速度计比例因子（LSB/g 基准）：LSB/(m/s^2) ÷g；LSB/Gal ×0.01/g（1 Gal = 0.01 m/s²）
            const double af = ua == 0 ? 1.0 : (ua == 1 ? 1.0 / g : 0.01 / g);
            const Eigen::Vector3d ka = r.accelScaleG * af;
            const Eigen::Vector3d ksg = r.accelScaleSigma * g * af;
            w.row("加速度计比例因子(" + std::string(kAccScaleU[ua]) + ")", Csv::fx(ka(0), 4),
                  Csv::fx(ka(1), 4), Csv::fx(ka(2), 4));
            w.row("加速度计比例因子1σ(" + std::string(kAccScaleU[ua]) + ")", Csv::fx(ksg(0), 4),
                  Csv::fx(ksg(1), 4), Csv::fx(ksg(2), 4));

            Eigen::Vector3d ba = r.calib.accelB;
            if (ub == 1) ba = r.accelBiasPhys;
            else if (ub == 2) ba = r.accelBiasPhys / (g * 1e-3);
            else if (ub == 3) ba = r.accelBiasPhys / 1e-5; // 1 mGal = 1e-5 m/s²
            else if (ub == 4) ba = r.accelBiasPhys / 1e-2; // 1 Gal  = 1e-2 m/s²
            w.row("加速度计零偏(" + std::string(kAccBiasU[ub]) + ")", Csv::fx(ba(0), 4),
                  Csv::fx(ba(1), 4), Csv::fx(ba(2), 4));
            for (int k = 0; k < 3; ++k) {
                const std::string rn = "M" + std::to_string(k) + "0, M" + std::to_string(k) + "1, M" +
                                       std::to_string(k) + "2";
                w.row(rn, Csv::fx(r.calib.accelM(k, 0), 6), Csv::fx(r.calib.accelM(k, 1), 6),
                      Csv::fx(r.calib.accelM(k, 2), 6));
            }
            // K_nom 跟随比例因子单位：LSB/g ×g；LSB/Gal ×0.01
            const double kn = ua == 0 ? r.nominalAccelK * g : (ua == 1 ? r.nominalAccelK : r.nominalAccelK * 0.01);
            w.row("标称比例因子K_nom(" + std::string(kAccScaleU[ua]) + ")", Csv::fx(kn, 6), "", "");
            w.row("加速度计比例因子误差delta_s(" + std::string(kDsU[uda]) + ")",
                  Csv::sci(r.accelDeltaS(0) * kDsMul[uda], 9), Csv::sci(r.accelDeltaS(1) * kDsMul[uda], 9),
                  Csv::sci(r.accelDeltaS(2) * kDsMul[uda], 9));
            for (int k = 0; k < 3; ++k) {
                const std::string rn = "N" + std::to_string(k) + "0, N" + std::to_string(k) + "1, N" +
                                       std::to_string(k) + "2";
                w.row(rn, Csv::sci(r.accelCross(k, 0), 9), Csv::sci(r.accelCross(k, 1), 9),
                      Csv::sci(r.accelCross(k, 2), 9));
            }
            Eigen::Vector3d bg = r.calib.gyroBias;
            if (ug == 1) bg = r.gyroBiasPhys;
            else if (ug == 2) bg = r.gyroBiasPhys * (180.0 / PI) * 3600.0;
            w.row("陀螺零偏(" + std::string(kGyroBiasU[ug]) + ")", Csv::fx(bg(0), 4),
                  Csv::fx(bg(1), 4), Csv::fx(bg(2), 4));
            const Eigen::Vector3d kv = uk == 1 ? r.gyroKeff * DEG_TO_RAD : r.gyroKeff;
            w.row("陀螺有效比例因子K_eff,g(" + std::string(kGyroKeffU[uk]) + ")", Csv::sci(kv(0), 6),
                  Csv::sci(kv(1), 6), Csv::sci(kv(2), 6));
            const double ken = uk == 1 ? r.nominalKeff * DEG_TO_RAD : r.nominalKeff;
            w.row("标称有效比例因子K_eff,nom(" + std::string(kGyroKeffU[uk]) + ")", Csv::sci(ken, 6), "", "");
            const Eigen::Vector3d sv = us == 1 ? r.calib.gyroScale * DEG_TO_RAD / 3600.0 : r.calib.gyroScale;
            w.row("陀螺比例因子S_g(" + std::string(kGyroScaleU[us]) + ")", Csv::sci(sv(0), 6),
                  Csv::sci(sv(1), 6), Csv::sci(sv(2), 6));
            w.row("陀螺比例因子误差delta_s(" + std::string(kDsU[udg]) + ")",
                  Csv::sci(r.gyroDeltaS(0) * kDsMul[udg], 6), Csv::sci(r.gyroDeltaS(1) * kDsMul[udg], 6),
                  Csv::sci(r.gyroDeltaS(2) * kDsMul[udg], 6));
            w.row("dt(s)", Csv::fx(r.dt, 6));
            w.row("六位置拟合RMS(LSB)", Csv::fx(r.accelRms, 4));
            w.row("nPos(每位置历元数)", r.nPos[0], r.nPos[1], r.nPos[2], r.nPos[3], r.nPos[4], r.nPos[5]);
            w.row("nRot(每旋转历元数)", r.nRot[0], r.nRot[1], r.nRot[2], r.nRot[3], r.nRot[4], r.nRot[5]);
        });
    }

    void SixPosCalibTask::exportDetectCsv() {
        std::vector<std::pair<std::string, std::string> > seq;
        {
            std::lock_guard lk(resultMutex);
            seq = detectSeq;
        }
        if (seq.empty()) {
            setError("尚无分段结果，请先点「开始标定」");
            return;
        }
        exportCsvDialog(*this, "detect_summary.csv", "detect_summary.csv", [&](Csv::Writer &w) {
            w.row("文件", "识别到的静置/旋转段");
            for (const auto &[fst, snd]: seq) w.row(fst, snd);
        });
    }
    
    std::vector<std::string> SixPosCalibTask::compSegLabels() {
        std::lock_guard lk(resultMutex);
        std::vector<std::string> v;
        v.reserve(compSegs.pos.size() + compSegs.rot.size());
        for (const auto &[first, _]: compSegs.pos)
            v.emplace_back(std::string("静置 · ") + ImuCalibrator::posLabel(first));
        for (const auto &[first, _]: compSegs.rot)
            v.emplace_back(std::string("旋转 · ") + ImuCalibrator::rotLabel(first));
        std::map<std::string, int> seen;
        for (auto &s: v) {
            if (const int n = ++seen[s]; n > 1) s += " #" + std::to_string(n);
        }
        return v;
    }

    static std::string compPosFileName(const ImuCalibrator::Pos p) {
        static const char *tag[6] = {"xup", "xdn", "yup", "ydn", "zup", "zdn"};
        return std::string("calib_") + tag[static_cast<int>(p)] + "_comp.csv";
    }

    static std::string compRotFileName(const ImuCalibrator::Rot r) {
        static const char *tag[6] = {"rot", "xrev", "yfwd", "yrev", "zfwd", "zrev"};
        return std::string("calib_") + tag[static_cast<int>(r)] + "_comp.csv";
    }

    void SixPosCalibTask::exportCompCsv() {
        CalibReport r;
        if (!snapshotReport(r)) return;

        ImuLog seg;
        std::string label;
        std::string defName;
        bool isPos = false;
        int selIdx = -1;
        {
            std::lock_guard lk(resultMutex);
            if (const int np = static_cast<int>(compSegs.pos.size()), nr = static_cast<int>(compSegs.rot.size()); np + nr > 0) {
                const int k = std::clamp(compSel, 0, np + nr - 1);
                selIdx = k;
                if (k < np) {
                    seg = compSegs.pos[k].second;
                    isPos = true;
                    defName = compPosFileName(compSegs.pos[k].first);
                } else {
                    seg = compSegs.rot[k - np].second;
                    defName = compRotFileName(compSegs.rot[k - np].first);
                }
            }
        }
        if (seg.empty()) {
            setError("缺少该段样本，无法导出补偿序列（请重新标定）");
            return;
        }
        if (const auto segLabels = compSegLabels();
            selIdx >= 0 && selIdx < static_cast<int>(segLabels.size()))
            label = segLabels[selIdx];

        const Eigen::Matrix3d Minv = r.calib.accelM.inverse();
        const int bx = static_cast<int>(std::lround(r.calib.gyroBias(0)));
        const int by = static_cast<int>(std::lround(r.calib.gyroBias(1)));
        const int bz = static_cast<int>(std::lround(r.calib.gyroBias(2)));
        exportCsvDialog(*this, defName, defName + "（" + label + "）", [&](Csv::Writer &w) {
            if (isPos) {
                w.row("tow", "comp_fx(m/s2)", "comp_fy(m/s2)", "comp_fz(m/s2)");
                for (const auto &s: seg) {
                    const Eigen::Vector3d f = Minv * (Eigen::Vector3d(s.ax, s.ay, s.az) - r.calib.accelB);
                    w.row(Csv::fx(s.tow, 3), Csv::fx(f(0), 5), Csv::fx(f(1), 5), Csv::fx(f(2), 5));
                }
            } else {
                w.row("tow", "raw_gx", "raw_gy", "raw_gz", "comp_gx", "comp_gy", "comp_gz");
                for (const auto &s: seg)
                    w.row(Csv::fx(s.tow, 3), s.gx, s.gy, s.gz, s.gx - bx, s.gy - by, s.gz - bz);
            }
        });
    }
}
