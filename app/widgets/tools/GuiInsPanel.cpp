#include "GuiInsPanel.h"

#include "GuiHelpers.h"
#include "implot.h"

#include "SolverTask.h"
#include "core/PathMemory.h"
#include "io/AsciiLogReader.h"
#include "io/OEM7Reader.h"
#include "ins/Aligner.h"
#include "ins/ImuCalibrator.h"
#include "core/InsMath.h"
#include "core/CoordConvert.h"
#include "Const.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <set>
#include <sstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace Tools::InsPanel {
    constexpr double SETTING_ER = 7.2921150e-5;
    constexpr double SETTING_G = 9.7936174; // m/s^2
    constexpr double SETTING_LAT_DEG = 30.531651244; // 北纬 (°)
    namespace {
        template<class T>
        struct OnlineDecimator {
            std::vector<T> buf;
            size_t stride = 1;
            size_t counter = 0;
            size_t maxPts = 4000;

            void reset() {
                buf.clear();
                stride = 1;
                counter = 0;
            }

            void push(const T &s) {
                if (counter++ % stride == 0) buf.push_back(s);
                if (buf.size() >= 2 * maxPts) {
                    size_t w = 0;
                    for (size_t i = 0; i < buf.size(); i += 2) buf[w++] = buf[i];
                    buf.resize(w);
                    stride *= 2;
                }
            }
        };

        template<class Reader, class ReadFn, class OnSample, class OnBatch>
        void pumpImu(Reader &r, ReadFn readOne, const std::atomic<bool> &stop,
                     OnSample onSample, OnBatch onBatch) {
            size_t last = 0, batchMark = 0;
            while (!stop) {
                const bool more = readOne();
                const ImuLog &L = r.getImuLog();
                for (size_t i = last; i < L.size(); ++i) onSample(L[i]);
                last = L.size();
                if (!more) break;
                if (L.size() - batchMark >= 2048) {
                    onBatch(static_cast<long long>(L.size()));
                    batchMark = L.size();
                }
            }
            onBatch(static_cast<long long>(r.getImuLog().size()));
        }

        constexpr float kChartHeight = 360.0f;
        constexpr float kFileChartH = 220.0f; // 每个文件原始波形图的高度（比大图紧凑）
        constexpr ImVec4 kColX(0.90f, 0.30f, 0.30f, 1.00f);
        constexpr ImVec4 kColY(0.30f, 0.90f, 0.30f, 1.00f);
        constexpr ImVec4 kColZ(0.30f, 0.50f, 1.00f, 1.00f);

        ImPlotSpec lineSpec(const ImVec4 &c) {
            ImPlotSpec s;
            s.LineColor = c;
            s.LineWeight = 1.5f;
            return s;
        }

        void toElapsed(const std::vector<double> &tow, std::vector<double> &out) {
            out.clear();
            if (tow.empty()) return;
            out.reserve(tow.size());
            const double t0 = tow.front();
            for (const double t: tow) out.push_back(t - t0);
        }

        void fitXToLatest(const bool newData, const double xmax) {
            if (newData && xmax > 0.0 && std::isfinite(xmax))
                ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, xmax, ImPlotCond_Always);
        }

        struct PerFileChart {
            std::string name; // 文件名（不含路径）
            std::string det; // 检测到的段描述，如 "X 上×1 Y 正转×1"
            bool dup = false; // 是否重复文件（仅展示波形，不参与标定）
            std::vector<double> t;
            std::vector<double> ax, ay, az; // 加速度计三轴原始计数
            std::vector<double> gx, gy, gz; // 陀螺仪三轴原始计数
        };

        // 某文件加速度计原始输出（三轴 counts）
        void drawAccelChart(const char *title, const PerFileChart &c, const bool chartNew) {
            if (c.t.empty()) {
                ImGui::TextDisabled("（无数据）");
                return;
            }
            const int n = static_cast<int>(c.t.size());
            if (ImPlot::BeginPlot(title, ImVec2(-1, kFileChartH))) {
                ImPlot::SetupAxes("时间 t (s)", "加速度计 (counts)");
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                fitXToLatest(chartNew, c.t.back());
                ImPlot::PlotLine("ax", c.t.data(), c.ax.data(), n, lineSpec(kColX));
                ImPlot::PlotLine("ay", c.t.data(), c.ay.data(), n, lineSpec(kColY));
                ImPlot::PlotLine("az", c.t.data(), c.az.data(), n, lineSpec(kColZ));
                ImPlot::EndPlot();
            }
        }

        // 某文件陀螺仪原始输出（三轴 counts）
        void drawGyroChart(const char *title, const PerFileChart &c, const bool chartNew) {
            if (c.t.empty()) {
                ImGui::TextDisabled("（无数据）");
                return;
            }
            const int n = static_cast<int>(c.t.size());
            if (ImPlot::BeginPlot(title, ImVec2(-1, kFileChartH))) {
                ImPlot::SetupAxes("时间 t (s)", "陀螺仪 (counts)");
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                fitXToLatest(chartNew, c.t.back());
                ImPlot::PlotLine("gx", c.t.data(), c.gx.data(), n, lineSpec(kColX));
                ImPlot::PlotLine("gy", c.t.data(), c.gy.data(), n, lineSpec(kColY));
                ImPlot::PlotLine("gz", c.t.data(), c.gz.data(), n, lineSpec(kColZ));
                ImPlot::EndPlot();
            }
        }

        // 三条 X/Y/Z 柱状图（加表比例因子或陀螺 K_eff 共用），ready=false 时画占位。
        void drawScaleBars(const char *title, const bool ready, const double v[3], const ImVec4 &fill,
                           const char *fmt = "%.1f") {
            if (ImPlot::BeginPlot(title, ImVec2(-1, kChartHeight))) {
                if (ready) {
                    ImPlot::SetupAxes("轴", "值", ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit);
                } else {
                    ImPlot::SetupAxes("轴", "值", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
                    ImPlot::SetupAxisLimits(ImAxis_Y1, -1.0, 1.0, ImPlotCond_Always);
                }
                ImPlot::SetupAxisLimits(ImAxis_X1, -0.6, 2.6, ImPlotCond_Always);
                constexpr double xs[3] = {0, 1, 2};
                const char *xl[3] = {"X", "Y", "Z"};
                ImPlot::SetupAxisTicks(ImAxis_X1, xs, 3, xl, false);
                if (ready) {
                    ImPlotSpec bs;
                    bs.FillColor = fill;
                    bs.LineColor = ImVec4(0.55f, 0.75f, 1.00f, 1.0f);
                    bs.LineWeight = 1.0f;
                    ImPlot::PlotBars("值", xs, v, 3, 0.5, bs);
                    for (int k = 0; k < 3; ++k) {
                        char buf[32];
                        std::snprintf(buf, sizeof(buf), fmt, v[k]);
                        ImPlot::PlotText(buf, xs[k], v[k]);
                    }
                } else {
                    ImPlot::PlotText("计算中", 1.0, 0.0);
                }
                ImPlot::EndPlot();
            }
        }

        // ImGui 两列并排：分别画左右两张图（或占位）。
        template<class F1, class F2>
        void twoCol(const F1 &left, const F2 &right) {
            ImGui::Columns(2, nullptr, false);
            left();
            ImGui::NextColumn();
            right();
            ImGui::Columns(1);
        }

        void vec3Row(const char *label, const double x, const double y, const double z, const char *fmt = "%.3f") {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(label);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text(fmt, x);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text(fmt, y);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text(fmt, z);
        }

        // 稳态(后 1/3)均值与 1σ
        void steadyStats(const std::vector<AttitudeSample> &s,
                         double &ym, double &ysd, double &pm, double &psd, double &rm, double &rsd) {
            ym = ysd = pm = psd = rm = rsd = 0.0;
            if (s.empty()) return;
            const size_t lo = s.size() * 2 / 3;
            double ny = 0, np = 0, nr = 0;
            for (size_t i = lo; i < s.size(); ++i) {
                ny += s[i].yaw;
                np += s[i].pitch;
                nr += s[i].roll;
            }
            const auto m = static_cast<double>(s.size() - lo);
            if (m <= 0) return;
            ny /= m;
            np /= m;
            nr /= m;
            double vy = 0, vp = 0, vr = 0;
            for (size_t i = lo; i < s.size(); ++i) {
                vy += (s[i].yaw - ny) * (s[i].yaw - ny);
                vp += (s[i].pitch - np) * (s[i].pitch - np);
                vr += (s[i].roll - nr) * (s[i].roll - nr);
            }
            ym = ny * RAD_TO_DEG;
            ysd = std::sqrt(vy / m) * RAD_TO_DEG;
            pm = np * RAD_TO_DEG;
            psd = std::sqrt(vp / m) * RAD_TO_DEG;
            rm = nr * RAD_TO_DEG;
            rsd = std::sqrt(vr / m) * RAD_TO_DEG;
        }
    }

    struct SixPosCalibTask : GnssTask::SolveTask {
        std::vector<std::string> filePaths; // 选中的文件（任意数量，自动检测）
        double g = SETTING_G; // 当地重力加速度 (m/s^2)
        double latDeg = SETTING_LAT_DEG; // 当地纬度 (°N)
        double earthRate = SETTING_ER; // 地球自转角速率 (rad/s)
        double turnAngleDeg = 360.0; // 转台精确转角 (°)

        std::mutex resultMutex;
        CalibReport report; // 终算报告（全部标定完成后可用）
        std::atomic<bool> hasResult{false}; // 全部标定完成（含陀螺 K_eff）
        std::atomic<bool> accelReady{false}; // 六位置解完：加表模型 + 6 张补偿比力图可用

        // 流式状态：进度语义同 GNSS，只报"已处理样本数"计数（reader 无 progress 接口）。
        std::atomic<long long> sampleCount{0};
        std::atomic<int> segStatic{0}; // 已识别静置段数
        std::atomic<int> segRot{0}; // 已识别旋转段数
        std::atomic<int> dupSkipped{0}; // 跳过重复文件数
        std::string currentFile; // 当前正在解析的文件名（进度提示用）

        // 检测摘要（Running/Done 都展示）：每文件一行"检测到什么"
        std::vector<std::string> detectLines;
        std::atomic<bool> detectDone{false};

        // 结果图表（横轴均为相对首样本的秒数，抽稀到 ≤4k）：每个文件一张加速表 + 一张陀螺图
        std::vector<PerFileChart> fileCharts;
        // 加表中间量（accelReady 后可用）：用于实时填表 + 画补偿比力图
        Eigen::Matrix3d accelMp{Eigen::Matrix3d::Identity()};
        Eigen::Vector3d accelBp{Eigen::Vector3d::Zero()};
        Eigen::Vector3d accelScaleGp{Eigen::Vector3d::Zero()};
        double accelRmsp = 0.0;
        std::atomic<bool> chartDirty{true};

        SixPosCalibTask() {
            mode = GnssTask::SolverMode::Ins;
            fileName = "六位置法标定";
        }

        void renderConfigPanel() override;

        void renderResult();

    private:
        void startCalib();

        void calibWorker();
    };

    void SixPosCalibTask::startCalib() {
        {
            // 实验室常量无条件持久化（校验失败也落盘，同四解算器铁律）
            const auto &cfg = AppConfig::PathMemory::instance();
            cfg.setNum("ins_g", g);
            cfg.setNum("ins_lat", latDeg);
            cfg.setNum("ins_we", earthRate);
            cfg.setNum("ins_turn", turnAngleDeg);
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
        ImuCalibrator::CalibSegments segs;
        std::vector<std::string> detLines;
        int dups = 0;

        // 流式解析单个文件：边读边把原始计数推入抽稀器发布，读尽返回。
        auto parseOne = [&](const std::string &path, ImuLog &out) -> bool {
            out.clear();
            auto onSample = [&](const ImuSample &s) { out.push_back(s); };
            auto onBatch = [&](const long long n) { sampleCount.store(n); };
            if (endsWithIC(path, ".asc")) {
                AsciiLogReader r;
                if (!r.open(path)) return false;
                ImuSample t;
                pumpImu(r, [&] { return r.getNextImu(t); }, stop, onSample, onBatch);
            } else {
                OEM7Reader r;
                if (!r.open(path)) return false;
                ImuSample t;
                pumpImu(r, [&] { return r.getNextImu(t); }, stop, onSample, onBatch);
            }
            return !out.empty();
        };

        std::set<std::string> seenHash;
        int nStatic = 0, nRot = 0;
        std::vector<PerFileChart> fcs; // 逐文件的原始波形（任意数量文件）

        // 把一个 ImuLog 抽稀成 PerFileChart（横轴为相对首样本秒数，抽稀到 ≤4000 点）
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
                if (!stop) detLines.push_back(name + " : 无法解析或为空");
                continue;
            }
            // 每个文件都构建原始输出波形（无论是否重复，都给出加速度计 + 陀螺仪两张图）
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
                fcs.push_back(std::move(fc));
                continue;
            }
            seenHash.insert(hash);

            // 分段 + 分类
            auto segsFile = ImuCalibrator::detect(log);
            int cPos[6] = {0, 0, 0, 0, 0, 0}, cRot[6] = {0, 0, 0, 0, 0, 0};
            for (const auto &s: segsFile) {
                if (s.isStatic) {
                    cPos[static_cast<int>(s.pos)]++;
                    ++nStatic;
                } else {
                    cRot[static_cast<int>(s.rot)]++;
                    ++nRot;
                }
            }
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

            // 抽取子段并入聚合集合
            for (const auto &[isStatic, pos, rot, i0, i1]: segsFile) {
                ImuLog sub(log.begin() + i0, log.begin() + i1);
                if (isStatic) segs.pos.emplace_back(pos, std::move(sub));
                else segs.rot.emplace_back(rot, std::move(sub));
            }
            fcs.push_back(std::move(fc));
        }

        segStatic.store(nStatic);
        segRot.store(nRot);
        dupSkipped.store(dups);
        {
            std::lock_guard lk(resultMutex);
            detectLines = detLines;
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

        // 标定（覆盖不足会在 report.ok=false 中标明，并在 errorMsg 给出缺项）
        CalibReport rep = ImuCalibrator::calibrate(segs, turnAngleDeg * DEG_TO_RAD, g);
        if (!rep.ok) {
            setError(rep.errorMsg);
            loading = false;
            state = State::Done;
            return;
        }

        // 逐文件原始波形已在解析阶段构建到 fcs，此处直接落盘
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
                ImGui::Text("添加任意数量的 IMU 日志文件，程序自动检测每个文件包含：");
                ImGui::Text("  静置段（6 位置之一：X/Y/Z 上/下）与旋转段（6 旋转之一：X/Y/Z 正/逆转）。");
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
                        if (ImGui::SmallButton(("✕##cal" + std::to_string(i)).c_str())) {
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
                ImGui::InputDouble("地球自转率 ωe (rad/s)", &earthRate, 0.0, 0.0, "%.8e");
                ImGui::InputDouble("转台精确转角 α (°)", &turnAngleDeg, 0.0, 0.0, "%.3f");
                ImGui::PopItemWidth();
                if (ImGui::Button("恢复实验室默认值")) {
                    g = SETTING_G;
                    latDeg = SETTING_LAT_DEG;
                    earthRate = SETTING_ER;
                    turnAngleDeg = 360.0;
                }
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

    void SixPosCalibTask::renderResult() {
        const bool running = state == State::Running;
        if (errorFlag()) {
            ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "错误: %s", errorMessage().c_str());
        }
        {
            std::lock_guard lk(resultMutex);
            if (detectDone.load()) {
                ImGui::Spacing();
                ImGui::Text("检测到的内容：");
                for (const auto &l: detectLines) ImGui::Text("  %s", l.c_str());
                ImGui::Text("已识别 静置段 %d / 旋转段 %d，跳过重复 %d，已处理历元 %lld",
                            segStatic.load(), segRot.load(), dupSkipped.load(),
                            static_cast<long long>(sampleCount.load()));
            }
        }
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

        // 取一份快照（Running / Done 都展示，未算出处写"计算中"）
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

        if (ImGui::BeginTable("##cal_tbl", 4,
                              ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableSetupColumn("量");
            ImGui::TableSetupColumn("X");
            ImGui::TableSetupColumn("Y");
            ImGui::TableSetupColumn("Z");
            ImGui::TableHeadersRow();
            auto cell = [&](const char *label, const bool ready, const double x, const double y, const double z, const char *fmt) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(label);
                for (int c = 0; c < 3; ++c) {
                    ImGui::TableSetColumnIndex(1 + c);
                    if (ready) {
                        char b[48];
                        std::snprintf(b, sizeof(b), fmt, c == 0 ? x : c == 1 ? y : z);
                        ImGui::TextUnformatted(b);
                    } else {
                        ImGui::TextDisabled("计算中");
                    }
                }
            };
            cell("比例因子 (counts/g)", aReady, accS(0), accS(1), accS(2), "%.1f");
            // 比例因子 1σ：accelScaleSigma 单位是 counts/(m/s²)，表格按 counts/g 显示（×g）。
            // 注意六位置法每轴 dof=2，σ 本身波动大（2 自由度卡方的相对标准差约 70%），
            // 仅作数量级参考：|三轴比例因子差| ≲ σ 说明轴间差异在噪声内。
            const Eigen::Vector3d accSigG = rep.accelScaleSigma * g;
            cell("比例因子 ±1σ (counts/g)", aReady, accSigG(0), accSigG(1), accSigG(2), "%.2f");
            cell("加表零偏 (counts)", aReady, rep.calib.accelB(0), rep.calib.accelB(1), rep.calib.accelB(2), "%.2f");
            cell("陀螺 K_eff (counts/rad)", hResult, rep.gyroKeff(0), rep.gyroKeff(1), rep.gyroKeff(2), "%.4e");
            cell("陀螺比例因子 (counts/(rad/s))", hResult, rep.calib.gyroScale(0), rep.calib.gyroScale(1),
                 rep.calib.gyroScale(2), "%.4e");
            cell("陀螺零偏 (counts/历元)", hResult, rep.calib.gyroBias(0), rep.calib.gyroBias(1), rep.calib.gyroBias(2),
                 "%.3f");
            cell("陀螺比例因子误差 δs", hResult, rep.gyroDeltaS(0), rep.gyroDeltaS(1), rep.gyroDeltaS(2), "%.3e");
            ImGui::EndTable();
        }

        // 质量判读（随结果分阶段点亮）
        if (aReady) {
            const double meanG = accS.mean();
            const double spreadG = accS.maxCoeff() - accS.minCoeff();
            const bool okG = meanG > 1.0 && spreadG / meanG < 0.05;
            const bool okRms = arm < 0.02 * meanG;
            ImGui::Spacing();
            ImGui::Text("dt = %.6f s   accelRMS = %.3f counts   meanG = %.1f", rep.dt, arm, meanG);
            ImGui::Text("三轴比例因子一致性 (spread/mean<5%%): %s   六位置残差 (<2%%g): %s", okG ? "PASS" : "FAIL",
                        okRms ? "PASS" : "FAIL");
        }
        if (hResult) {
            const double maxds = rep.gyroDeltaS.cwiseAbs().maxCoeff();
            ImGui::Text("陀螺 δs (<5%%): %s", maxds < 0.05 ? "PASS" : "FAIL");
        }

        // ---- 比例因子一致性：加表比例因子 + 陀螺 K_eff 两图并排 ----
        ImGui::Spacing();
        ImGui::Text("比例因子一致性（两图并排）");
        const double scA[3] = {accS(0), accS(1), accS(2)};
        const double scG[3] = {rep.gyroKeff(0), rep.gyroKeff(1), rep.gyroKeff(2)};
        twoCol([&] { drawScaleBars("加速度计三轴比例因子 (counts/g)", aReady, scA, kColZ, "%.1f"); },
               [&] { drawScaleBars("陀螺 K_eff (counts/rad)", hResult, scG, kColZ, "%.4e"); });

        // ---- 每个文件的原始输出：加速度计 + 陀螺仪 两张图并排 ----
        ImGui::Spacing();
        ImGui::Text("各文件原始输出（加速度计 / 陀螺仪，三轴原始计数）");
        for (const auto &fc: fcSnap) {
            ImGui::Spacing();
            ImGui::Text("%s   [%s]%s", fc.name.c_str(), fc.det.c_str(),
                        fc.dup ? "（重复，未计入标定）" : "");
            twoCol([&] { drawAccelChart((fc.name + " · 加速度计").c_str(), fc, chartNew); },
                   [&] { drawGyroChart((fc.name + " · 陀螺仪").c_str(), fc, chartNew); });
        }

        ImGui::Spacing();
        if (ImGui::Button("导出 CSV", ImVec2(140, 0))) {
            std::wstring out;
            if (Solvers::Helpers::saveCSVDialog("六位置法标定_result.csv", out)) {
                FILE *fp = nullptr;
                if (_wfopen_s(&fp, out.c_str(), L"w, ccs=UTF-8") == 0 && fp) {
                    fprintf(fp, "量,X,Y,Z\n");
                    fprintf(fp, "比例因子(counts/g),%.1f,%.1f,%.1f\n", accS(0), accS(1), accS(2));
                    fprintf(fp, "加表零偏(counts),%.2f,%.2f,%.2f\n", rep.calib.accelB(0), rep.calib.accelB(1), rep.calib.accelB(2));
                    fprintf(fp, "陀螺K_eff(counts/rad),%.6e,%.6e,%.6e\n", rep.gyroKeff(0), rep.gyroKeff(1), rep.gyroKeff(2));
                    fprintf(fp, "陀螺比例因子(counts/rad/s),%.6e,%.6e,%.6e\n", rep.calib.gyroScale(0), rep.calib.gyroScale(1),
                            rep.calib.gyroScale(2));
                    fprintf(fp, "陀螺零偏(counts/历元),%.3f,%.3f,%.3f\n", rep.calib.gyroBias(0), rep.calib.gyroBias(1), rep.calib.gyroBias(2));
                    fprintf(fp, "陀螺比例因子误差δs,%.3e,%.3e,%.3e\n", rep.gyroDeltaS(0), rep.gyroDeltaS(1), rep.gyroDeltaS(2));
                    fprintf(fp, "dt(s),%.6f\n", rep.dt);
                    fprintf(fp, "accelRMS(counts),%.3f\n", arm);
                    fprintf(fp, "meanG,%.1f\n", accS.mean());
                    fclose(fp);
                }
            }
        }
    }

    struct StaticAlignTask : GnssTask::SolveTask {
        std::string alignPath;
        double g = SETTING_G;
        double latDeg = SETTING_LAT_DEG;
        double earthRate = SETTING_ER;
        bool useGyroBias = false;
        double gyroBias[3] = {0, 0, 0}; // counts / 样本
        int win = 400; // 滑窗长度（样本）

        std::mutex resultMutex;
        Attitude att{};
        std::vector<AttitudeSample> series; // 完整逐历元序列（Done 后用于 CSV / 稳态统计）
        std::vector<AttitudeSample> plotPts; // 抽稀绘图缓冲（流式期间每帧只拷这份）
        std::atomic<bool> hasResult{false};

        // 流式状态：只报"已处理样本数"计数（同 GNSS：solvedCount 计数式，reader 无 progress 接口）
        std::atomic<long long> sampleCount{0};
        // "图表数据已更新"一次性标志（同 GNSS 的 newed）：渲染侧据此把 X 轴拉到最新时刻后清零。
        std::atomic<bool> chartDirty{true};

        // 稳态统计（Done 时算一次，避免每帧整段拷贝）
        double steadyY = 0, steadyYsd = 0, steadyP = 0, steadyPsd = 0, steadyR = 0, steadyRsd = 0;

        // 对准质量诊断（重力残差/自转残差/机体三轴 NED 指向），流式阶段随累计均值更新
        AlignQuality alignQ;
        bool alignQValid = false;

        StaticAlignTask() {
            mode = GnssTask::SolverMode::Ins;
            fileName = "静态解析粗对准";
        }

        void renderConfigPanel() override;

        void renderResult();

    private:
        void startAlign();

        void alignWorker();
    };

    void StaticAlignTask::startAlign() {
        {
            // 站点/滤波参数无条件持久化（校验失败也落盘；gyroBias 空串=清除，符合 PathMemory 语义）
            const auto &cfg = AppConfig::PathMemory::instance();
            cfg.setNum("ins_g", g);
            cfg.setNum("ins_lat", latDeg);
            cfg.setNum("ins_we", earthRate);
            cfg.set("ins_win", std::to_string(win));
            cfg.set("ins_align_bias", useGyroBias
                                          ? fmtG(gyroBias[0]) + "," + fmtG(gyroBias[1]) + "," + fmtG(gyroBias[2])
                                          : std::string());
        }
        if (alignPath.empty()) {
            setError("请先选择对准段日志文件 (ASCII .ASC)");
            return;
        }
        if (g <= 0.0) {
            setError("重力加速度 g 必须为正数");
            return;
        }
        state = State::Running;
        loading = true;
        if (worker.joinable()) worker.join();
        stop = false;
        hasResult.store(false);
        sampleCount.store(0);
        chartDirty.store(false);
        {
            std::lock_guard lk(resultMutex);
            series.clear();
            plotPts.clear();
        }
        std::shared_ptr<SolverTaskBase> self = shared_from_this();
        worker = std::thread([self] { dynamic_cast<StaticAlignTask *>(self.get())->alignWorker(); });
    }

    void StaticAlignTask::alignWorker() {
        SiteParams sp;
        sp.g = g;
        sp.latDeg = latDeg;
        sp.earthRate = earthRate;
        const Eigen::Vector3d gb = useGyroBias
                                       ? Eigen::Vector3d(gyroBias[0], gyroBias[1], gyroBias[2])
                                       : Eigen::Vector3d::Zero();

        Aligner::AlignStream st;
        st.init(win, sp, gb);

        // 整段均值用累计和（与 Aligner::solve 的 mean 口径一致，无需另存原始 log）。
        Eigen::Vector3d sumAcc = Eigen::Vector3d::Zero(), sumGyr = Eigen::Vector3d::Zero();
        long long count = 0;
        std::vector<AttitudeSample> seriesLocal;
        seriesLocal.reserve(1 << 16);
        OnlineDecimator<AttitudeSample> dec;

        auto onSample = [&](const ImuSample &s) {
            AttitudeSample a;
            st.step(s, a);
            sumAcc += Eigen::Vector3d(s.ax, s.ay, s.az);
            sumGyr += Eigen::Vector3d(s.gx, s.gy, s.gz);
            ++count;
            // 滑窗未满（前 win 个样本）时窗口均值仍含单样本 TRIAD 野值，会令曲线首点跳远；
            // 跳过这段暖机，使曲线起点即为满窗稳定估计（整段均值表仍用全部样本，不受影响）。
            if (st.count < static_cast<size_t>(win)) return;
            seriesLocal.push_back(a);
            dec.push(a);
        };
        auto onBatch = [&](const long long n) {
            sampleCount.store(n);
            std::lock_guard lk(resultMutex);
            plotPts = dec.buf;
            chartDirty.store(true); // 新增样本：让曲线横轴跟到最新时刻
            // 实时整段均值：随样本累积更新 att，使结果表的"整段均值"行在流式阶段就动态变化
            // （本 lambda 开头已持 resultMutex，下面直接写成员）
            if (count > 0) {
                const Eigen::Vector3d accm = sumAcc / static_cast<double>(count);
                const Eigen::Vector3d gyrm = sumGyr / static_cast<double>(count);
                const Eigen::Matrix3d C = alignDCM(-accm, gyrm - gb, sp);
                DCMToEuler(C, att.roll, att.pitch, att.yaw);
                alignQ = Aligner::diagnoseMeans(accm, gyrm, gb, sp);
                alignQValid = true;
            }
        };

        if (endsWithIC(alignPath, ".asc")) {
            AsciiLogReader r;
            if (!r.open(alignPath)) {
                setError("无法打开 ASCII 日志: " + alignPath);
                loading = false;
                state = State::Done;
                return;
            }
            ImuSample t;
            pumpImu(r, [&] { return r.getNextImu(t); }, stop, onSample, onBatch);
        } else {
            OEM7Reader r;
            if (!r.open(alignPath)) {
                setError("无法打开 OEM7 二进制日志: " + alignPath);
                loading = false;
                state = State::Done;
                return;
            }
            // 逐帧 IMU 接口：纯 IMU 的 .DAT（无 GNSS 历元）用 getNextEpoch 会在一次调用内
            // 扫完全部帧后返回 false，曲线/样本计数全程无输出；getNextImu 每帧产一个样本。
            ImuSample t;
            pumpImu(r, [&] { return r.getNextImu(t); }, stop, onSample, onBatch);
        }

        if (stop) {
            loading = false;
            state = State::Done;
            return;
        }
        if (seriesLocal.empty()) {
            setError("未在该文件中解析到 IMU 历元: " + alignPath);
            loading = false;
            state = State::Done;
            return;
        }

        // 整段对准一次 TRIAD（与 Aligner::solve 逐位一致）。
        Attitude a{};
        AlignQuality aq;
        {
            const Eigen::Vector3d acc = sumAcc / static_cast<double>(count);
            const Eigen::Vector3d gyr = sumGyr / static_cast<double>(count);
            const Eigen::Matrix3d C = alignDCM(-acc, gyr - gb, sp);
            DCMToEuler(C, a.roll, a.pitch, a.yaw);
            aq = Aligner::diagnoseMeans(acc, gyr, gb, sp);
        }
        double ym, ysd, pm, psd, rm, rsd;
        steadyStats(seriesLocal, ym, ysd, pm, psd, rm, rsd);

        {
            std::lock_guard lk(resultMutex);
            att = a;
            alignQ = aq;
            alignQValid = true;
            series = std::move(seriesLocal);
            plotPts = dec.buf;
            steadyY = ym;
            steadyYsd = ysd;
            steadyP = pm;
            steadyPsd = psd;
            steadyR = rm;
            steadyRsd = rsd;
        }
        chartDirty.store(true);
        hasResult.store(true);
        loading = false;
        state = State::Done;
    }

    void StaticAlignTask::renderConfigPanel() {
        Solvers::Helpers::beginConfigBody("静态解析粗对准配置###cfg_", *this);

        if (ImGui::BeginTabBar("##cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                ImGui::TextDisabled("当前站点常数：g = %.6f m/s²，φ = %.6f °N，ωe = %.6e rad/s"
                                    "（在\"参数\"页填写，自动记忆）",
                                    g, latDeg, earthRate);
                Solvers::Helpers::fileRow("对准段日志 (ASCII .ASC)", &alignPath, "ins_align", Solvers::Helpers::fIns,
                                    std::size(Solvers::Helpers::fIns), nullptr);
                ImGui::TextDisabled("对准段与标定段为不同时段/不同格式；本工具直接用原始计数（TRIAD 只需方向）。");
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("参数")) {
                ImGui::Text("站点参数（默认取实验室指导书常数，改动自动记忆到 gnsslab.ini）：");
                ImGui::PushItemWidth(260);
                ImGui::InputDouble("当地重力加速度 g (m/s²)", &g, 0.0, 0.0, "%.6f");
                ImGui::InputDouble("当地纬度 φ (°N)", &latDeg, 0.0, 0.0, "%.6f");
                ImGui::InputDouble("地球自转率 ωe (rad/s)", &earthRate, 0.0, 0.0, "%.8e");
                ImGui::InputInt("滑窗长度 (历元)", &win);
                ImGui::PopItemWidth();
                ImGui::Separator();
                if (ImGui::Checkbox("扣除陀螺零偏 (counts/历元)", &useGyroBias)) {
                }
                if (useGyroBias) {
                    ImGui::PushItemWidth(120);
                    ImGui::InputDouble("bias X", &gyroBias[0], 0, 0, "%.3f");
                    ImGui::SameLine();
                    ImGui::InputDouble("bias Y", &gyroBias[1], 0, 0, "%.3f");
                    ImGui::SameLine();
                    ImGui::InputDouble("bias Z", &gyroBias[2], 0, 0, "%.3f");
                    ImGui::PopItemWidth();
                }
                if (ImGui::Button("恢复实验室默认值")) {
                    g = SETTING_G;
                    latDeg = SETTING_LAT_DEG;
                    earthRate = SETTING_ER;
                    win = 400;
                }
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        ImGui::Spacing();
        if (ImGui::Button("开始对准", ImVec2(200, 40))) startAlign();
        ImGui::SameLine();
        Solvers::Helpers::renderCancelButton(this);
        if (errorFlag() && !hasResult)
            ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "%s", errorMessage().c_str());

        Solvers::Helpers::endConfigBody();
    }

    void StaticAlignTask::renderResult() {
        if (errorFlag()) {
            ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "错误: %s", errorMessage().c_str());
            return;
        }
        const bool running = state == State::Running;
        const bool done = hasResult.load();

        if (running) {
            ImGui::TextColored(ImVec4(0.6f, 0.6f, 1.0f, 1.0f),
                               "正在对准 [%s]：已解析 %lld 个历元", fileName.c_str(),
                               static_cast<long long>(sampleCount.load()));
        }

        {
            Attitude m;
            {
                std::lock_guard lk(resultMutex);
                m = att;
            }
            if (ImGui::BeginTable("##align_tbl", 4,
                                  ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableSetupColumn("姿态 (deg)");
                ImGui::TableSetupColumn("yaw");
                ImGui::TableSetupColumn("pitch");
                ImGui::TableSetupColumn("roll");
                ImGui::TableHeadersRow();
                vec3Row("整段均值", m.yaw * RAD_TO_DEG, m.pitch * RAD_TO_DEG, m.roll * RAD_TO_DEG, "%.4f");
                if (done) {
                    vec3Row("稳态(后1/3)均值", steadyY, steadyP, steadyR, "%.4f");
                    vec3Row("稳态 1σ", steadyYsd, steadyPsd, steadyRsd, "%.4f");
                } else {
                    for (int r = 0; r < 2; ++r) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextUnformatted(r == 0 ? "稳态(后1/3)均值" : "稳态 1σ");
                        for (int c = 0; c < 3; ++c) {
                            ImGui::TableSetColumnIndex(1 + c);
                            ImGui::TextDisabled("计算中");
                        }
                    }
                }
                ImGui::EndTable();
            }
        }
        ImGui::Spacing();

        // ===== 对准质量诊断表（残差大 ⇒ 该段并非真正静置，或陀螺零偏未扣）=====
        {
            AlignQuality aq;
            bool valid = false;
            {
                std::lock_guard lk(resultMutex);
                aq = alignQ;
                valid = alignQValid;
            }
            if (valid && ImGui::BeginTable("##alignq_tbl", 4,
                                           ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                           ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableSetupColumn("对准质量");
                ImGui::TableSetupColumn("数值 / 北");
                ImGui::TableSetupColumn("东");
                ImGui::TableSetupColumn("地");
                ImGui::TableHeadersRow();
                auto qRow = [](const char *label, const double v, const char *fmt) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(label);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text(fmt, v);
                };
                qRow("重力方向残差 (°)", aq.gravityErrDeg, "%.4f");
                qRow("地球自转残差 (rad/s)", aq.earthRateErrRad, "%.3e");
                auto axisRow = [](const char *label, const Eigen::Vector3d &v) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(label);
                    for (int c = 0; c < 3; ++c) {
                        ImGui::TableSetColumnIndex(1 + c);
                        ImGui::Text("%.4f", v(c));
                    }
                };
                axisRow("机体 X 轴 → NED", aq.axisX);
                axisRow("机体 Y 轴 → NED", aq.axisY);
                axisRow("机体 Z 轴 → NED", aq.axisZ);
                ImGui::EndTable();
            }
        }
        ImGui::Spacing();

        // 抽稀缓冲 → 三条曲线（yaw/pitch/roll 各自相对首样本的历时）
        std::vector<AttitudeSample> pts;
        {
            std::lock_guard lk(resultMutex);
            pts = plotPts; // 抽稀缓冲 ≤~4k
        }
        std::vector<double> px, py, pp, pr;
        if (!pts.empty()) {
            px.reserve(pts.size());
            py.reserve(pts.size());
            pp.reserve(pts.size());
            pr.reserve(pts.size());
            for (const auto &[tow, roll, pitch, yaw]: pts) {
                px.push_back(tow);
                py.push_back(yaw * RAD_TO_DEG);
                pp.push_back(pitch * RAD_TO_DEG);
                pr.push_back(roll * RAD_TO_DEG);
            }
        }
        std::vector<double> pt;
        toElapsed(px, pt); // 横轴：相对首样本的历时（绝对 tow≈1.47e9 会把刻度挤成一串同值）
        const bool chartNew = chartDirty.exchange(false);

        // 三个姿态角各画一张图（不画到一起）
        auto drawAtt = [&](const char *title, const std::vector<double> &y, const ImVec4 &col) {
            if (pt.empty()) {
                ImGui::TextDisabled("（无数据）");
                return;
            }
            const int n = static_cast<int>(pt.size());
            if (ImPlot::BeginPlot(title, ImVec2(-1, kChartHeight))) {
                ImPlot::SetupAxes("时间 t (s)", "姿态角 (deg)");
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                fitXToLatest(chartNew, pt.back());
                ImPlot::PlotLine("角度", pt.data(), y.data(), n, lineSpec(col));
                ImPlot::EndPlot();
            }
        };
        drawAtt("yaw 随时间", py, kColX);
        drawAtt("pitch 随时间", pp, kColY);
        drawAtt("roll 随时间", pr, kColZ);

        if (running) {
            if (ImGui::Button("停止")) stop = true;
            ImGui::Spacing();
        }

        // 注：结果表已上移至本函数开头（一开始就出现，整段均值随计算实时刷新）。
        ImGui::Spacing();
        if (ImGui::Button("导出 CSV", ImVec2(140, 0))) {
            std::wstring out;
            if (Solvers::Helpers::saveCSVDialog("静态解析粗对准_result.csv", out)) {
                std::vector<AttitudeSample> s;
                {
                    std::lock_guard lk(resultMutex);
                    s = series; // 完整分辨率，仅点击时拷一次
                }
                FILE *fp = nullptr;
                if (_wfopen_s(&fp, out.c_str(), L"w, ccs=UTF-8") == 0 && fp) {
                    fprintf(fp, "tow,yaw(deg),pitch(deg),roll(deg)\n");
                    for (const auto &[tow, roll, pitch, yaw]: s)
                        fprintf(fp, "%.3f,%.6f,%.6f,%.6f\n", tow, yaw * RAD_TO_DEG, pitch * RAD_TO_DEG, roll * RAD_TO_DEG);
                    fclose(fp);
                }
            }
        }
    }

    std::shared_ptr<GnssTask::SolverTaskBase> MakeSixPosCalibTask() {
        auto t = std::make_shared<SixPosCalibTask>();
        const auto &pm = AppConfig::PathMemory::instance();
        // 文件列表持久化（单一键，'|' 分隔）
        if (const std::string files = pm.get("ins_cal_files"); !files.empty()) {
            size_t pos = 0;
            while (pos < files.size()) {
                size_t nxt = files.find('|', pos);
                if (nxt == std::string::npos) nxt = files.size();
                if (std::string p = files.substr(pos, nxt - pos); !p.empty()) t->filePaths.push_back(p);
                pos = nxt + 1;
            }
        }
        t->g = pm.getNum("ins_g", SETTING_G, 1.0, 12.0);
        t->latDeg = pm.getNum("ins_lat", SETTING_LAT_DEG, -90.0, 90.0);
        t->earthRate = pm.getNum("ins_we", SETTING_ER, 1e-7, 1e-3);
        t->turnAngleDeg = pm.getNum("ins_turn", 360.0, 1.0, 3600.0);
        return t;
    }

    std::shared_ptr<GnssTask::SolverTaskBase> MakeStaticAlignTask() {
        auto t = std::make_shared<StaticAlignTask>();
        t->alignPath = AppConfig::PathMemory::instance().get("ins_align");
        t->g = AppConfig::PathMemory::instance().getNum("ins_g", SETTING_G, 1.0, 12.0);
        t->latDeg = AppConfig::PathMemory::instance().getNum("ins_lat", SETTING_LAT_DEG, -90.0, 90.0);
        t->earthRate = AppConfig::PathMemory::instance().getNum("ins_we", SETTING_ER, 1e-7, 1e-3);
        t->win = AppConfig::PathMemory::instance().getInt("ins_win", 400, 2, 20000);
        if (const std::string bs = AppConfig::PathMemory::instance().get("ins_align_bias"); !bs.empty()) {
            std::istringstream ss(bs);
            std::string tok;
            double v[3] = {0, 0, 0};
            int n = 0;
            while (n < 3 && std::getline(ss, tok, ',')) {
                try {
                    size_t pos = 0;
                    const double d = std::stod(tok, &pos);
                    if (pos != tok.size()) break;
                    v[n++] = d;
                } catch (...) {
                    break;
                }
            }
            if (n == 3) {
                t->useGyroBias = true;
                t->gyroBias[0] = v[0];
                t->gyroBias[1] = v[1];
                t->gyroBias[2] = v[2];
            }
        }
        return t;
    }

    bool RenderInsResult(const std::shared_ptr<GnssTask::SolverTaskBase> &job) {
        if (auto *c = dynamic_cast<SixPosCalibTask *>(job.get())) {
            c->renderResult();
            return true;
        }
        if (auto *a = dynamic_cast<StaticAlignTask *>(job.get())) {
            a->renderResult();
            return true;
        }
        return false;
    }
}
