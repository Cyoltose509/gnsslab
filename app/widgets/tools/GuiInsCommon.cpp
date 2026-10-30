#include "GuiInsCommon.h"

#include "ui/ParamPage.h"

#include <algorithm>

namespace Tools::InsPanel {
    namespace {
        using Ui::ParamPage::between;
        using Ui::ParamPage::field;
        using Ui::ParamPage::Group;



        constexpr const char *kInsPrefix = "ins_";



        const std::vector<Group<CalibParams> > &calibSchema() {
            static const std::vector<Group<CalibParams> > s = {
                {"站点 / 转台", {
                    field("当地重力加速度 g (m/s²)", "g", &CalibParams::g, between(1.0, 12.0)),
                    field("转台精确转角 α (°)", "turnAngleDeg", &CalibParams::turnAngleDeg,
                          between(1.0, 3600.0)),
                }},
            };
            return s;
        }


        const std::vector<Group<AlignParams> > &alignSchema() {
            static const std::vector<Group<AlignParams> > s = {
                {"站点", {
                    field("当地纬度 φ (°N)", "latDeg", &AlignParams::latDeg, between(-90.0, 90.0)),
                    field("当地重力加速度 g (m/s²)", "g", &AlignParams::g, between(1.0, 12.0)),
                }},
                {"解算窗口", {
                    field("滑窗长度 (历元)", "win", &AlignParams::win, between(2.0, 20000.0)),
                }},
            };
            return s;
        }
    }

    void renderInsParams(CalibParams &par) {
        Ui::ParamPage::render(calibSchema(), par, CalibParams{});
    }

    void renderInsParams(AlignParams &par) {
        Ui::ParamPage::render(alignSchema(), par, AlignParams{});
    }

    void saveInsParams(const CalibParams &par) {
        Ui::ParamPage::save(calibSchema(), par, kInsPrefix);
    }

    void saveInsParams(const AlignParams &par) {
        Ui::ParamPage::save(alignSchema(), par, kInsPrefix);
    }

    void loadInsParams(CalibParams &par) {
        Ui::ParamPage::load(calibSchema(), par, CalibParams{}, kInsPrefix);
    }

    void loadInsParams(AlignParams &par) {
        Ui::ParamPage::load(alignSchema(), par, AlignParams{}, kInsPrefix);
    }

    namespace {

        const char *alertTitle(const AlertLevel lv) {
            switch (lv) {
                case AlertLevel::Error: return "错误";
                case AlertLevel::Warn: return "注意";
                default: return "提示";
            }
        }
    }

    void drawAlert(const AlertLevel lv, const char *text) {
        if (!text || !*text) return;
        const ImVec4 c = lv == AlertLevel::Error ? kColFail : lv == AlertLevel::Warn ? kColWarn : kColInfo;
        ImGui::PushID(text);
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(c.x, c.y, c.z, 0.10f));
        ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(c.x, c.y, c.z, 0.55f));
        ImGui::PushStyleColor(ImGuiCol_Text, c);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 6.0f));
        if (ImGui::BeginChild("##alert", ImVec2(0.0f, 0.0f),
                              ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY)) {
            ImGui::TextUnformatted(alertTitle(lv));
            ImGui::SameLine(0.0f, 10.0f);
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_Text));
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextUnformatted(text);
            ImGui::PopTextWrapPos();
            ImGui::PopStyleColor();
        }
        ImGui::EndChild();
        ImGui::PopStyleVar(2);
        ImGui::PopStyleColor(3);
        ImGui::PopID();
        ImGui::Spacing();
    }


    bool ScaleResolution::observe(const RecordScaleInfo &info) {
        switch (info.kind) {
            case RecordScale::Model:
                if (!recordModel) {
                    recordModel = info.model;
                    return true;
                }
                if (info.model->type == recordModel->type) return true;
                conflict = "选中的文件混用了不同 IMU 型号（" + std::string(recordModel->name) + " 与 " +
                           info.model->name + "），当量无法统一。请按型号分两批标定。";
                return false;
            case RecordScale::UnknownType:
                if (unknownType < 0) unknownType = info.type;
                return true;
            case RecordScale::NoType:
                ++noType;
                return true;
        }
        return true;
    }

    bool ScaleResolution::resolve(ScaleFallback &sf, const std::vector<ImuLog> &staticCandidates, const double g) {
        if (recordModel) {
            scale = recordModel->scale;
            modelName = recordModel->name;
            modelType = recordModel->type;
            src = ScaleSource::Record;
            return true;
        }
        if (sf.accepted.load(std::memory_order_relaxed)) {
            adoptManual(sf.scale());
            return true;
        }
        for (const ImuLog &seg: staticCandidates)
            if (tryGuess(seg, g)) return true;
        sf.request(guessAccel, guessRate, failMessage());
        return false;
    }

    std::string ScaleResolution::failMessage() const {
        if (unknownType >= 0)
            return "记录里的 IMU Type=" + std::to_string(unknownType) + " 不在内置型号表内，无法确定解码当量。";
        if (!note.empty()) return "记录里没有 IMU 型号自描述，也无法用静置段反推当量：" + note + "。";
        return "记录里没有 IMU 型号自描述，也没有可用的静置段反推当量。";
    }

    std::string ScaleResolution::describe() const {
        switch (src) {
            case ScaleSource::Record: return modelName + "（Type=" + std::to_string(modelType) + "）";
            case ScaleSource::Guess: return modelName + "（量级反推）";
            case ScaleSource::Manual: return "用户手填当量";
            default: return {};
        }
    }

    std::string ScaleResolution::recordLabel() const {
        if (!recordModel) return {};
        return std::string(recordModel->name) + "（Type=" + std::to_string(recordModel->type) + "）";
    }

    ScaleNotice ScaleResolution::notice(const bool settled) const {
        if (!settled) {
            if (!recordModel) return {};
            return {true, AlertLevel::Info, "当量来源：记录自描述的 " + recordLabel() + "。"};
        }
        if (!known()) {
            std::string t = "未能确定解码当量，|f| 与 |ω| 的模长校验不可用";
            if (unknownType >= 0)
                t += "：记录里的 IMU Type=" + std::to_string(unknownType) + " 不在内置型号表内";
            else if (!note.empty())
                t += "：静置段反推当量失败，" + note;
            return {true, AlertLevel::Warn, t + "。"};
        }
        bool weak = src != ScaleSource::Record;
        std::string t;
        switch (src) {
            case ScaleSource::Record: t = "当量来源：记录自描述的 " + recordLabel();
                break;
            case ScaleSource::Guess: t = "当量来源：由静置段量级反推的 " + modelName;
                break;
            case ScaleSource::Manual: t = "当量来源：用户按手册手填";
                break;
            default: return {};
        }
        if (noType > 0) {
            t += "；本批有 " + std::to_string(noType) + " 个文件没有 IMU 型号自描述，已按上述当量处理。";
            weak = true;
        } else {
            t += "。";
        }
        return {true, weak ? AlertLevel::Warn : AlertLevel::Info, t};
    }

    void ScaleResolution::adoptManual(const ImuScale &s) {
        scale = s;
        modelName.clear();
        modelType = -1;
        src = ScaleSource::Manual;
    }

    bool ScaleResolution::tryGuess(const ImuLog &staticSeg, const double g) {
        if (staticSeg.empty()) return false;
        const ImuScaleGuess gs = guessImuScale(staticSeg, g);
        if (gs.model) {
            scale = gs.model->scale;
            modelName = gs.model->name;
            modelType = gs.model->type;
            src = ScaleSource::Guess;
            return true;
        }
        if (note.empty()) note = gs.note;
        if (!(guessAccel > 0.0)) guessAccel = gs.accelMpsPerLsb;
        if (!(guessRate > 0.0)) guessRate = gs.rateHz;
        return false;
    }

    void ScaleIdentifier::probeSample(const ImuSample &s) {
        if (probe.size() < kScaleProbeSamples) probe.push_back(s);
    }

    void ScaleIdentifier::offerProbe() {
        if (res.hasRecord() || probe.empty()) return;
        candidates.push_back(std::move(probe));
        probe.clear();
    }

    void ScaleIdentifier::offerStatic(const ImuLog &seg) {
        if (res.hasRecord() || seg.empty()) return;
        candidates.push_back(seg);
    }

    bool ScaleIdentifier::resolve(const double g) {
        return res.resolve(sf, candidates, g);
    }

    std::string ScaleIdentifier::failText() const {
        return res.failMessage() + "请在下方填入这台设备手册上的解码当量后重算。";
    }

    bool ScaleIdentifier::mixedModel() const {
        return !res.conflict.empty();
    }

    std::string ScaleIdentifier::openFailure(const std::string &path) const {
        return mixedModel() ? res.conflict : "无法打开 IMU 日志：" + path;
    }

    std::string ScaleIdentifier::label() const {
        return res.describe();
    }

    ImuScale ScaleIdentifier::currentScale() const {
        return res.currentScale();
    }

    ImuScale ScaleIdentifier::scale() const {
        return res.scale;
    }

    ScaleNotice ScaleIdentifier::notice(const bool settled) const {
        return res.notice(settled);
    }

    void keepManualScaleFor(ScaleFallback &sf, const std::string &sig) {
        if (sf.accepted.load(std::memory_order_relaxed) && sf.sig != sig) sf.reset();
        sf.sig = sig;
    }

    bool drawScaleFallback(ScaleFallback &sf, const bool allowSkip) {
        if (!sf.asked.load(std::memory_order_acquire)) {
            if (sf.accepted.load(std::memory_order_relaxed) &&
                ImGui::SmallButton("修改手填当量…##kf_edit")) {
                sf.accepted.store(false, std::memory_order_relaxed);
                sf.asked.store(true, std::memory_order_release);
            }
            return false;
        }
        drawAlert(AlertLevel::Warn,
                  sf.reason.empty()
                      ? "无法自动确定解码当量：记录里没有 IMU 型号，静置段量级也反推不出唯一型号。"
                      : sf.reason.c_str());
        ImGui::TextDisabled("请按这台设备手册填写解码当量，然后点「以此当量重算」。");
        ImGui::Spacing();
        auto field = [](const char *id, const char *label, const char *unit, double *v, const char *fmt) {
            ImGui::TextUnformatted(label);
            ImGui::SameLine(160.0f);
            ImGui::SetNextItemWidth(140.0f);
            ImGui::InputDouble(id, v, 0.0, 0.0, fmt);
            ImGui::SameLine();
            ImGui::TextDisabled("%s", unit);
        };
        field("##kf_a", "加计当量 k_a", "(m/s)/LSB", &sf.accelMpsPerLsb, "%.6e");
        field("##kf_g", "陀螺当量 k_g", "°/LSB", &sf.gyroDegPerLsb, "%.6e");
        field("##kf_r", "采样率", "Hz", &sf.rateHz, "%.3f");
        ImGui::Spacing();

        bool retry = false;
        ImGui::BeginDisabled(!sf.scale().known());
        if (ImGui::Button("以此当量重算", ImVec2(160, 0))) {
            sf.accepted.store(true, std::memory_order_relaxed);
            sf.asked.store(false, std::memory_order_release);
            retry = true;
        }
        ImGui::EndDisabled();
        if (allowSkip) {
            ImGui::SameLine();
            if (ImGui::Button("跳过当量校验", ImVec2(150, 0)))
                sf.asked.store(false, std::memory_order_release);
        }
        ImGui::Spacing();
        return retry;
    }


    size_t decimateStride(const size_t n, const size_t maxPts) {
        return n > maxPts ? (n + maxPts - 1) / maxPts : 1;
    }

    void decimateImu(const ImuLog &src, ImuLog &out) {
        out.clear();
        if (src.empty()) return;
        const size_t stride = decimateStride(src.size());
        out.reserve((src.size() + stride - 1) / stride);
        for (size_t i = 0; i < src.size(); i += stride) out.push_back(src[i]);
    }

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

    Eigen::Vector3d basisValue(const CalibBasis &b, const CalibReport &r, const double g) {
        Eigen::Vector3d v;
        if (b.vec) v = r.*b.vec;
        else if (b.cal) v = r.calib.*b.cal;
        else v = Eigen::Vector3d::Constant(r.*b.sca);
        return b.timesG ? v * g : v;
    }

    Eigen::Vector3d calibQtyValue(const CalibQty &q, const CalibUnit &u,
                                  const CalibReport &r, const double g) {
        const Eigen::Vector3d base = basisValue(u.usePhys ? q.phys : q.counts, r, g);
        return base * (u.perG ? u.factor / g : u.factor);
    }

    int pmDecimals(const double s) {
        if (!(s > 0.0) || !std::isfinite(s)) return 3;
        const int e = static_cast<int>(std::floor(std::log10(s)));
        return std::clamp(1 - e, 0, 6);
    }

    void drawCompChart(const char *title, const std::vector<double> &t,
                       const std::vector<double> &a, const std::vector<double> &b,
                       const std::vector<double> &c, const bool chartNew) {
        if (t.empty() || a.empty()) {
            ImGui::TextDisabled("（无数据）");
            return;
        }
        const int n = static_cast<int>(t.size());
        if (ImPlot::BeginPlot(title, ImVec2(-1, kFileChartH))) {
            ImPlot::SetupAxes("时间 t (s)", "值");
            ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
            const double xmax = t.back() - t.front();
            fitXToLatest(chartNew, xmax);
            ImPlot::PlotLine("X", t.data(), a.data(), n, lineSpec(kColX));
            ImPlot::PlotLine("Y", t.data(), b.data(), n, lineSpec(kColY));
            ImPlot::PlotLine("Z", t.data(), c.data(), n, lineSpec(kColZ));
            ImPlot::EndPlot();
        }
    }

    void drawScaleBars(const char *title, const bool ready, const double v[3], const ImVec4 &fill,
                       const char *fmt) {
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

    void vec3Row(const char *label, const double x, const double y, const double z, const char *fmt) {
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
