#include "GuiInsCommon.h"

#include "ui/ParamPage.h"

#include <algorithm>
#include "i18n/I18n.h"

namespace Tools::Ins {
    namespace {
        using UI::ParamPage::between;
        using UI::ParamPage::field;
        using UI::ParamPage::Group;



        constexpr auto kInsPrefix = "ins_";



        const std::vector<Group<CalibParams> > &calibSchema() {
            static const std::vector<Group<CalibParams> > s = {
                {"ins.param.group.station_turntable", {
                    field("ins.param.g", "g", &CalibParams::g, between(1.0, 12.0)),
                    field("ins.param.turn_angle", "turnAngleDeg", &CalibParams::turnAngleDeg,
                          between(1.0, 3600.0)),
                }},
            };
            return s;
        }


        const std::vector<Group<AlignParams> > &alignSchema() {
            static const std::vector<Group<AlignParams> > s = {
                {"Site", {
                    field("ins.param.lat", "latDeg", &AlignParams::latDeg, between(-90.0, 90.0)),
                    field("ins.param.g", "g", &AlignParams::g, between(1.0, 12.0)),
                }},
                {"ins.param.group.window", {
                    field("ins.param.win", "win", &AlignParams::win, between(2.0, 20000.0)),
                }},
            };
            return s;
        }


        const std::vector<Group<VerifyParams> > &verifySchema() {
            static const std::vector<Group<VerifyParams> > s = {
                {"ins.param.group.init_pos", {
                    field("ins.param.v_lat", "vLatDeg", &VerifyParams::latDeg, between(-90.0, 90.0)),
                    field("ins.param.v_lon", "vLonDeg", &VerifyParams::lonDeg, between(-180.0, 180.0)),
                    field("ins.param.v_h", "vHM", &VerifyParams::hM, between(-2000.0, 100000.0)),
                }},
                {"ins.param.group.init_vel", {
                    field("ins.param.v_vn", "vVN", &VerifyParams::vN, between(-1000.0, 1000.0)),
                    field("ins.param.v_ve", "vVE", &VerifyParams::vE, between(-1000.0, 1000.0)),
                    field("ins.param.v_vd", "vVD", &VerifyParams::vD, between(-1000.0, 1000.0)),
                }},
                {"ins.param.group.init_att", {
                    field("ins.param.v_roll", "vRollDeg", &VerifyParams::rollDeg, between(-360.0, 360.0)),
                    field("ins.param.v_pitch", "vPitchDeg", &VerifyParams::pitchDeg, between(-90.0, 90.0)),
                    field("ins.param.v_heading", "vHeadingDeg", &VerifyParams::headingDeg,
                          between(-360.0, 360.0)),
                }},
                {"ins.param.group.time", {
                    field("ins.param.v_start_tow", "vStartTow", &VerifyParams::startTime,
                          between(0.0, 1000000.0)),
                }},
            };
            return s;
        }


        const char *alertTitle(const AlertLevel lv) {
            switch (lv) {
                case AlertLevel::Error: return I18n::trc("ins.alert.error");
                case AlertLevel::Warn: return I18n::trc("ins.alert.warn");
                default: return I18n::trc("ins.alert.info");
            }
        }
    }

    void renderInsParams(CalibParams &par) {
        UI::ParamPage::render(calibSchema(), par, CalibParams{});
    }

    void renderInsParams(AlignParams &par) {
        UI::ParamPage::render(alignSchema(), par, AlignParams{});
    }

    void renderInsParams(VerifyParams &par) {
        UI::ParamPage::render(verifySchema(), par, VerifyParams{});
    }

    void saveInsParams(const CalibParams &par) {
        UI::ParamPage::save(calibSchema(), par, kInsPrefix);
    }

    void saveInsParams(const AlignParams &par) {
        UI::ParamPage::save(alignSchema(), par, kInsPrefix);
    }

    void saveInsParams(const VerifyParams &par) {
        UI::ParamPage::save(verifySchema(), par, kInsPrefix);
    }

    void loadInsParams(CalibParams &par) {
        UI::ParamPage::load(calibSchema(), par, CalibParams{}, kInsPrefix);
    }

    void loadInsParams(AlignParams &par) {
        UI::ParamPage::load(alignSchema(), par, AlignParams{}, kInsPrefix);
    }

    void loadInsParams(VerifyParams &par) {
        UI::ParamPage::load(verifySchema(), par, VerifyParams{}, kInsPrefix);
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
                conflict = I18n::tr("ins.scale.model_conflict", recordModel->name, info.model->name);
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
        if (unknownType >= 0) return I18n::tr("ins.scale.unknown_type", unknownType);
        if (!note.empty()) return I18n::tr("ins.scale.no_selftype_note", note);
        return I18n::tr("ins.scale.no_selftype");
    }

    std::string ScaleResolution::describe() const {
        switch (src) {
            case ScaleSource::Record: return I18n::tr("ins.scale.record_label", modelName, modelType);
            case ScaleSource::Guess: return I18n::tr("ins.scale.guess_describe", modelName);
            case ScaleSource::Manual: return I18n::tr("ins.scale.manual");
            default: return {};
        }
    }

    std::string ScaleResolution::recordLabel() const {
        if (!recordModel) return {};
        return I18n::tr("ins.scale.record_label", recordModel->name, recordModel->type);
    }

    ScaleNotice ScaleResolution::notice(const bool settled) const {
        if (!settled) {
            if (!recordModel) return {};
            return {true, AlertLevel::Info, I18n::tr("ins.scale.src_record_short", recordLabel())};
        }
        if (!known()) {
            std::string t = I18n::tr("ins.scale.unknown_head");
            if (unknownType >= 0) t += I18n::tr("ins.scale.unknown_why_type", unknownType);
            else if (!note.empty()) t += I18n::tr("ins.scale.unknown_why_guess", note);
            return {true, AlertLevel::Warn, t + I18n::tr("ins.punct.period")};
        }
        bool weak = src != ScaleSource::Record;
        std::string t;
        switch (src) {
            case ScaleSource::Record: t = I18n::tr("ins.scale.src_record", recordLabel());
                break;
            case ScaleSource::Guess: t = I18n::tr("ins.scale.src_guess", modelName);
                break;
            case ScaleSource::Manual: t = I18n::tr("ins.scale.src_manual");
                break;
            default: return {};
        }
        if (noType > 0) {
            t += I18n::tr("ins.scale.no_type_note", noType);
            weak = true;
        } else {
            t += I18n::tr("ins.punct.period");
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
        return res.failMessage() + I18n::tr("ins.scale.fill_manual_hint");
    }

    bool ScaleIdentifier::mixedModel() const {
        return !res.conflict.empty();
    }

    std::string ScaleIdentifier::failureMessage(const std::string &path) const {
        return mixedModel() ? res.conflict : I18n::tr("ins.err.open_imu", path);
    }

    void keepManualScaleFor(ScaleFallback &sf, const std::string &sig) {
        if (sf.accepted.load(std::memory_order_relaxed) && sf.sig != sig) sf.reset();
        sf.sig = sig;
    }

    bool drawScaleFallback(ScaleFallback &sf, const bool allowSkip) {
        if (!sf.asked.load(std::memory_order_acquire)) {
            if (sf.accepted.load(std::memory_order_relaxed) &&
                ImGui::SmallButton((std::string(I18n::trc("ins.scale.edit_manual")) +
                                    "##kf_edit").c_str())) {
                sf.accepted.store(false, std::memory_order_relaxed);
                sf.asked.store(true, std::memory_order_release);
            }
            return false;
        }
        drawAlert(AlertLevel::Warn, sf.reason.empty() ? I18n::trc("ins.scale.no_guess")
                                                      : sf.reason.c_str());
        ImGui::TextDisabled("%s", I18n::trc("ins.scale.manual_panel_hint"));
        ImGui::Spacing();
        auto field = [](const char *id, const char *label, const char *unit, std::atomic<double> &v,
                        const char *fmt) {
            ImGui::TextUnformatted(I18n::trc(label));
            ImGui::SameLine(160.0f);
            ImGui::SetNextItemWidth(140.0f);
            double val = v.load(std::memory_order_relaxed);
            if (ImGui::InputDouble(id, &val, 0.0, 0.0, fmt)) v.store(val, std::memory_order_relaxed);
            ImGui::SameLine();
            ImGui::TextDisabled("%s", unit);
        };
        field("##kf_a", "ins.scale.k_a", "(m/s)/LSB", sf.accelMpsPerLsb, "%.6e");
        field("##kf_g", "ins.scale.k_g", "°/LSB", sf.gyroDegPerLsb, "%.6e");
        field("##kf_r", "SamplingRate", "Hz", sf.rateHz, "%.3f");
        ImGui::Spacing();

        bool retry = false;
        ImGui::BeginDisabled(!sf.scale().known());
        if (ImGui::Button(I18n::trc("ins.scale.recount"), ImVec2(160, 0))) {
            sf.accepted.store(true, std::memory_order_relaxed);
            sf.asked.store(false, std::memory_order_release);
            retry = true;
        }
        ImGui::EndDisabled();
        if (allowSkip) {
            ImGui::SameLine();
            if (ImGui::Button(I18n::trc("ins.scale.skip_check"), ImVec2(150, 0)))
                sf.asked.store(false, std::memory_order_release);
        }
        ImGui::Spacing();
        return retry;
    }


    std::size_t decimateStride(const std::size_t n, const std::size_t maxPts) {
        return n > maxPts ? (n + maxPts - 1) / maxPts : 1;
    }

    void decimateImu(const ImuLog &src, ImuLog &out) {
        out.clear();
        if (src.empty()) return;
        const std::size_t stride = decimateStride(src.size());
        out.reserve((src.size() + stride - 1) / stride);
        for (std::size_t i = 0; i < src.size(); i += stride) out.push_back(src[i]);
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
            ImGui::TextDisabled("%s", I18n::trc("ins.no_data"));
            return;
        }
        const int n = static_cast<int>(t.size());
        if (ImPlot::BeginPlot(I18n::trc(title), ImVec2(-1, kFileChartH))) {
            ImPlot::SetupAxes(I18n::trc("ins.chart.x_time"), I18n::trc("common.value"));
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
        if (ImPlot::BeginPlot(I18n::trc(title), ImVec2(-1, kChartHeight))) {
            if (ready) {
                ImPlot::SetupAxes(I18n::trc("ins.chart.x_axis"), I18n::trc("common.value"),
                                  ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit);
            } else {
                ImPlot::SetupAxes(I18n::trc("ins.chart.x_axis"), I18n::trc("common.value"),
                                  ImPlotAxisFlags_None, ImPlotAxisFlags_None);
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
                ImPlot::PlotBars(I18n::trc("common.value"), xs, v, 3, 0.5, bs);
                for (int k = 0; k < 3; ++k) {
                    char buf[32];
                    std::snprintf(buf, sizeof(buf), fmt, v[k]);
                    ImPlot::PlotText(buf, xs[k], v[k]);
                }
            } else {
                ImPlot::PlotText(I18n::trc("ins.chart.computing"), 1.0, 0.0);
            }
            ImPlot::EndPlot();
        }
    }

    void vec3Row(const char *label, const double x, const double y, const double z, const char *fmt) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(I18n::trc(label));
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
        const std::size_t lo = s.size() * 2 / 3;
        double ny = 0, np = 0, nr = 0;
        for (std::size_t i = lo; i < s.size(); ++i) {
            ny += s[i].yaw;
            np += s[i].pitch;
            nr += s[i].roll;
        }
        const auto m = static_cast<double>(s.size() - lo);
        ny /= m;
        np /= m;
        nr /= m;
        double vy = 0, vp = 0, vr = 0;
        for (std::size_t i = lo; i < s.size(); ++i) {
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
