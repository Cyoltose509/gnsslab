#include "GuiHelpers.h"

#include <algorithm>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "StringUtils.h"
#include "core/Const.h"
#include "solvers/ins/ImuScaleProbe.h"
#include "ui/GuiColors.h"
#include "ui/ImGuiLabel.h"
#include "i18n/I18n.h"

static int inputTextResizeCallback(ImGuiInputTextCallbackData *data) {
    if (data->EventFlag == ImGuiInputTextFlags_CallbackResize) {
        auto *str = static_cast<std::string *>(data->UserData);
        str->resize(data->BufSize - 1);
        data->Buf = const_cast<char *>(str->c_str());
    }
    return 0;
}

namespace Solvers::Helpers {
    PasteResult clipboardXYZButton(const char *id) {
        ImGui::PushID(id);
        PasteResult p;
        p.clicked = ImGui::Button(I18n::trc("helper.paste"));
        if (p.clicked)
            if (const char *clip = ImGui::GetClipboardText()) p.count = Str::scanMeters(clip, p.xyz, 3);
        ImGui::PopID();
        return p;
    }

    void renderImuScale(ImuScale &scale, INS::ScaleDetect &detect, const std::string &imuPath,
                        const double gravity, const char *id) {
        ImGui::PushID(id);
        detect.poll(scale);
        double gyroDeg = scale.gyroRadPerLsb * RAD_TO_DEG;
        ImGui::PushItemWidth(150);
        const bool accelEdited = ImGui::InputDouble(UI::idLabel("ins.scale.k_a", "##ka"),
                                                    &scale.accelMpsPerLsb, 0.0, 0.0, "%.6e");
        const bool gyroEdited = ImGui::InputDouble(UI::idLabel("ins.scale.k_g", "##kg"),
                                                   &gyroDeg, 0.0, 0.0, "%.6e");
        const bool rateEdited = ImGui::InputDouble(UI::idLabel("ins.scale.rate", "##rate"),
                                                   &scale.rateHz, 0.0, 0.0, "%.2f");
        ImGui::PopItemWidth();
        scale.gyroRadPerLsb = gyroDeg * DEG_TO_RAD;
        bool manual = accelEdited || gyroEdited || rateEdited;
        if (const PasteResult p = clipboardXYZButton("scale_paste"); p.count > 0) {
            scale.accelMpsPerLsb = p.xyz[0];
            scale.gyroRadPerLsb = p.xyz[1] * DEG_TO_RAD;
            if (p.count > 2) scale.rateHz = p.xyz[2];
            manual = true;
        }
        if (manual) {
            detect.level = INS::ScaleDetect::Level::None;
            detect.text = I18n::tr("ins.scale.manual_note");
        }

        ImGui::SameLine();
        ImGui::BeginDisabled(imuPath.empty() || detect.running());
        if (ImGui::Button(I18n::trc(detect.running() ? "ins.scale.probing" : "ins.scale.detect")))
            detect.start(imuPath, gravity);
        ImGui::EndDisabled();
        if (detect.text.empty()) {
            ImGui::PopID();
            return;
        }
        const ImVec4 col = detect.level == INS::ScaleDetect::Level::Pass ? UI::kColPass
                           : detect.level == INS::ScaleDetect::Level::Warn ? UI::kColWarn
                           : detect.level == INS::ScaleDetect::Level::Fail ? UI::kColFail
                                                                         : UI::kColMute;
        ImGui::TextColored(col, "%s", detect.text.c_str());
        ImGui::PopID();
    }

    void renderEpochNav(int &selectedEpoch, const int epochCount,
                        const char *epochInfo, const char *tailText) {
        if (epochCount <= 0) return;
        if (selectedEpoch < 0) selectedEpoch = 0;
        if (selectedEpoch >= epochCount) selectedEpoch = epochCount - 1;
        if (ImGui::Button("<<")) selectedEpoch = 0;
        ImGui::SameLine();
        if (ImGui::Button("<")) { if (selectedEpoch > 0) --selectedEpoch; }
        ImGui::SameLine();
        ImGui::PushItemWidth(150);
        int ep = selectedEpoch + 1;
        if (ImGui::SliderInt("##es", &ep, 1, epochCount)) selectedEpoch = ep - 1;
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (ImGui::Button(">")) { if (selectedEpoch < epochCount - 1) ++selectedEpoch; }
        ImGui::SameLine();
        if (ImGui::Button(">>")) selectedEpoch = epochCount - 1;
        if (epochInfo && *epochInfo) {
            ImGui::SameLine();
            ImGui::TextDisabled("| %s", epochInfo);
        }
        if (tailText && *tailText) {
            ImGui::SameLine();
            ImGui::Text("| %s", tailText);
        }
    }

    RobustResRange computeRobustResRange(const std::map<SatID, std::vector<double> > &satRes,
                                         const double minHalf) {
        std::vector<double> all_v;
        for (const auto &[_, vs]: satRes)
            for (double v: vs) all_v.push_back(v);
        RobustResRange r;
        if (all_v.size() < 2) return r;
        std::sort(all_v.begin(), all_v.end());
        const auto n = static_cast<long>(all_v.size());
        const double lo = all_v[static_cast<std::size_t>(0.01 * n)];
        const double hi = all_v[static_cast<std::size_t>(0.99 * n)];
        double half = 0.5 * (hi - lo);
        if (!(half > 0.0)) half = minHalf;
        half = std::max(minHalf, std::min(half, 20.0));
        r.lo = -half;
        r.hi = half;
        return r;
    }

    bool inputTextStd(const char *label, std::string *str, const ImGuiInputTextFlags flags) {
        if (str->capacity() < 512) str->reserve(512);
        const bool changed = ImGui::InputText(
            label, str->data(), str->capacity() + 1,
            flags | ImGuiInputTextFlags_CallbackResize, inputTextResizeCallback, str);
        if (const std::size_t len = std::strlen(str->data()); len != str->size()) str->resize(len);
        return changed;
    }
}
