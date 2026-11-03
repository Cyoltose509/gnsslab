#include <windows.h>

#include "GuiHelpers.h"

#include <algorithm>
#include <cstring>
#include <functional>
#include <map>
#include <string>
#include <vector>


void Solvers::Helpers::renderRealtimeConfig(Solvers::Task *task,
                                            std::vector<Solvers::RealtimeEndpoint> *endpoints,
                                            const std::vector<std::string> &labels,
                                            double *cutoffDeg, std::set<char> *systems,
                                            double *sampleInterval,
                                            const std::function<void()> &onStart,
                                            const std::function<void()> &extraContent) {
    const std::size_t n = endpoints ? endpoints->size() : 0;
    ImGui::Text("连接到 NovAtel 实时流 (socket)：");
    ImGui::PushItemWidth(200);
    for (std::size_t i = 0; i < n && i < labels.size(); ++i) {
        if (!labels[i].empty()) ImGui::Text("%s：", labels[i].c_str());
        inputTextStd(("IP 地址##rt_ep_" + std::to_string(i)).c_str(), &(*endpoints)[i].ip);
        inputTextStd(("端口##rt_ep_" + std::to_string(i)).c_str(), &(*endpoints)[i].port);
    }
    ImGui::PopItemWidth();
    ImGui::Separator();

    renderSimpleConfig(cutoffDeg, systems, sampleInterval);

    if (extraContent) {
        ImGui::Separator();
        extraContent();
    }
    ImGui::Separator();
    if (ImGui::Button("开始解算", ImVec2(200, 40))) {
        if (onStart) onStart();
    }
    ImGui::SameLine();
    renderCancelButton(task);
}

Solvers::Helpers::RobustResRange Solvers::Helpers::computeRobustResRange(const std::map<SatID, std::vector<double> > &satRes,
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

static int inputTextResizeCallback(ImGuiInputTextCallbackData *data) {
    if (data->EventFlag == ImGuiInputTextFlags_CallbackResize) {
        auto *str = static_cast<std::string *>(data->UserData);
        str->resize(data->BufSize - 1);
        data->Buf = const_cast<char *>(str->c_str());
    }
    return 0;
}

bool Solvers::Helpers::inputTextStd(const char *label, std::string *str, const ImGuiInputTextFlags flags) {
    if (str->capacity() < 512) str->reserve(512);
    const bool changed = ImGui::InputText(
        label, str->data(), str->capacity() + 1,
        flags | ImGuiInputTextFlags_CallbackResize, inputTextResizeCallback, str);
    if (const std::size_t len = std::strlen(str->data()); len != str->size()) str->resize(len);
    return changed;
}


void Solvers::Helpers::renderSimpleConfig(double *cutoffDeg, std::set<char> *sysMask,
                                          double *sampleInterval,
                                          const std::function<void()> &extraSameLine) {
    ImGui::PushItemWidth(120);
    ImGui::InputDouble("截止高度角(°)", cutoffDeg);
    if (sampleInterval) {
        ImGui::SameLine();
        if (*sampleInterval < 0.0) *sampleInterval = 0.0;
        // 采样间隔是整秒，框用整数输入；存储仍是 double（解算侧真源），整数输入后只会是整值。
        int secs = static_cast<int>(*sampleInterval);
        if (ImGui::InputInt("采样间隔(s)", &secs)) {
            if (secs < 0) secs = 0;
            *sampleInterval = static_cast<double>(secs);
        }
    }
    ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("选择星座:");
    for (auto &[ch, label]: kConstellations) {
        bool on = sysMask->count(ch) > 0;
        ImGui::SameLine();
        ImGui::Checkbox(label, &on);
        if (on) sysMask->insert(ch);
        else sysMask->erase(ch);
    }
    if (extraSameLine) {
        ImGui::SameLine();
        extraSameLine();
    }
    ImGui::NewLine();
}


void Solvers::Helpers::beginConfigBody(const std::string &idBase, const Solvers::Task &task) {
    const std::string id = idBase + std::to_string(task.cfgIndex);
    ImGui::BeginChild(id.c_str(), ImVec2(0.0f, 0.0f), ImGuiChildFlags_None);
}

void Solvers::Helpers::endConfigBody() {
    ImGui::EndChild();
}
