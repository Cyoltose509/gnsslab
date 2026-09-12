#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include "GuiHelpers.h"
#include "core/AppConfig.h"

#include "StringUtils.h"
#include <algorithm>


// 由逐星伪距残差估计稳健 Y 轴范围
GuiHelpers::RobustResRange GuiHelpers::computeRobustResRange(const std::map<SatID, std::vector<double>> &satRes) {
    std::vector<double> all_v;
    for (const auto &[_, vs] : satRes)
        for (double v : vs) all_v.push_back(v);
    RobustResRange r;
    if (all_v.size() < 2) return r;          // 不足 2 个样本：默认 ±8
    std::sort(all_v.begin(), all_v.end());
    const auto n = static_cast<long>(all_v.size());
    const double lo = all_v[static_cast<size_t>(0.01 * n)];
    const double hi = all_v[static_cast<size_t>(0.99 * n)];
    double half = 0.5 * (hi - lo);
    if (!(half > 0.0)) half = 1.0;
    half = std::max(1.0, std::min(half, 20.0));
    r.lo = -half;
    r.hi = half;
    return r;
}

// InputText 直接绑定 std::string
static int inputTextResizeCallback(ImGuiInputTextCallbackData *data) {
    if (data->EventFlag == ImGuiInputTextFlags_CallbackResize) {
        auto *str = static_cast<std::string *>(data->UserData);
        str->resize(data->BufSize - 1);
        data->Buf = const_cast<char *>(str->c_str());//NOLINT
    }
    return 0;
}

bool GuiHelpers::inputTextStd(const char *label, std::string *str, const ImGuiInputTextFlags flags) {
    if (str->capacity() < 512) str->reserve(512);
    const bool changed = ImGui::InputText(
        label, str->data(), str->capacity() + 1,
        flags | ImGuiInputTextFlags_CallbackResize, inputTextResizeCallback, str);
    if (const size_t len = std::strlen(str->data()); len != str->size()) str->resize(len);
    return changed;
}



void GuiHelpers::fileRow(const char *label, std::string *buf, const char *iniKey,
                         const GuiFileFilter *filters, const int nFilters,
                         const std::function<void(const std::string &)> &onSelect) {
    ImGui::Text("%s:", label);
    ImGui::PushItemWidth(-200.0f);
    inputTextStd(("##" + std::string(label)).c_str(), buf);
    ImGui::PopItemWidth();
    ImGui::SameLine();
    if (ImGui::Button(("浏览##" + std::string(label)).c_str())) {
        if (std::wstring wpath; ShowOpenFileDialog(wpath, filters, nFilters)) {
            const std::string p = wideToAcp(wpath);
            *buf = p;
            if (onSelect) onSelect(p);   // 派生动作（如自动扫描伴生星历）
            // 选中即持久化：写入 ini（下次打开默认填回），并提到历史最前
            AppConfig::instance().addRecent(iniKey, p);
            AppConfig::instance().set(iniKey, p);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button(("历史##" + std::string(label)).c_str()))
        ImGui::OpenPopup(("##hist_" + std::string(label)).c_str());
    if (ImGui::BeginPopup(("##hist_" + std::string(label)).c_str())) {
        const auto rec = AppConfig::instance().getRecent(iniKey);
        if (rec.empty()) ImGui::TextDisabled("(暂无)");
        for (auto &r : rec)
            if (ImGui::Selectable(r.c_str())) {
                *buf = r;
                if (onSelect) onSelect(r);
                // 选中历史项也要持久化：写入 ini（下次打开默认填回），并提到历史最前
                AppConfig::instance().set(iniKey, r);
                AppConfig::instance().addRecent(iniKey, r);
            }
        ImGui::EndPopup();
    }
}

// 统一的“截止高度角 + 星座选择”控件（PPP / LEO / SPP 共用）
void GuiHelpers::renderCutoffAndSystems(double *cutoffDeg, std::set<char> *sysMask,
                                        const std::function<void()> &extraSameLine) {
    ImGui::PushItemWidth(120);
    ImGui::InputDouble("截止高度角(°)", cutoffDeg);
    ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("选择星座:");
    struct {
        char ch;
        const char *label;
    } cons[] = {{'G', "GPS"}, {'C', "BDS"}};
    for (auto &[ch, label] : cons) {
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

// 路径取文件名（去掉目录部分）
std::string GuiHelpers::baseName(const std::string &p) {
    const auto pos = p.find_last_of("\\/");
    return pos != std::string::npos ? p.substr(pos + 1) : p;
}

// 判断路径是否为 SP3 精密轨道文件
bool GuiHelpers::isSp3Path(const std::string &path) {
    std::string low = path;
    std::transform(low.begin(), low.end(), low.begin(), ::tolower);
    return low.size() >= 4 && low.compare(low.size() - 4, 4, ".sp3") == 0;
}

// 解算系统 = 用户勾选掩码 ∩ 观测文件实际出现的系统；无交集时退回文件全部系统
std::set<char> GuiHelpers::resolveNavSystems(const std::set<char> &mask,
                                             const std::map<char, std::vector<std::string>> &mapObsTypes,
                                             const char fallbackSys) {
    std::set<char> sysSet;
    for (const auto &[sys, types] : mapObsTypes)
        if (mask.count(sys)) sysSet.insert(sys);
    if (sysSet.empty())
        for (const auto &[sys, types] : mapObsTypes) sysSet.insert(sys);
    if (sysSet.empty() && fallbackSys) sysSet.insert(fallbackSys);
    return sysSet;
}

// 配置窗口通用开头
void GuiHelpers::beginConfigWindow(const std::string &idBase, void *task) {
    if (!ImGui::IsPopupOpen(reinterpret_cast<const char *>(static_cast<ImGuiID>(0)),
                            ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel))
        ImGui::SetNextWindowFocus();
    const std::string winId = idBase + std::to_string(reinterpret_cast<uintptr_t>(task));
    ImGui::Begin(winId.c_str(), nullptr, ImGuiWindowFlags_NoCollapse);
}

// CSV 导出：默认文件名 → 保存对话框。复用 StringUtils::ansiToWide（统一 SPP 原 ansiToWide 与 PPP/LEO 原 MultiByteToWideChar 两种写法）。
bool GuiHelpers::saveCSVDialog(const std::string &defaultName, std::wstring &outPath) {
    const std::wstring wDef = ansiToWide(defaultName);
    return ShowSaveFileDialog(outPath, fCsv, 2, wDef.c_str(), L"csv");
}

