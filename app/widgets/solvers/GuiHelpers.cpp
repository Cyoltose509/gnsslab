#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include "GuiHelpers.h"
#include "SolverTask.h"
#include "core/PathMemory.h"
#include "Log.h"

#include "download/ProductMeta.h"
#include "download/DownloadStore.h"
#include "GuiFtpDownload.h"

#include "StringUtils.h"
#include <algorithm>
#include <map>


void Solvers::Helpers::renderRealtimeConfig(GnssTask::SolverTaskBase *task,
                                            std::vector<GnssTask::RealtimeEndpoint> *endpoints,
                                            const std::vector<std::string> &labels,
                                            double *cutoffDeg, std::set<char> *systems,
                                            const std::function<void()> &onStart,
                                            const std::function<void()> &extraContent) {
    const size_t n = endpoints ? endpoints->size() : 0;
    ImGui::Text("连接到 Oem7 实时流 (socket)：");
    ImGui::PushItemWidth(200);
    for (size_t i = 0; i < n && i < labels.size(); ++i) {
        if (!labels[i].empty()) ImGui::Text("%s：", labels[i].c_str());
        inputTextStd(("IP 地址##rt_ep_" + std::to_string(i)).c_str(), &(*endpoints)[i].ip);
        inputTextStd(("端口##rt_ep_" + std::to_string(i)).c_str(), &(*endpoints)[i].port);
    }
    ImGui::PopItemWidth();
    ImGui::Separator();

    renderCutoffAndSystems(cutoffDeg, systems);

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
    if (all_v.size() < 2) return r; // 不足 2 个样本：默认 ±8
    std::sort(all_v.begin(), all_v.end());
    const auto n = static_cast<long>(all_v.size());
    const double lo = all_v[static_cast<size_t>(0.01 * n)];
    const double hi = all_v[static_cast<size_t>(0.99 * n)];
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
        data->Buf = const_cast<char *>(str->c_str()); //NOLINT
    }
    return 0;
}

bool Solvers::Helpers::inputTextStd(const char *label, std::string *str, const ImGuiInputTextFlags flags) {
    if (str->capacity() < 512) str->reserve(512);
    const bool changed = ImGui::InputText(
        label, str->data(), str->capacity() + 1,
        flags | ImGuiInputTextFlags_CallbackResize, inputTextResizeCallback, str);
    if (const size_t len = std::strlen(str->data()); len != str->size()) str->resize(len);
    return changed;
}


void Solvers::Helpers::fileRow(const char *label, std::string *buf, const char *iniKey,
                         const GuiFileFilter *filters, const int nFilters,
                         const std::function<void(const std::string &)> &onSelect,
                         const char *obsIniKey, std::string *autoNavBuf) {
    static std::map<std::string, std::string> synced;
    ImGui::Text("%s:", label);
    ImGui::PushItemWidth(-200.0f);
    const std::string before = *buf;
    inputTextStd(("##" + std::string(label)).c_str(), buf);
    const std::string syncKey = std::string(iniKey) + "@" +
                                std::to_string(std::hash<const void *>{}(buf));
    const bool needCheck = before != *buf || buf->empty()
                           || synced.find(syncKey) == synced.end();
    if (needCheck) {
        if (AppConfig::PathMemory::instance().get(iniKey) != *buf) {
            AppConfig::PathMemory::instance().set(iniKey, *buf);
        }
        synced[syncKey] = *buf;
    }
    ImGui::PopItemWidth();
    ImGui::SameLine();
    if (ImGui::Button(("浏览##" + std::string(label)).c_str())) {
        if (std::wstring wpath; ShowOpenFileDialog(wpath, filters, nFilters)) {
            const std::string p = wideToAcp(wpath);
            *buf = p;
            if (onSelect) onSelect(p);
            if (autoNavBuf) {
                if (const std::string d = GnssTask::AutoDetectNav(p); !d.empty()) *autoNavBuf = d;
            }
            AppConfig::PathMemory::instance().addRecent(iniKey, p);
            AppConfig::PathMemory::instance().set(iniKey, p);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button(("历史##" + std::string(label)).c_str()))
        ImGui::OpenPopup(("##hist_" + std::string(label)).c_str());
    if (ImGui::BeginPopup(("##hist_" + std::string(label)).c_str())) {
        const auto rec = AppConfig::PathMemory::instance().getRecent(iniKey);
        if (rec.empty()) ImGui::TextDisabled("(暂无)");
        for (auto &r: rec)
            if (ImGui::Selectable(r.c_str())) {
                *buf = r;
                if (onSelect) onSelect(r);
                if (autoNavBuf) {
                    if (const std::string d = GnssTask::AutoDetectNav(r); !d.empty()) *autoNavBuf = d;
                }
                AppConfig::PathMemory::instance().set(iniKey, r);
                AppConfig::PathMemory::instance().addRecent(iniKey, r);
            }
        ImGui::EndPopup();
    }

    if (const ProductType ptype = ProductMeta::typeFromIniKey(iniKey); ptype != ProductType::Unknown && ptype != ProductType::ATX) {
        // 日期源：观测文件路径（iniKey 里不含，调用点显式给出 obsIniKey）
        const std::string obsPath = obsIniKey && *obsIniKey
                                  ? AppConfig::PathMemory::instance().get(obsIniKey)
                                  : "";
        const auto yd = ProductMeta::parseObsDate(obsPath);

        ImGui::SameLine();
        if (ImGui::Button(("获取##" + std::string(label)).c_str())) {
            if (!yd) {
                LOG_WARN << "[获取] 无法从观测文件名推断日期（" << label << "），请先填观测文件路径";
                ImGui::OpenPopup(("##getfail_" + std::string(label)).c_str());
            } else {
                if (const auto local = DownloadStore::findLocal(ptype, yd->first, yd->second); !local.empty()) {
                    const std::string p =
                            DownloadStore::dirFor(ptype, yd->first, yd->second) + local.front();
                    *buf = p;
                    if (onSelect) onSelect(p);
                    if (autoNavBuf) {
                        if (const std::string d = GnssTask::AutoDetectNav(p); !d.empty()) *autoNavBuf = d;
                    }
                    AppConfig::PathMemory::instance().set(iniKey, p);
                    AppConfig::PathMemory::instance().addRecent(iniKey, p);
                    LOG_INFO << "[获取] 本地命中 " << ProductMeta::typeDir(ptype) << " "
                            << ProductMeta::doyToDateStr(yd->first, yd->second) << " → " << p;
                } else {
                    LOG_INFO << "[获取] 本地未命中 " << ProductMeta::typeDir(ptype) << " "
                            << ProductMeta::doyToDateStr(yd->first, yd->second)
                            << "，弹出自动下载确认";
                    ImGui::OpenPopup(("##getdl_" + std::string(label)).c_str());
                }
            }
        }

        if (yd && ImGui::BeginPopupModal(("##getdl_" + std::string(label)).c_str(), nullptr,
                                         ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("本地 downloads/ 未找到 %s（COD / %d-%03d）。",
                        ProductMeta::typeDir(ptype).c_str(), yd->first, yd->second);
            ImGui::Text("是否打开下载面板自动获取？");
            if (ImGui::Button("是")) {
                Tools::FtpDownload::RequestAutoDownload(ptype, yd->first, yd->second, "COD");
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("否")) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
        if (!yd && ImGui::BeginPopupModal(("##getfail_" + std::string(label)).c_str(), nullptr,
                                          ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("无法从观测文件名推断日期，请先填写观测文件路径。");
            if (ImGui::Button("知道了")) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
    }
}

void Solvers::Helpers::renderCutoffAndSystems(double *cutoffDeg, std::set<char> *sysMask,
                                        const std::function<void()> &extraSameLine) {
    ImGui::PushItemWidth(120);
    ImGui::InputDouble("截止高度角(°)", cutoffDeg);
    ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("选择星座:");
    struct {
        char ch;
        const char *label;
    } cons[] = {{'G', "GPS"}, {'R', "GLO"}, {'E', "GAL"}, {'C', "BDS"}};
    for (auto &[ch, label]: cons) {
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
std::string Solvers::Helpers::baseName(const std::string &p) {
    const auto pos = p.find_last_of("\\/");
    return pos != std::string::npos ? p.substr(pos + 1) : p;
}

// 判断路径是否为 SP3 精密轨道文件
void Solvers::Helpers::beginConfigBody(const std::string &idBase, const GnssTask::SolverTaskBase &task) {
    const std::string id = idBase + std::to_string(task.cfgIndex);
    ImGui::BeginChild(id.c_str(), ImVec2(0.0f, 0.0f), ImGuiChildFlags_None);
}

void Solvers::Helpers::endConfigBody() {
    ImGui::EndChild();
}

bool Solvers::Helpers::saveFileDialog(const std::string &defaultName, std::wstring &outPath,
                                      const GuiFileFilter *filters, const int nFilters,
                                      const wchar_t *defExt) {
    const std::wstring wDef = acpToWide(defaultName);
    return ShowSaveFileDialog(outPath, filters, nFilters, wDef.c_str(), defExt);
}

bool Solvers::Helpers::saveCSVDialog(const std::string &defaultName, std::wstring &outPath) {
    return saveFileDialog(defaultName, outPath, fCsv, 2, L"csv");
}
