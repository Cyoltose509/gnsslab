#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include "GuiHelpers.h"
#include "SolverTask.h"   // GnssTask::SolverTaskBase 完整定义（beginConfigBody 读 cfgIndex）
#include "core/PathMemory.h"
#include "Log.h"

#include "download/ProductMeta.h"
#include "download/DownloadStore.h"
#include "GuiFtpDownload.h"

#include "StringUtils.h"
#include <algorithm>
#include <map>
#include <mutex>


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
                         const std::function<void(const std::string &)> &onSelect,
                         const char *obsIniKey) {
// 每个 (iniKey, buf 地址) 上次已同步到 ini 的值；用于只在真正变化时写一次盘。
    static std::map<std::string, std::string> synced;
    ImGui::Text("%s:", label);
    ImGui::PushItemWidth(-200.0f);
    // 用户手动清空输入框（含退格/Ctrl+A Del/粘贴后删/中文输入法组合）时必须同步擦掉 ini 键，
    // 否则下次启动又被旧值填回。不依赖 InputText 的 edited 返回值——它在若干清空路径下会漏报；
    // 改为「与 ini 里的实际值比对」，并用 synced 记住上次同步值，保证只在真正变化时写一次盘。
    const std::string before = *buf;
    inputTextStd(("##" + std::string(label)).c_str(), buf);
    // 以 (iniKey, buf 地址) 为键：同一 iniKey 可能被多个标签页各自持有独立的 buf。
    const std::string syncKey = std::string(iniKey) + "@" +
                                std::to_string(std::hash<const void *>{}(buf));
    // 首次渲染 / buf 本帧变化 / **buf 为空** 都要同步。buf 为空这一条是补漏：
    // 用户清空后若 ImGui 当帧未上报编辑（输入法组合、失焦、整段粘贴后删），
    // 只靠 before!=*buf 会漏掉，重启就被旧值填回。空的判定代价极小，可每帧做。
    const bool needCheck = (before != *buf) || buf->empty()
                           || synced.find(syncKey) == synced.end();
    // contains() 是 C++20，本项目 C++17 ⇒ 用 find。
    if (needCheck) {
        if (GnssCore::PathMemory::instance().get(iniKey) != *buf) {
            GnssCore::PathMemory::instance().set(iniKey, *buf);
        }
        synced[syncKey] = *buf;
    }
    ImGui::PopItemWidth();
    ImGui::SameLine();
    if (ImGui::Button(("浏览##" + std::string(label)).c_str())) {
        if (std::wstring wpath; ShowOpenFileDialog(wpath, filters, nFilters)) {
            const std::string p = wideToAcp(wpath);
            *buf = p;
            if (onSelect) onSelect(p);   // 派生动作（如自动扫描伴生星历）
            // 选中即持久化：写入 ini（下次打开默认填回），并提到历史最前
            GnssCore::PathMemory::instance().addRecent(iniKey, p);
            GnssCore::PathMemory::instance().set(iniKey, p);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button(("历史##" + std::string(label)).c_str()))
        ImGui::OpenPopup(("##hist_" + std::string(label)).c_str());
    if (ImGui::BeginPopup(("##hist_" + std::string(label)).c_str())) {
        const auto rec = GnssCore::PathMemory::instance().getRecent(iniKey);
        if (rec.empty()) ImGui::TextDisabled("(暂无)");
        for (auto &r : rec)
            if (ImGui::Selectable(r.c_str())) {
                *buf = r;
                if (onSelect) onSelect(r);
                // 选中历史项也要持久化：写入 ini（下次打开默认填回），并提到历史最前
                GnssCore::PathMemory::instance().set(iniKey, r);
                GnssCore::PathMemory::instance().addRecent(iniKey, r);
            }
        ImGui::EndPopup();
    }

    // ---- "获取"：从本地 downloads/ 按 (产品类型, 日期) 取文件；未命中提示自动下载 ----
    // ATX 是版本化文件（非逐日），镜像上也没有，故不出"获取"按钮。
    const ProductType ptype = ProductMeta::typeFromIniKey(iniKey);
    if (ptype != ProductType::Unknown && ptype != ProductType::ATX) {
        // 日期源：观测文件路径（iniKey 里不含，调用点显式给出 obsIniKey）
        std::string obsPath = (obsIniKey && *obsIniKey)
                                  ? GnssCore::PathMemory::instance().get(obsIniKey)
                                  : "";
        auto yd = ProductMeta::parseObsDate(obsPath);

        ImGui::SameLine();
        if (ImGui::Button(("获取##" + std::string(label)).c_str())) {
            if (!yd) {
                LOG_WARN << "[获取] 无法从观测文件名推断日期（" << label << "），请先填观测文件路径";
                ImGui::OpenPopup(("##getfail_" + std::string(label)).c_str());
            } else {
                const auto local = DownloadStore::findLocal(ptype, yd->first, yd->second);
                if (!local.empty()) {
                    const std::string p =
                        DownloadStore::dirFor(ptype, yd->first, yd->second) + local.front();
                    *buf = p;
                    if (onSelect) onSelect(p);
                    GnssCore::PathMemory::instance().set(iniKey, p);
                    GnssCore::PathMemory::instance().addRecent(iniKey, p);
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

        // 未找到 → 提示自动下载（默认数据中心 COD）
        if (yd && ImGui::BeginPopupModal(("##getdl_" + std::string(label)).c_str(), nullptr,
                                         ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("本地 downloads/ 未找到 %s（COD / %d-%03d）。",
                        ProductMeta::typeDir(ptype).c_str(), yd->first, yd->second);
            ImGui::Text("是否打开下载面板自动获取？");
            if (ImGui::Button("是")) {
                GuiFtpDownload::RequestAutoDownload(ptype, yd->first, yd->second, "COD");
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("否")) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
        // 无法从观测文件名推断日期
        if (!yd && ImGui::BeginPopupModal(("##getfail_" + std::string(label)).c_str(), nullptr,
                                          ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("无法从观测文件名推断日期，请先填写观测文件路径。");
            if (ImGui::Button("知道了")) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
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

// 判断路径是否为 RINEX 观测文件（.??O / .rnx / .obs）
bool GuiHelpers::isRinexObsPath(const std::string &path) {
    if (path.size() < 4) return false;
    std::string low = path;
    std::transform(low.begin(), low.end(), low.begin(), tolower);
    if (low.size() >= 4 && low.compare(low.size() - 4, 4, ".rnx") == 0) return true;
    if (low.size() >= 4 && low.compare(low.size() - 4, 4, ".obs") == 0) return true;
    // RINEX 2 约定：.??O，倒数第 3 个字符为数字，末尾为 'O'
    if (std::isdigit(static_cast<unsigned char>(path[path.size() - 3])) &&
        std::toupper(path.back()) == 'O')
        return true;
    return false;
}

// 判断路径是否为 SP3 精密轨道文件
bool GuiHelpers::isSp3Path(const std::string &path) {
    std::string low = path;
    std::transform(low.begin(), low.end(), low.begin(), tolower);
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

// 配置面板通用开头/收尾
void GuiHelpers::beginConfigBody(const std::string &idBase, const GnssTask::SolverTaskBase &task) {
    // idBase 里的中文前缀只为可读；子窗口 ID 取整串哈希，cfgIndex 保证每任务唯一且跨帧稳定。
    // 铺满标签页剩余空间 ⇒ 表单超出时可滚。
    const std::string id = idBase + std::to_string(task.cfgIndex);
    ImGui::BeginChild(id.c_str(), ImVec2(0.0f, 0.0f), ImGuiChildFlags_None);
}

void GuiHelpers::endConfigBody() {
    ImGui::EndChild();
}

// CSV 导出：默认文件名 → 保存对话框。复用 StringUtils::ansiToWide（统一 SPP 原 ansiToWide 与 PPP/LEO 原 MultiByteToWideChar 两种写法）。
bool GuiHelpers::saveCSVDialog(const std::string &defaultName, std::wstring &outPath) {
    const std::wstring wDef = ansiToWide(defaultName);
    return ShowSaveFileDialog(outPath, fCsv, 2, wDef.c_str(), L"csv");
}

