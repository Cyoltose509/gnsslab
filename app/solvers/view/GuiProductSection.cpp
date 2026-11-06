#include "ui/GuiColors.h"
#include <windows.h>

#include "GuiHelpers.h"
#include "GuiProductSection.h"

#include "core/PathMemory.h"
#include "io/Products.h"
#include "io/ProductPicker.h"
#include "io/ObsSource.h"
#include "ui/FileDialog.h"

#include "download/ProductMeta.h"
#include "download/Acquire.h"
#include "download/ProductMeta.h"

#include "StringUtils.h"
#include <algorithm>
#include <functional>
#include <map>
#include <string>
#include <vector>


namespace {
    // 选中一条路径后的公共动作：写入 PathMemory、进历史、触发回调、自动配 NAV
    void adoptPath(const char *iniKey, const std::string &p,
                   const std::function<void(const std::string &)> &onSelect,
                   std::string *autoNavBuf) {
        const auto &pm = AppConfig::PathMemory::instance();
        pm.set(iniKey, p);
        pm.addRecent(iniKey, p);
        if (onSelect) onSelect(p);
        if (autoNavBuf) {
            if (const std::string d = ObsSource::autoDetectNav(p); !d.empty()) *autoNavBuf = d;
        }
    }
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
            const std::string p = Str::wideToACP(wpath);
            *buf = p;
            adoptPath(iniKey, p, onSelect, autoNavBuf);
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
                adoptPath(iniKey, r, onSelect, autoNavBuf);
            }
        ImGui::EndPopup();
    }

    if (obsIniKey && std::string(iniKey) == obsIniKey) {
        static std::map<const void *, Download::Acquire::Panel> panels;
        auto &panel = panels[buf];
        panel.kind = Download::Acquire::Panel::Kind::Obs;
        panel.obsKey = iniKey;
        panel.obsOut = buf;

        ImGui::SameLine();
        if (ImGui::Button(("获取##" + std::string(label)).c_str())) {
            auto yd = ProductMeta::parseObsDate(*buf);
            if (!yd) yd = ProductMeta::defaultObsDate();
            panel.year = yd->first;
            panel.doy = yd->second;
            panel.open = true;
        }
        if (panel.open) Download::Acquire::render(panel, iniKey);
    }

    if (const ProductType ptype = ProductMeta::typeFromIniKey(iniKey); ptype != ProductType::Unknown) {
        static std::map<const void *, Download::Acquire::Panel> singlePanels;
        auto &panel = singlePanels[buf];
        panel.kind = Download::Acquire::Panel::Kind::Single;
        panel.singleType = ptype;
        panel.singleOut = buf;
        if (!panel.onPick)
            panel.onPick = [iniKey, onSelect, autoNavBuf](const std::string &path) {
                if (path.empty()) return;
                adoptPath(iniKey, path, onSelect, autoNavBuf);
            };

        const std::string obsPath = obsIniKey && *obsIniKey
                                        ? AppConfig::PathMemory::instance().get(obsIniKey)
                                        : "";
        const auto yd = ProductMeta::parseObsDate(obsPath);

        ImGui::SameLine();
        if (ImGui::Button(("获取##" + std::string(label)).c_str())) {
            const auto [y, d] = yd.value_or(ProductMeta::defaultObsDate());
            panel.year = y;
            panel.doy = d;
            panel.open = true;
        }
        if (panel.open) Download::Acquire::render(panel, iniKey);
    }
}

void Solvers::Helpers::renderProductSection(AppConfig::Products &p, const char *obsIniKey) {
    struct SectionMsg {
        std::string msg;
        bool ok = true;
    };
    static std::map<const void *, SectionMsg> msgs;
    auto &[msg, ok] = msgs[&p];

    const auto resync = [&] {
        p.syncFromFiles();
        p.save();
    };

    const auto chosenOf = [&](const std::string &path) -> std::string {
        const auto m = ProductMeta::parse(Str::baseName(path));
        if (!m) return {};
        std::vector<std::string> same;
        for (const auto &f: p.files) {
            if (const auto g = ProductMeta::parse(Str::baseName(f)); g && g->type == m->type) same.push_back(f);
        }
        if (same.size() < 2) return "选用";
        if (m->type == ProductType::VMF) return "选用"; // 时次组：每一份都要
        const auto ranked = ProductPicker::rank(same, m->type);
        const std::string win = ranked.empty() ? same.front() : ranked.front().path;
        return win == path ? "选用" : "备用";
    };

    const std::string obsPath = obsIniKey && *obsIniKey
                                    ? AppConfig::PathMemory::instance().get(obsIniKey)
                                    : "";
    const auto yd = ProductMeta::parseObsDate(obsPath);

    if (!ImGui::CollapsingHeader("产品文件", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        ImGui::TextDisabled("%d 个文件", static_cast<int>(p.files.size()));
        return;
    }

    if (ImGui::SmallButton("添加文件…##prod")) {
        if (std::vector<std::wstring> ws; ShowOpenFilesDialog(ws, FileDialog::fProd, 2)) {
            int added = 0;
            std::string bad;
            for (auto &w: ws) {
                const std::string path = Str::wideToACP(w);
                if (std::find(p.files.begin(), p.files.end(), path) != p.files.end()) continue;
                p.files.push_back(path);
                p.addRecent(path);
                if (!ProductMeta::parse(Str::baseName(path))) bad += Str::baseName(path) + " ";
                ++added;
            }
            resync();
            ok = bad.empty();
            msg = "已添加 " + std::to_string(added) + " 个文件";
            if (!bad.empty()) msg += "；认不出类型（不会用于解算）：" + bad;
        }
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("从记录添加##prod")) ImGui::OpenPopup("##prod_hist_pop");
    if (ImGui::BeginPopup("##prod_hist_pop")) {
        const auto rec = p.recent();
        if (rec.empty()) ImGui::TextDisabled("（暂无记录）");
        for (auto &r: rec)
            if (ImGui::Selectable(Str::baseName(r).c_str())) {
                if (std::find(p.files.begin(), p.files.end(), r) == p.files.end()) {
                    p.files.push_back(r);
                    resync();
                }
            }
        ImGui::EndPopup();
    }
    ImGui::SameLine();
    static std::map<std::string, Download::Acquire::Panel> acqPanels;
    auto &acq = acqPanels[obsIniKey ? obsIniKey : ""];
    acq.kind = Download::Acquire::Panel::Kind::Products;
    acq.target = &p.files;
    acq.owner = &p;
    if (ImGui::SmallButton("获取##prod")) {
        const auto [y, d] = yd.value_or(ProductMeta::defaultObsDate());
        acq.year = y;
        acq.doy = d;
        acq.open = true;
        ok = true;
        msg = "点已下载文件加入/移出解算，未下载文件点击开始下载";
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("清空##prod")) {
        p.files.clear();
        resync();
        msg.clear();
    }

    // 内联展开的产品清单（按钮下方），页面整体往下推
    if (acq.open) Download::Acquire::render(acq, obsIniKey ? obsIniKey : "prod");

    ImGui::Spacing();
    if (p.files.empty()) {
        ImGui::TextDisabled("尚未添加产品文件。可一次多选多个，类型由文件名自动识别。");
    } else {
        ImGui::Text("已选 %d 个文件：", static_cast<int>(p.files.size()));
        for (std::size_t i = 0; i < p.files.size(); ++i) {
            const std::string &path = p.files[i];
            const auto m = ProductMeta::parse(Str::baseName(path));

            ImGui::PushID(static_cast<int>(i));
            ImGui::Bullet();
            ImGui::Text("%s", Str::baseName(path).c_str());
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", path.c_str());

            ImGui::SameLine();
            if (!m) {
                ImGui::TextColored(UI::kColFail, "[未识别类型]");
            } else {
                const std::string use = chosenOf(path);
                const bool win = use == "选用";
                ImGui::TextColored(win ? UI::kColPass
                                       : UI::kColMute,
                                   "[%s/%s]", Download::typeLabel(m->type), use.c_str());
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("x##prod")) {
                p.files.erase(p.files.begin() + static_cast<long>(i));
                resync();
                ImGui::PopID();
                break; // 迭代器已失效，下一帧再画
            }
            ImGui::PopID();
        }
    }

    if (!msg.empty())
        ImGui::TextColored(ok ? UI::kColPass : UI::kColFail,
                           "%s", msg.c_str());
}
