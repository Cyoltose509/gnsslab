#include "Panel.h"

#include "Remote.h"
#include "Widgets.h"
#include "Catalog.h"
#include "download/ProductMeta.h"
#include "Store.h"

#include "core/PathMemory.h"
#include "ui/DatePicker.h"
#include "ui/Gui.h"
#include "ui/GuiColors.h"

#include "StringUtils.h"
#include "TimeConvert.h"

#include "imgui.h"

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <set>
#include <string>
#include <vector>

namespace Download::Panel {
    namespace {
        State s_state;
    }

    State &state() {
        return s_state;
    }

    std::set<std::string> State::selectedDatacenters() const {
        std::set<std::string> sel;
        const auto &dcs = ProductMeta::knownDatacenters();
        for (std::size_t i = 0; i < dcOn.size() && i < dcs.size(); ++i)
            if (dcOn[i]) sel.insert(dcs[i]);
        return sel;
    }

    void State::renderDateTab() {
        if (year == 0) UI::DatePicker::today(year, month, day);
        const auto &dcs = ProductMeta::knownDatacenters();
        if (dcOn.size() != dcs.size()) dcOn.assign(dcs.size(), 0);
        if (typeOn.size() != static_cast<std::size_t>(kDateTypeCount))
            typeOn.assign(kDateTypeCount, 1);

        ImGui::TextUnformatted("产品类型");
        Widgets::chipWrap([](const int i) { return kTypeLabels[i]; }, typeOn);

        ImGui::TextUnformatted("数据中心");
        Widgets::chipWrap([&dcs](const int i) { return dcs[static_cast<std::size_t>(i)].c_str(); }, dcOn);

        Widgets::searchBox("dateFind", dateFind, sizeof dateFind);

        ImGui::TextUnformatted("本地状态");
        ImGui::SameLine();
        Widgets::stateFilterChips("##dateState", &dateStateFilter);

        Widgets::dateRow(year, month, day, "date", "今天", UI::DatePicker::today, true);
        const int doy = Time::YMDToDOY({year, month, day});

        ImGui::Separator();

        std::set<ProductType> types;
        for (int i = 0; i < kDateTypeCount; ++i)
            if (typeOn[i]) types.insert(static_cast<ProductType>(i));

        const std::vector<std::string> dirs = ProductMeta::scanDirs(types, year, doy);
        const auto snap = Remote::collect(dirs, types, year, doy, selectedDatacenters());

        std::vector<Remote::Match> matches = snap.matches;
        Widgets::ListResult res;

        ImGui::BeginChild("##files", ImVec2(0, -30), ImGuiChildFlags_Borders);
        if (types.empty()) {
            ImGui::TextDisabled("先勾选上面的产品类型。");
        } else {
            Widgets::ListOptions opt;
            opt.grouping = Widgets::Grouping::ByType;
            opt.stateFilter = dateStateFilter;
            opt.find = dateFind;
            opt.onClick = [](const Remote::Match &m, const std::string &) {
                if (m.dirUrl.empty()) return;
                Remote::enqueue({m.m.filename, m.dirUrl, m.m.type, m.m.year, m.m.doy, m.m.spanDays});
            };
            res = Widgets::fileList(matches, opt);
        }
        ImGui::EndChild();

        {
            const bool narrowed = dateStateFilter != 0 || *dateFind;
            std::string status;
            if (!Widgets::progressLine(status)) {
                if (narrowed)
                    ImGui::Text("共 %zu / %d 个匹配文件   %s", matches.size(), res.total, status.c_str());
                else
                    ImGui::Text("共 %zu 个匹配文件   %s", matches.size(), status.c_str());
            }
            if (!matches.empty()) {
                ImGui::SameLine();
                if (ImGui::SmallButton("下载全部匹配##dlAll")) {
                    for (const auto &mt: matches) {
                        if (mt.dirUrl.empty()) continue;
                        if (!Store::findLocal(mt.m.type, mt.m.year, mt.m.doy,
                                              mt.m.filename).empty())
                            continue;
                        Remote::enqueue({mt.m.filename, mt.dirUrl, mt.m.type, mt.m.year, mt.m.doy,
                                         mt.m.spanDays});
                    }
                }
            }
        }
    }

    void State::renderObsTab() {
        if (obsYear == 0) UI::DatePicker::today(obsYear, obsMonth, obsDay);

        Widgets::dateRow(obsYear, obsMonth, obsDay, "obs", "今天", UI::DatePicker::today, false);
        const int doy = Time::YMDToDOY({obsYear, obsMonth, obsDay});

        Widgets::searchBox("obsFind", obsFind, sizeof obsFind);

        ImGui::TextUnformatted("本地状态");
        ImGui::SameLine();
        Widgets::stateFilterChips("##obsState", &obsStateFilter);

        const std::set types{ProductType::OBS};
        const std::vector<std::string> dirs = ProductMeta::scanDirs(types, obsYear, doy);
        const auto snap = Remote::collect(dirs, types, obsYear, doy, {}, "读取观测目录…");

        std::vector<Remote::Match> matches = snap.matches;

        Widgets::ListResult res;
        ImGui::BeginChild("##obsFiles", ImVec2(0, -30), ImGuiChildFlags_Borders);
        {
            Widgets::ListOptions opt;
            opt.grouping = Widgets::Grouping::None;
            opt.stateFilter = obsStateFilter;
            opt.find = obsFind;
            opt.onClick = [](const Remote::Match &m, const std::string &) {
                if (m.dirUrl.empty()) return;
                Remote::enqueue({m.m.filename, m.dirUrl, ProductType::OBS, m.m.year, m.m.doy, m.m.spanDays});
            };
            res = Widgets::fileList(matches, opt);
        }
        ImGui::EndChild();

        {
            std::string status;
            if (!Widgets::progressLine(status)) {
                if (obsStateFilter != 0 || *obsFind)
                    ImGui::Text("共 %zu / %d 个观测文件   %s", matches.size(),
                                res.total, status.c_str());
                else
                    ImGui::Text("共 %zu 个观测文件   %s", matches.size(), status.c_str());
            }
        }
    }

    void State::openImportDialog(const VersionedProduct &p) {
        const GuiFileFilter filters[2] = {
            {Str::ACPToWide(p.label), p.exts},
            {L"所有文件", {L"*"}},
        };

        std::wstring wpath;
        if (!ShowOpenFileDialog(wpath, filters, 2)) return;

        const std::string path = Str::wideToACP(wpath);
        const auto [ok, file, version, detail] = Catalog::importLocal(p, path);
        importOk = ok;
        importMsg = ok
                        ? Str::baseName(path) + " → " + version + "/" + file + "（" + detail + "）"
                        : "导入失败：" + detail + "　（" + Str::baseName(path) + "）";
        if (ok)
            Catalog::noteImported(p.id, version, file, "已导入本地文件");
    }

    void State::renderAuxTab() {
        const auto &cfg = AppConfig::PathMemory::instance();
        bool autoOn = Catalog::updatesEnabled();
        if (ImGui::Checkbox("启动时自动检查并更新", &autoOn)) {
            Catalog::setUpdatesEnabled(autoOn);
            cfg.set("dl_autosync", autoOn ? "1" : "0");
        }
        ImGui::SameLine();
        if (ImGui::Button("全部检查更新"))
            Catalog::checkNow();

        ImGui::Separator();

        constexpr ImGuiTableFlags tf = ImGuiTableFlags_SizingStretchProp |
                                       ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH;
        if (ImGui::BeginTable("##aux", 5, tf)) {
            ImGui::TableSetupColumn("产品");
            ImGui::TableSetupColumn("本地");
            ImGui::TableSetupColumn("远端最新");
            ImGui::TableSetupColumn("状态");
            ImGui::TableSetupColumn("操作", ImGuiTableColumnFlags_WidthFixed, 210.0f);
            ImGui::TableHeadersRow();

            for (const auto &p: Catalog::all()) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(p.label);

                const auto inst = Store::active(p.id);
                const auto [phase, version, file, detail, progress, gotBytes, totalBytes] =
                        Catalog::updateStatus(p.id);

                ImGui::TableSetColumnIndex(1);
                if (inst) {
                    ImGui::TextColored(UI::kColPass, "%s", inst->file.c_str());
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s\n版本 %s，%s 更新", inst->path.c_str(),
                                          inst->version.c_str(), inst->fetchedAt.c_str());
                } else {
                    ImGui::TextDisabled("未安装");
                }

                ImGui::TableSetColumnIndex(2);
                if (phase == Catalog::UpdateStatus::Checking)
                    ImGui::TextDisabled("检查中…");
                else if (!version.empty())
                    ImGui::TextUnformatted(file.c_str());
                else
                    ImGui::TextDisabled("—");

                ImGui::TableSetColumnIndex(3);
                switch (phase) {
                    case Catalog::UpdateStatus::Downloading: {
                        char tip[192];
                        if (totalBytes > 0.0)
                            std::snprintf(tip, sizeof tip, "%s\n%.1f / %.1f MB（中断可续传）",
                                          file.c_str(), gotBytes / 1048576.0,
                                          totalBytes / 1048576.0);
                        else
                            std::snprintf(tip, sizeof tip, "%s\n已收 %.1f MB", file.c_str(),
                                          gotBytes / 1048576.0);
                        Widgets::downloadChip(file.c_str(), progress, -1.0f, tip);
                        break;
                    }
                    case Catalog::UpdateStatus::Ready:
                        ImGui::TextColored(UI::kColPass, "%s", detail.empty() ? "就绪" : detail.c_str());
                        break;
                    case Catalog::UpdateStatus::Error:
                        ImGui::TextColored(UI::kColFail, "%s", detail.c_str());
                        break;
                    default:
                        ImGui::TextDisabled("—");
                        break;
                }

                ImGui::TableSetColumnIndex(4);
                const bool busy = phase == Catalog::UpdateStatus::Checking ||
                                  phase == Catalog::UpdateStatus::Downloading;
                ImGui::BeginDisabled(busy);
                if (ImGui::SmallButton(("检查##chk_" + std::string(p.id)).c_str()))
                    Catalog::checkNow(p.id);
                ImGui::EndDisabled();

                ImGui::SameLine();
                if (ImGui::SmallButton(("选取本地文件##imp_" + std::string(p.id)).c_str()))
                    openImportDialog(p);
            }
            ImGui::EndTable();
        }

        ImGui::Spacing();
        if (!importMsg.empty())
            ImGui::TextColored(importOk ? UI::kColPass : UI::kColFail, "%s", importMsg.c_str());
      //  ImGui::TextDisabled("本地目录：%s", Store::productRoot().c_str());
    }

    void State::renderLocalTab() {
        const int done = Remote::progress().done;
        if (done != localDone) {
            localDone = done;
            localDirty = true;
        }

        if (localDirty) {
            localFiles = Store::browse();
            localDirty = false;
            for (auto it = localSel.begin(); it != localSel.end();) {
                const bool alive = std::any_of(localFiles.begin(), localFiles.end(),
                                               [&it](const Store::BrowseEntry &e) { return e.path == *it; });
                it = alive ? std::next(it) : localSel.erase(it);
            }
        }

        Widgets::searchBox("localFind", localFind, sizeof localFind);
        ImGui::SameLine();
        if (ImGui::Button("刷新")) localDirty = true;

        const std::string needle = Str::toUpper(std::string(localFind));
        std::vector<const Store::BrowseEntry *> shown;
        long long shownBytes = 0;
        for (const auto &e: localFiles) {
            if (!needle.empty() && Str::toUpper(e.name).find(needle) == std::string::npos) continue;
            shown.push_back(&e);
            shownBytes += e.size;
        }

        constexpr ImGuiTableFlags tf = ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg |
                                       ImGuiTableFlags_BordersInnerH;

        ImGui::BeginChild("##localFiles", ImVec2(0, -30), ImGuiChildFlags_Borders);
        if (shown.empty()) {
            ImGui::TextDisabled("%s", localFiles.empty()
                                          ? "本地还没有下载任何文件。"
                                          : "当前筛选下没有匹配的文件。");
        } else if (ImGui::BeginTable("##localTable", 5, tf)) {
            ImGui::TableSetupColumn("##pick", ImGuiTableColumnFlags_WidthFixed, 26.0f);
            ImGui::TableSetupColumn("文件");
            ImGui::TableSetupColumn("类别", ImGuiTableColumnFlags_WidthFixed, 120.0f);
            ImGui::TableSetupColumn("日期", ImGuiTableColumnFlags_WidthFixed, 86.0f);
            ImGui::TableSetupColumn("大小", ImGuiTableColumnFlags_WidthFixed, 78.0f);

            for (const auto *e: shown) {
                ImGui::PushID(e->path.c_str());
                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                bool on = localSel.count(e->path) != 0;
                if (ImGui::Checkbox("##pick", &on)) {
                    if (on) localSel.insert(e->path);
                    else localSel.erase(e->path);
                }

                ImGui::TableSetColumnIndex(1);
                if (ImGui::Selectable(e->name.c_str()))
                    Remote::setStatus(Store::revealFile(e->path)
                                          ? "已在资源管理器中定位文件"
                                          : "打开资源管理器失败（文件可能已被移除）");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", e->path.c_str());
                if (ImGui::BeginPopupContextItem("##localCtx")) {
                    if (ImGui::MenuItem("彻底删除…")) Widgets::askDelete({e->path});
                    ImGui::EndPopup();
                }

                ImGui::TableSetColumnIndex(2);
                if (e->aux) {
                    const auto *p = Catalog::find(e->category);
                    ImGui::TextDisabled("%s", p ? p->label : e->category.c_str());
                } else {
                    ImGui::TextDisabled("%s", e->category.c_str());
                }

                ImGui::TableSetColumnIndex(3);
                ImGui::TextDisabled("%s", e->stamp.c_str());

                ImGui::TableSetColumnIndex(4);
                const std::string sz = Widgets::sizeText(e->size);
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x -
                                     ImGui::CalcTextSize(sz.c_str()).x);
                ImGui::TextDisabled("%s", sz.c_str());
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::EndChild();

        {
            int selCount = 0;
            long long selBytes = 0;
            for (const auto &e: localFiles)
                if (localSel.count(e.path)) {
                    ++selCount;
                    selBytes += e.size;
                }

            bool allOn = !shown.empty();
            for (const auto *e: shown)
                if (!localSel.count(e->path)) {
                    allOn = false;
                    break;
                }
            if (ImGui::Checkbox("全选", &allOn)) {
                for (const auto *e: shown) {
                    if (allOn) localSel.insert(e->path);
                    else localSel.erase(e->path);
                }
            }
            ImGui::SameLine();
            ImGui::TextDisabled("共 %zu 个 · %s", shown.size(), Widgets::sizeText(shownBytes).c_str());
            if (selCount > 0) {
                ImGui::SameLine();
                ImGui::TextDisabled("已选 %d · %s", selCount, Widgets::sizeText(selBytes).c_str());
            }

            ImGui::SameLine();
            ImGui::BeginDisabled(selCount == 0);
            if (ImGui::SmallButton("删除选中")) {
                std::vector<std::string> ps;
                ps.reserve(localSel.size());
                for (const auto &p: localSel) ps.push_back(p);
                Widgets::askDelete(ps);
            }
            ImGui::EndDisabled();

            ImGui::SameLine();
            ImGui::BeginDisabled(localFiles.empty());
            if (ImGui::SmallButton("删除全部")) {
                std::vector<std::string> ps;
                ps.reserve(localFiles.size());
                for (const auto &e: localFiles) ps.push_back(e.path);
                Widgets::askDelete(ps);
            }
            ImGui::EndDisabled();
        }
    }

    void State::render() {
        if (!open) {
            localShown = false;
            return;
        }

        bool shown = open;
        ImGui::SetNextWindowSize(ImVec2(900, 560), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("产品下载", &shown)) {
            ImGui::End();
            open = shown;
            return;
        }

        if (ImGui::BeginTabBar("##dl_tabs")) {
            if (ImGui::BeginTabItem("日精密产品")) {
                renderDateTab();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("观测文件")) {
                renderObsTab();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("辅助数据")) {
                renderAuxTab();
                ImGui::EndTabItem();
            }
            const bool localTab = ImGui::BeginTabItem("本地文件");
            if (localTab) {
                if (!localShown) localDirty = true;
                renderLocalTab();
                ImGui::EndTabItem();
            }
            localShown = localTab;
            ImGui::EndTabBar();
        }

        if (Widgets::deleteConfirm()) {
            localDirty = true;
            localSel.clear();
        }

        ImGui::End();
        open = shown;
    }

    void State::shutdown() {
        Remote::shutdown();
    }
}
