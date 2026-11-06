#include "Widgets.h"

#include "ProductMeta.h"
#include "Store.h"

#include "StringUtils.h"
#include "TimeConvert.h"

#include "ui/DatePicker.h"
#include "ui/GuiColors.h"

#include "imgui.h"

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace Download::Widgets {
    namespace {
        /// 彻底删除确认弹窗的状态机：右键或批量删除先把路径挂起，由 deleteConfirm() 每帧推进。
        struct DeleteConfirm {
            std::vector<std::string> paths;
            long long bytes = 0;
            bool pending = false;

            void ask(std::vector<std::string> ps) {
                paths = std::move(ps);
                bytes = 0;
                for (const auto &p: paths) {
                    std::error_code ec;
                    const auto sz = static_cast<long long>(fs::file_size(p, ec));
                    if (!ec) bytes += sz;
                }
                pending = true;
            }

            bool draw() {
                bool deleted = false;
                if (pending) {
                    pending = false;
                    ImGui::OpenPopup("##confirmDel");
                }
                if (!ImGui::BeginPopupModal("##confirmDel", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
                    return false;

                ImGui::TextColored(UI::kColFail, "%s", "彻底删除");
                if (paths.size() == 1) {
                    ImGui::TextUnformatted("将永久移除以下本地文件：");
                    ImGui::TextDisabled("%s", paths.front().c_str());
                } else {
                    ImGui::Text("将永久移除 %zu 个本地文件，共 %s：", paths.size(), sizeText(bytes).c_str());
                    constexpr std::size_t kMaxShown = 8;
                    for (std::size_t i = 0; i < paths.size() && i < kMaxShown; ++i)
                        ImGui::TextDisabled("%s", Str::baseName(paths[i]).c_str());
                    if (paths.size() > kMaxShown)
                        ImGui::TextDisabled("…另有 %zu 个", paths.size() - kMaxShown);
                }
                ImGui::Spacing();
                if (ImGui::Button("删除", ImVec2(100, 0))) {
                    const auto r = Store::removeFiles(paths);
                    if (r.failed == 0)
                        Remote::setStatus("已彻底删除 " + std::to_string(r.removed) + " 个本地文件");
                    else
                        Remote::setStatus("已彻底删除 " + std::to_string(r.removed) + " 个，" +
                                          std::to_string(r.failed) + " 个删除失败（可能被占用）");
                    deleted = true;
                    paths.clear();
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("取消", ImVec2(90, 0))) {
                    paths.clear();
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
                return deleted;
            }
        };

        DeleteConfirm s_deleteConfirm;

        /// 一条候选在本帧的呈现状态。本地路径要问盘，一帧只问一次。
        struct Row {
            std::string local; ///< 本地落点，空＝没有
            bool selected = false;
        };

        /// 本地是否已有该文件：有则给路径，没有则空串。
        std::string localOf(const Remote::Match &m) {
            if (!m.localPath.empty()) return m.localPath;
            return Store::findLocal(m.m.type, m.m.year, m.m.doy, m.m.filename);
        }

        /// tooltip 正文：覆盖天数、采样间隔、本地落点。
        std::string tooltipOf(const Remote::Match &mt, const std::string &local, const std::string &hint) {
            const ProductMeta &m = mt.m;
            std::string tip = Store::localNameFor(m.filename);
            const std::string span = "\n覆盖 " + std::to_string(m.spanDays) + " 天（起始 " +
                                     ProductMeta::doyToDateStr(m.year, m.doy) + "）  采样 " +
                                     (m.interval.empty() ? "-" : m.interval);
            if (local.empty()) {
                tip += span + "\n本地：无（点击下载）";
            } else if (mt.dirUrl.empty()) {
                tip += "\n本地：" + local + "\n远端列表未列出（仅本地）";
            } else {
                tip += span + "\n本地：" + local + "\n" + hint;
            }
            return tip;
        }
    }

    bool tagChip(const char *label, const bool on) {
        constexpr ImVec4 accent(0.26f, 0.59f, 0.98f, 1.00f);
        int colors = 3;
        ImGui::PushStyleColor(ImGuiCol_Button,
                              on ? accent : ImGui::GetStyleColorVec4(ImGuiCol_FrameBg));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                              on
                                  ? ImVec4(0.33f, 0.66f, 1.00f, 1.00f)
                                  : ImGui::GetStyleColorVec4(ImGuiCol_FrameBgHovered));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                              on
                                  ? ImVec4(0.20f, 0.50f, 0.90f, 1.00f)
                                  : ImGui::GetStyleColorVec4(ImGuiCol_FrameBgActive));
        if (on) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
            colors = 4;
        }
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 9.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(9.0f, 4.0f));
        const bool clicked = ImGui::Button(label);
        ImGui::PopStyleVar(2);
        ImGui::PopStyleColor(colors);
        return clicked;
    }

    float chipWidth(const char *label) {
        return ImGui::CalcTextSize(label).x + ImGui::GetStyle().FramePadding.x * 2.0f;
    }

    void heightGrip(const char *id, float &height, const float minH, const float maxH) {
        ImGui::InvisibleButton(id, ImVec2(ImGui::GetContentRegionAvail().x, 7.0f));
        if (ImGui::IsItemActive())
            height = std::clamp(height + ImGui::GetIO().MouseDelta.y, minH, std::max(minH, maxH));
        if (ImGui::IsItemHovered() || ImGui::IsItemActive())
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
        const ImVec2 a = ImGui::GetItemRectMin();
        const ImVec2 b = ImGui::GetItemRectMax();
        const float cy = (a.y + b.y) * 0.5f;
        const float cx = (a.x + b.x) * 0.5f;
        ImGui::GetWindowDrawList()->AddLine(ImVec2(cx - 26.0f, cy), ImVec2(cx + 26.0f, cy),
                                            ImGui::GetColorU32(ImGuiCol_Separator), 2.0f);
    }

    void dateRow(int &year, int &month, int &day, const char *id, const char *quickLabel,
                 void (*quick)(int &, int &, int &), const bool showWeek) {
        ImGui::PushID(id);
        ImGui::TextUnformatted("日期");
        ImGui::SameLine();
        UI::DatePicker::button(year, month, day);

        if (quick) {
            ImGui::SameLine();
            if (ImGui::SmallButton(quickLabel)) quick(year, month, day);
        }
        const int doy = Time::YMDToDOY({year, month, day});
        ImGui::SameLine();
        if (showWeek)
            ImGui::TextDisabled("年积日 %03d   第 %d 周", doy, ProductMeta::GPSWeek(year, doy));
        else
            ImGui::TextDisabled("年积日 %03d", doy);
        ImGui::PopID();
    }

    void chipWrap(const std::function<const char *(int)> &label, std::vector<unsigned char> &on) {
        float x = 0.0f;
        const float avail = ImGui::GetContentRegionAvail().x;
        const float gap = ImGui::GetStyle().ItemSpacing.x;
        for (std::size_t i = 0; i < on.size(); ++i) {
            const char *text = label(static_cast<int>(i));
            if (const float w = chipWidth(text); x > 0.0f && x + w > avail) x = 0.0f;
            if (x > 0.0f) ImGui::SameLine();
            if (tagChip(text, on[i] != 0)) on[i] = on[i] ? 0 : 1;
            x += ImGui::GetItemRectSize().x + gap;
        }
    }

    bool progressLine(std::string &status) {
        const auto pr = Remote::progress();
        status = pr.status;
        if (pr.scanning) {
            ImGui::Text("扫描中 %d/%d  %s", pr.scanDone, pr.scanTotal, pr.status.c_str());
            return true;
        }
        if (pr.active > 0) {
            ImGui::Text("下载中  已完成 %d/%d  %s", pr.done, pr.total, pr.status.c_str());
            return true;
        }
        return false;
    }

    void pushRemoteChipStyle() {
        constexpr float k = 0.55f;
        const ImVec4 b = ImGui::GetStyleColorVec4(ImGuiCol_Button);
        const ImVec4 h = ImGui::GetStyleColorVec4(ImGuiCol_ButtonHovered);
        const ImVec4 a = ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive);
        const ImVec4 t = ImGui::GetStyleColorVec4(ImGuiCol_Text);
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(b.x * k, b.y * k, b.z * k, b.w));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(h.x * k, h.y * k, h.z * k, h.w));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(a.x * k, a.y * k, a.z * k, a.w));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(t.x * 0.78f, t.y * 0.78f, t.z * 0.78f, t.w));
    }

    void popRemoteChipStyle() { ImGui::PopStyleColor(4); }

    void downloadChip(const char *label, const float frac, const float width, const char *tip) {
        char pct[16];
        if (frac < 0.0f) std::snprintf(pct, sizeof pct, "…");
        else std::snprintf(pct, sizeof pct, "%.0f%%", frac * 100.0f);
        const std::string ov = std::string(label) + "  " + pct;

        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 9.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(9.0f, 4.0f));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        ImGui::ProgressBar(frac < 0.0f ? 0.0f : frac, ImVec2(width, 0.0f), ov.c_str());
        ImGui::PopStyleColor();
        ImGui::PopStyleVar(2);
        if (tip && *tip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip);
    }

    bool passStateFilter(const int f, const bool hasLocal) {
        if (f == 1) return hasLocal;
        if (f == 2) return !hasLocal;
        return true;
    }

    void stateFilterChips(const char *scope, int *v) {
        static const char *labels[3] = {"全部", "已下载", "未下载"};
        ImGui::PushID(scope);
        for (int i = 0; i < 3; ++i) {
            if (i) ImGui::SameLine();
            if (tagChip(labels[i], *v == i)) *v = i;
        }
        ImGui::PopID();
    }

    void searchBox(const char *id, char *buf, const int size) {
        ImGui::PushID(id);
        ImGui::TextUnformatted("搜索");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(220.0f);
        ImGui::InputTextWithHint("##find", "文件名关键字…", buf, static_cast<std::size_t>(size));
        if (*buf) {
            ImGui::SameLine();
            if (ImGui::SmallButton("清空")) buf[0] = '\0';
        }
        ImGui::PopID();
    }

    FileAction fileContextMenu(const std::string &localPath) {
        auto act = FileAction::None;
        if (ImGui::BeginPopupContextItem("##fileCtx")) {
            if (ImGui::MenuItem("从资源管理器打开")) act = FileAction::Reveal;
            if (ImGui::MenuItem("彻底删除…")) act = FileAction::Delete;
            ImGui::EndPopup();
        }
        if (act == FileAction::Delete) {
            askDelete({localPath});
        } else if (act == FileAction::Reveal) {
            Remote::setStatus(Store::revealFile(localPath)
                                  ? "已在资源管理器中定位文件"
                                  : "打开资源管理器失败（文件可能已被移除）");
        }
        return act;
    }

    void askDelete(const std::vector<std::string> &paths) {
        if (!paths.empty()) s_deleteConfirm.ask(paths);
    }

    bool deleteConfirm() {
        return s_deleteConfirm.draw();
    }

    std::string sizeText(const long long bytes) {
        static const char *unit[] = {"B", "KB", "MB", "GB", "TB"};
        double v = static_cast<double>(bytes);
        int u = 0;
        while (v >= 1024.0 && u < 4) {
            v /= 1024.0;
            ++u;
        }
        char buf[32];
        std::snprintf(buf, sizeof buf, u == 0 ? "%.0f %s" : "%.1f %s", v, unit[u]);
        return buf;
    }

    ListResult fileList(std::vector<Remote::Match> &matches, const ListOptions &opt) {
        ListResult res;
        res.total = static_cast<int>(matches.size());

        std::vector<Row> rows(matches.size());
        for (std::size_t i = 0; i < matches.size(); ++i) {
            rows[i].local = localOf(matches[i]);
            rows[i].selected = opt.isSelected && opt.isSelected(rows[i].local);
        }

        // 关键字与本地状态一起筛：命中的就地压实到前面。
        // 远端名与本地落盘名都参与比对（.crx → .rnx 这类改名后仍能搜到）。
        const std::string needle = opt.find ? Str::toUpper(opt.find) : std::string();
        const auto hitFind = [&needle](const std::string &remoteName) {
            if (needle.empty()) return true;
            if (Str::toUpper(remoteName).find(needle) != std::string::npos) return true;
            return Str::toUpper(Store::localNameFor(remoteName)).find(needle) != std::string::npos;
        };

        std::size_t keep = 0;
        for (std::size_t i = 0; i < matches.size(); ++i) {
            if (!hitFind(matches[i].m.filename)) continue;
            if (!passStateFilter(opt.stateFilter, !rows[i].local.empty())) continue;
            if (keep != i) {
                matches[keep] = matches[i];
                rows[keep] = rows[i];
            }
            ++keep;
        }
        matches.resize(keep);
        res.shown = static_cast<int>(keep);

        if (keep == 0) {
            ImGui::TextDisabled("%s", Remote::progress().scanning ? "扫描中…" : "当前筛选下没有匹配的文件。");
            return res;
        }

        // 分组：ByType 按产品类型分，末尾再兜一个无标题组，装不在日期类型表里的（观测文件）。
        std::vector<std::pair<const char *, std::vector<std::size_t> > > groups;
        if (opt.grouping == Grouping::ByType) {
            groups.resize(kDateTypeCount);
            for (int t = 0; t < kDateTypeCount; ++t) groups[t].first = kTypeLabels[t];
            std::vector<std::size_t> other;
            for (std::size_t i = 0; i < keep; ++i) {
                const int t = static_cast<int>(matches[i].m.type);
                if (t >= 0 && t < kDateTypeCount) groups[t].second.push_back(i);
                else other.push_back(i);
            }
            if (!other.empty()) groups.emplace_back(nullptr, std::move(other));
        } else {
            std::vector<std::size_t> all(keep);
            for (std::size_t i = 0; i < keep; ++i) all[i] = i;
            groups.emplace_back(nullptr, std::move(all));
        }

        bool firstGroup = true;
        for (const auto &[name, idx]: groups) {
            if (idx.empty()) continue;
            if (!firstGroup) ImGui::Spacing();
            firstGroup = false;
            if (name) ImGui::TextDisabled("%s", name);

            float x = 0.0f;
            const float avail = ImGui::GetContentRegionAvail().x;
            const float gap = ImGui::GetStyle().ItemSpacing.x;
            for (std::size_t i: idx) {
                const Remote::Match &m = matches[i];
                const ProductMeta &meta = m.m;
                const Row &row = rows[i];

                std::string label = Store::localNameFor(meta.filename);
                if (row.selected) label.insert(0, "√ ");

                float frac = -1.0f;
                const bool downloading = Remote::downloadingOf(m.dirUrl + meta.filename, frac);
                const float w = downloading ? chipWidth((label + "  100%").c_str())
                                            : chipWidth(label.c_str());
                if (x > 0.0f && x + w > avail) x = 0.0f;
                if (x > 0.0f) ImGui::SameLine();

                ImGui::PushID(m.m.filename.c_str());
                if (downloading) {
                    const std::string tip = label + "\n下载中…";
                    downloadChip(label.c_str(), frac, w, tip.c_str());
                } else {
                    const bool has = !row.local.empty();
                    if (has) ImGui::PushStyleColor(ImGuiCol_Text, UI::kColPass);
                    else pushRemoteChipStyle();
                    const bool pressed = ImGui::Button(label.c_str());
                    if (has) ImGui::PopStyleColor();
                    else popRemoteChipStyle();
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s", tooltipOf(m, row.local, opt.localHint(row.selected)).c_str());
                    if (has) fileContextMenu(row.local);
                    if (pressed && opt.onClick) opt.onClick(m, row.local);
                }
                ImGui::PopID();
                x += ImGui::GetItemRectSize().x + gap;
            }
        }
        return res;
    }
}
