#include "download/ui/Widgets.h"
#include "FileUtils.h"

#include "download/local/FileMeta.h"
#include "core/TimeStruct.h"
#include "download/local/Store.h"

#include "StringUtils.h"
#include "core/TimeStruct.h"

#include "ui/DatePicker.h"
#include "i18n/I18n.h"
#include "ui/GuiColors.h"

#include "imgui.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>
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
                for (const auto &p: paths) bytes += Store::sizeOnDisk(p);
                pending = true;
            }

            bool draw() {
                bool deleted = false;
                if (!ImGui::BeginPopupModal("##confirmDel", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
                    return false;

                ImGui::TextColored(UI::kColFail, "%s", I18n::trc("dl.ui.delete_permanent_title"));
                if (paths.size() == 1) {
                    ImGui::TextUnformatted(I18n::trc("dl.ui.remove_one"));
                    ImGui::TextDisabled("%s", paths.front().c_str());
                } else {
                    ImGui::TextUnformatted(I18n::tr("dl.ui.remove_many", paths.size(),
                                                    sizeText(bytes)).c_str());
                    constexpr std::size_t kMaxShown = 8;
                    for (std::size_t i = 0; i < paths.size() && i < kMaxShown; ++i)
                        ImGui::TextDisabled("%s", File::baseName(paths[i]).c_str());
                    if (paths.size() > kMaxShown)
                        ImGui::TextDisabled("%s", I18n::tr("dl.ui.and_more",
                                                           paths.size() - kMaxShown).c_str());
                }
                ImGui::Spacing();
                if (ImGui::Button(I18n::trc("dl.ui.delete"), ImVec2(100, 0))) {
                    if (const auto r = Store::removeFiles(paths); r.failed == 0)
                        Fetcher::setStatus(I18n::tr("dl.ui.deleted_count", r.removed));
                    else
                        Fetcher::setStatus(I18n::tr("dl.ui.deleted_partial", r.removed, r.failed));
                    deleted = true;
                    paths.clear();
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button(I18n::trc("common.cancel"), ImVec2(90, 0))) {
                    paths.clear();
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
                return deleted;
            }
        };

        DeleteConfirm s_deleteConfirm;

        struct Row {
            std::string local; ///< 本地落点，空＝没有
            bool selected = false;
        };

        std::string localOf(const Fetcher::Match &m) {
            if (!m.localPath.empty()) return m.localPath;
            return Store::findLocal(m.m.type, m.m.year, m.m.doy, m.m.filename);
        }
        std::string tooltipOf(const Fetcher::Match &mt, const std::string &local, const std::string &hint) {
            const FileMeta &m = mt.m;
            std::string tip = Store::localNameFor(m.filename);
            const std::string span = I18n::tr("dl.ui.tip_span", m.spanDays,
                                              DOYToDateText(m.year, m.doy),
                                              m.interval.empty() ? "-" : m.interval);
            const std::string remote = mt.remoteUrl();
            const std::string source = remote.empty() ? std::string()
                                                      : I18n::tr("dl.ui.tip_source", remote);
            if (local.empty()) {
                tip += span + source + I18n::tr("dl.ui.tip_no_local");
            } else if (remote.empty()) {
                tip += I18n::tr("dl.ui.tip_local_only", local);
            } else {
                tip += span + source + I18n::tr("dl.ui.tip_local", local);
                if (!hint.empty()) tip += "\n" + hint;
            }
            return tip;
        }

        /// 圆角胶囊按钮，选中时高亮。
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

        bool passStateFilter(const int f, const bool hasLocal) {
            if (f == 1) return hasLocal;
            if (f == 2) return !hasLocal;
            return true;
        }

        /// 文件胶囊的右键菜单；选彻底删除会打开确认弹窗。
        void fileContextMenu(const std::string &localPath) {
            bool ask = false;
            if (ImGui::BeginPopupContextItem("##fileCtx")) {
                if (ImGui::MenuItem(I18n::trc("common.open_explorer")))
                    Fetcher::setStatus(Store::revealFile(localPath) ? I18n::tr("dl.ui.revealed")
                                                                    : I18n::tr("dl.ui.reveal_failed"));
                if (ImGui::MenuItem(I18n::trc("common.delete_permanent"))) ask = true;
                ImGui::EndPopup();
            }
            if (ask) askDelete({localPath});
        }

        /// 下载中的胶囊：左键暂停，右键定位临时文件或取消并彻底删除。
        void downloadingContextMenu(const std::string &remote, const std::string &partPath) {
            bool ask = false;
            if (ImGui::BeginPopupContextItem("##dlCtx")) {
                if (ImGui::MenuItem(I18n::trc("common.open_explorer")))
                    Fetcher::setStatus(Store::revealFile(partPath)
                                               ? I18n::tr("dl.ui.revealed_temp")
                                               : I18n::tr("dl.ui.reveal_temp_failed"));
                if (ImGui::MenuItem(I18n::trc("dl.ui.delete_cancel_dl"))) ask = true;
                ImGui::EndPopup();
            }
            if (ask) {
                Fetcher::pauseDownload(remote);
                askDelete({partPath});
            }
        }
    }

    void dateRow(int &year, int &month, int &day, const char *id, const char *quickLabel,
                 void (*quick)(int &, int &, int &), const bool showWeek) {
        ImGui::PushID(id);
        ImGui::TextUnformatted(I18n::trc("common.date"));
        ImGui::SameLine();
        UI::DatePicker::button(year, month, day);

        if (quick) {
            ImGui::SameLine();
            if (ImGui::SmallButton(quickLabel)) quick(year, month, day);
        }
        const int doy = YearMonthDay{year, month, day}.toDOY();
        char doyText[8];
        std::snprintf(doyText, sizeof doyText, "%03d", doy);
        ImGui::SameLine();
        if (showWeek)
            ImGui::TextDisabled("%s", I18n::tr("dl.ui.doy_week", doyText,
                                               GPSWeek(year, doy)).c_str());
        else
            ImGui::TextDisabled("%s", I18n::tr("dl.ui.doy", doyText).c_str());
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

    void typeChips(std::vector<unsigned char> &on) {
        if (on.size() != static_cast<std::size_t>(kDateTypeCount)) on.assign(kDateTypeCount, 1);
        ImGui::TextUnformatted(I18n::trc("dl.ui.product_type"));
        chipWrap([](const int i) { return typeLabel(static_cast<ProductType>(i)); }, on);
    }

    bool progressLine(const Fetcher::Snapshot &snap) {
        const auto [active, done, total, status] = Fetcher::progress();
        if (snap.scanning) {
            ImGui::TextUnformatted(I18n::tr("dl.ui.scanning", snap.scanned, snap.total).c_str());
            if (ImGui::IsItemHovered()) {
                ImGui::BeginTooltip();
                ImGui::TextUnformatted(I18n::trc("dl.ui.reading_dirs"));
                ImGui::Separator();
                int shown = 0;
                for (const auto &s: snap.sources) {
                    if (s.done) continue;
                    ImGui::BulletText("%s", s.url.c_str());
                    if (++shown >= 12) break;
                }
                ImGui::EndTooltip();
            }
            return true;
        }
        if (active > 0) {
            ImGui::TextUnformatted(I18n::tr("dl.ui.downloading_done", done, total, status).c_str());
            return true;
        }
        return false;
    }

    void failedSources(const Fetcher::Snapshot &snap) {
        if (snap.scanning || snap.failed <= 0) return;
        ImGui::SameLine();
        ImGui::TextColored(UI::kColFail, "%s", I18n::tr("dl.ui.dirs_failed", snap.failed).c_str());
        if (ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            for (const auto &s: snap.sources) {
                if (!s.done || s.ok) continue;
                ImGui::BulletText("%s", s.url.c_str());
                ImGui::TextDisabled("    %s", s.err.c_str());
            }
            ImGui::EndTooltip();
        }
        ImGui::SameLine();
        if (ImGui::SmallButton(I18n::trc("dl.ui.retry"))) Fetcher::retryFailed();
    }

    void downloadChip(const char *label, const float frac, const float width, const char *tip,
                      const char *stage) {
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
        if (!ImGui::IsItemHovered() || ((!tip || !*tip) && (!stage || !*stage))) return;
        ImGui::BeginTooltip();
        if (tip && *tip) ImGui::TextUnformatted(tip);
        if (stage && *stage) ImGui::TextColored(UI::kColWarn, "%s", stage);
        ImGui::EndTooltip();
    }

    void stateFilterChips(const char *scope, int *v) {
        static const char *keys[3] = {"common.all", "dl.ui.filter_done", "dl.ui.filter_todo"};
        ImGui::PushID(scope);
        for (int i = 0; i < 3; ++i) {
            if (i) ImGui::SameLine();
            if (tagChip(I18n::trc(keys[i]), *v == i)) *v = i;
        }
        ImGui::PopID();
    }

    void searchBox(const char *id, char *buf, const int size) {
        ImGui::PushID(id);
        ImGui::TextUnformatted(I18n::trc("dl.ui.search"));
        ImGui::SameLine();
        ImGui::SetNextItemWidth(220.0f);
        ImGui::InputTextWithHint("##find", I18n::trc("dl.ui.find_hint"), buf,
                                 static_cast<std::size_t>(size));
        if (*buf) {
            ImGui::SameLine();
            if (ImGui::SmallButton(I18n::trc("common.clear"))) buf[0] = '\0';
        }
        ImGui::PopID();
    }

    void askDelete(const std::vector<std::string> &paths) {
        if (!paths.empty()) s_deleteConfirm.ask(paths);
    }

    bool deleteConfirm() {
        return s_deleteConfirm.draw();
    }

    std::string sizeText(const long long bytes) {
        static const char *unit[] = {"B", "KB", "MB", "GB", "TB"};
        auto v = static_cast<double>(bytes);
        int u = 0;
        while (v >= 1024.0 && u < 4) {
            v /= 1024.0;
            ++u;
        }
        char buf[32];
        std::snprintf(buf, sizeof buf, u == 0 ? "%.0f %s" : "%.1f %s", v, unit[u]);
        return buf;
    }

    std::string progressText(const char *verb, const std::string &file, const double frac,
                            const double gotBytes, const double totalBytes, const double speed) {
        std::string s = I18n::tr(verb);
        if (!file.empty()) s += " " + file;
        if (frac >= 0.0) {
            const int pct = static_cast<int>(std::round(frac * 100.0));
            s += " " + std::to_string(pct) + "%";
        } else {
            s += "…";
        }
        if (totalBytes > 0.0)
            s += " (" + sizeText(static_cast<long long>(gotBytes)) + "/"
                 + sizeText(static_cast<long long>(totalBytes)) + ")";
        else if (gotBytes > 0.0)
            s += " (" + sizeText(static_cast<long long>(gotBytes)) + ")";
        if (speed > 0.0) s += " · " + sizeText(static_cast<long long>(speed)) + "/s";
        return s;
    }

    void renderProgress(const char *verb, const std::string &file, const double frac,
                       const double gotBytes, const double totalBytes, const double speed) {
        ImGui::TextColored(UI::kColWarn, "%s",
                           progressText(verb, file, frac, gotBytes, totalBytes, speed).c_str());
    }

    ListResult fileList(std::vector<Fetcher::Match> &matches, const ListOptions &opt) {
        ListResult res;
        res.total = static_cast<int>(matches.size());

        std::vector<Row> rows(matches.size());
        for (std::size_t i = 0; i < matches.size(); ++i) {
            rows[i].local = localOf(matches[i]);
            rows[i].selected = opt.isSelected && opt.isSelected(rows[i].local);
        }

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
            ImGui::TextDisabled("%s", opt.scanning ? I18n::trc("dl.ui.scanning_ellipsis")
                                                   : I18n::trc("dl.ui.no_match"));
            if (!opt.scanning)
                for (const auto &n: opt.notes)
                    if (!n.empty()) ImGui::TextDisabled("  %s", n.c_str());
            return res;
        }

        // 分组：ByType 按日期产品类型分，末尾再兜一个无标题组，装不在日期类型表里的
        // （观测文件、低轨观测、低轨轨道、基站坐标）。
        std::vector<std::pair<const char *, std::vector<std::size_t> > > groups;
        if (opt.grouping == Grouping::ByType) {
            groups.resize(kDateTypeCount);
            for (int t = 0; t < kDateTypeCount; ++t) groups[t].first = typeLabel(static_cast<ProductType>(t));
            std::vector<std::size_t> other;
            for (std::size_t i = 0; i < keep; ++i) {
                if (const int t = dateTypeIndex(matches[i].m.type); t >= 0) groups[t].second.push_back(i);
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

            // 组标题做成可收起的树节点：类型多时一屏铺不完，收起后只留一行「类型 (个数)」
            // 也能看清各类型命中多少。ID 用类型名，显示文案另给，否则个数一变收起状态就丢。
            bool open = true;
            if (name)
                open = ImGui::TreeNodeEx(name, ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_SpanAvailWidth,
                                         "%s (%d)", name, static_cast<int>(idx.size()));
            if (!open) continue;

            float x = 0.0f;
            const float avail = ImGui::GetContentRegionAvail().x;
            const float gap = ImGui::GetStyle().ItemSpacing.x;
            for (std::size_t i: idx) {
                const Fetcher::Match &m = matches[i];
                const FileMeta &meta = m.m;
                const Row &row = rows[i];

                std::string label = Store::localNameFor(meta.filename);
                if (row.selected) label.insert(0, "√ ");

                float frac = -1.0f;
                double totalBytes = -1.0, gotBytes = 0.0, speed = 0.0;
                const bool downloading = Fetcher::downloadingOf(m.remoteUrl(), frac, totalBytes, gotBytes,
                                                                speed);
                const Fetcher::PostState post = downloading
                    ? Fetcher::PostState{}
                    : Fetcher::postStateOf(meta.filename);
                const bool busy = !post.stage.empty();
                const bool bar = busy && post.frac >= 0.0f;
                const float w = downloading || bar ? chipWidth((label + "  100%").c_str())
                                                   : chipWidth(label.c_str());
                if (x > 0.0f && x + w > avail) x = 0.0f;
                if (x > 0.0f) ImGui::SameLine();

                ImGui::PushID(m.m.filename.c_str());
                if (downloading) {
                    std::string size =
                            totalBytes > 0.0
                                ? I18n::tr("dl.ui.size_of",
                                           sizeText(static_cast<long long>(totalBytes)))
                                : I18n::tr("dl.ui.size_unknown");
                    size += I18n::tr("dl.ui.size_got", sizeText(static_cast<long long>(gotBytes)));
                    if (speed > 0.0) size += " · " + sizeText(static_cast<long long>(speed)) + "/s";
                    if (speed > 0.0 && totalBytes > gotBytes) {
                        const double left = (totalBytes - gotBytes) / speed;
                        size += left >= 3600.0
                                    ? I18n::tr("dl.ui.eta_hours", static_cast<int>(left / 3600.0))
                                    : left >= 60.0
                                          ? I18n::tr("dl.ui.eta_minutes", static_cast<int>(left / 60.0))
                                          : I18n::tr("dl.ui.eta_seconds", static_cast<int>(left));
                    }
                    const std::string tip = label + "\n" + size + I18n::tr("dl.ui.tip_downloading");
                    downloadChip(label.c_str(), frac, w, tip.c_str());
                    if (ImGui::IsItemClicked()) Fetcher::pauseDownload(m.remoteUrl());
                    downloadingContextMenu(m.remoteUrl(),
                                           Store::datePath(meta.type, meta.year, meta.doy,
                                                           meta.filename) + ".part");
                } else if (bar) {
                    const std::string tip = tooltipOf(m, row.local, std::string());
                    downloadChip(label.c_str(), post.frac, w, tip.c_str(), post.stage.c_str());
                } else {
                    // 后台处理中的文件即便已经落盘也不算「已下载完成」，不套绿色、不挂右键菜单。
                    const bool has = !row.local.empty() && !busy;
                    if (has) ImGui::PushStyleColor(ImGuiCol_Text, UI::kColPass);
                    else pushRemoteChipStyle();
                    const bool pressed = ImGui::Button(label.c_str());
                    if (has) ImGui::PopStyleColor();
                    else popRemoteChipStyle();
                    if (ImGui::IsItemHovered()) {
                        const std::string tip = tooltipOf(m, row.local,
                                                          busy ? std::string() : opt.localHint(row.selected));
                        ImGui::BeginTooltip();
                        ImGui::TextUnformatted(tip.c_str());
                        if (busy) ImGui::TextColored(UI::kColWarn, "%s", post.stage.c_str());
                        ImGui::EndTooltip();
                    }
                    if (has) fileContextMenu(row.local);
                    if (pressed && !busy && opt.onClick) opt.onClick(m, row.local);
                }
                ImGui::PopID();
                x += ImGui::GetItemRectSize().x + gap;
            }
            if (name) ImGui::TreePop();
        }
        return res;
    }

    ListResult remoteList(const Fetcher::Snapshot &snap, const ListOptions &opt, const char *childId,
                          const char *noun, const float height) {
        std::vector<Fetcher::Match> matches = snap.matches;
        ListOptions o = opt;
        o.scanning = snap.scanning;
        for (const auto &s: snap.sources)
            if (!s.note.empty()) o.notes.push_back(s.note);

        ListResult res;
        ImGui::BeginChild(childId, ImVec2(0, height == 0.0f ? -30.0f : height), ImGuiChildFlags_Borders);
        res = fileList(matches, o);
        ImGui::EndChild();

        const bool narrowed = o.stateFilter != 0 || (o.find && *o.find);
        if (!progressLine(snap)) {
            if (narrowed)
                ImGui::TextUnformatted(I18n::tr("dl.ui.total_narrowed", res.shown, res.total,
                                                noun).c_str());
            else ImGui::TextUnformatted(I18n::tr("dl.ui.total_count", res.shown, noun).c_str());
        }
        failedSources(snap);
        return res;
    }
}
