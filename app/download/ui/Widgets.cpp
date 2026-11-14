#include "download/ui/Widgets.h"
#include "FileUtils.h"

#include "download/local/ProductMeta.h"
#include "download/local/Store.h"

#include "StringUtils.h"
#include "TimeStruct.h"

#include "ui/DatePicker.h"
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

                ImGui::TextColored(UI::kColFail, "%s", "彻底删除");
                if (paths.size() == 1) {
                    ImGui::TextUnformatted("将永久移除以下本地文件：");
                    ImGui::TextDisabled("%s", paths.front().c_str());
                } else {
                    ImGui::Text("将永久移除 %zu 个本地文件，共 %s：", paths.size(), sizeText(bytes).c_str());
                    constexpr std::size_t kMaxShown = 8;
                    for (std::size_t i = 0; i < paths.size() && i < kMaxShown; ++i)
                        ImGui::TextDisabled("%s", File::baseName(paths[i]).c_str());
                    if (paths.size() > kMaxShown)
                        ImGui::TextDisabled("…另有 %zu 个", paths.size() - kMaxShown);
                }
                ImGui::Spacing();
                if (ImGui::Button("删除", ImVec2(100, 0))) {
                    if (const auto r = Store::removeFiles(paths); r.failed == 0)
                        Fetcher::setStatus("已彻底删除 " + std::to_string(r.removed) + " 个本地文件");
                    else
                        Fetcher::setStatus("已彻底删除 " + std::to_string(r.removed) + " 个，" +
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

        struct Row {
            std::string local; ///< 本地落点，空＝没有
            bool selected = false;
        };

        std::string localOf(const Fetcher::Match &m) {
            if (!m.localPath.empty()) return m.localPath;
            return Store::findLocal(m.m.type, m.m.year, m.m.doy, m.m.filename);
        }
        std::string tooltipOf(const Fetcher::Match &mt, const std::string &local, const std::string &hint) {
            const ProductMeta &m = mt.m;
            std::string tip = Store::localNameFor(m.filename);
            const std::string span = "\n覆盖 " + std::to_string(m.spanDays) + " 天（起始 " +
                                     ProductMeta::doyToDateStr(m.year, m.doy) + "）  采样 " +
                                     (m.interval.empty() ? "-" : m.interval);
            const std::string remote = mt.remoteUrl();
            const std::string source = remote.empty() ? "" : "\n来源：" + remote;
            if (local.empty()) {
                tip += span + source + "\n本地：无（点击下载）";
            } else if (remote.empty()) {
                tip += "\n本地：" + local + "\n远端列表未列出（仅本地）";
            } else {
                tip += span + source + "\n本地：" + local;
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
                if (ImGui::MenuItem("从资源管理器打开"))
                    Fetcher::setStatus(Store::revealFile(localPath)
                                          ? "已在资源管理器中定位文件"
                                          : "打开资源管理器失败（文件可能已被移除）");
                if (ImGui::MenuItem("彻底删除…")) ask = true;
                ImGui::EndPopup();
            }
            if (ask) askDelete({localPath});
        }

        /// 下载中的胶囊：左键暂停，右键定位临时文件或取消并彻底删除。
        void downloadingContextMenu(const std::string &remote, const std::string &partPath) {
            bool ask = false;
            if (ImGui::BeginPopupContextItem("##dlCtx")) {
                if (ImGui::MenuItem("从资源管理器打开"))
                    Fetcher::setStatus(Store::revealFile(partPath)
                                          ? "已在资源管理器中定位临时文件"
                                          : "定位失败（临时文件可能还未落盘）");
                if (ImGui::MenuItem("彻底删除（取消下载）…")) ask = true;
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
        ImGui::TextUnformatted("日期");
        ImGui::SameLine();
        UI::DatePicker::button(year, month, day);

        if (quick) {
            ImGui::SameLine();
            if (ImGui::SmallButton(quickLabel)) quick(year, month, day);
        }
        const int doy = YearMonthDay{year, month, day}.toDOY();
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

    void typeChips(std::vector<unsigned char> &on) {
        if (on.size() != static_cast<std::size_t>(kDateTypeCount)) on.assign(kDateTypeCount, 1);
        ImGui::TextUnformatted("产品类型");
        chipWrap([](const int i) { return kTypeLabels[i]; }, on);
    }

    bool progressLine(const Fetcher::Snapshot &snap) {
        const auto [active, done, total, status] = Fetcher::progress();
        if (snap.scanning) {
            ImGui::Text("扫描中 %d/%d", snap.scanned, snap.total);
            if (ImGui::IsItemHovered()) {
                ImGui::BeginTooltip();
                ImGui::TextUnformatted("正在读取的目录");
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
            ImGui::Text("下载中  已完成 %d/%d  %s", done, total, status.c_str());
            return true;
        }
        return false;
    }

    void failedSources(const Fetcher::Snapshot &snap) {
        if (snap.scanning || snap.failed <= 0) return;
        ImGui::SameLine();
        ImGui::TextColored(UI::kColFail, "· %d 路目录没取回", snap.failed);
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
        if (ImGui::SmallButton("重试")) Fetcher::retryFailed();
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
        std::string s = verb;
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
            ImGui::TextDisabled("%s", opt.scanning ? "扫描中…" : "当前筛选下没有匹配的文件。");
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
            for (int t = 0; t < kDateTypeCount; ++t) groups[t].first = kTypeLabels[t];
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
                const ProductMeta &meta = m.m;
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
                    std::string size = totalBytes > 0.0
                                           ? "大小 " + sizeText(static_cast<long long>(totalBytes))
                                           : std::string("总量未知");
                    size += " · 已下载 " + sizeText(static_cast<long long>(gotBytes));
                    if (speed > 0.0) size += " · " + sizeText(static_cast<long long>(speed)) + "/s";
                    if (speed > 0.0 && totalBytes > gotBytes) {
                        const double left = (totalBytes - gotBytes) / speed;
                        size += left >= 3600.0 ? " · 约 " + std::to_string(static_cast<int>(left / 3600.0)) + " 小时"
                                : left >= 60.0 ? " · 约 " + std::to_string(static_cast<int>(left / 60.0)) + " 分钟"
                                               : " · 约 " + std::to_string(static_cast<int>(left)) + " 秒";
                    }
                    const std::string tip = label + "\n" + size + "\n下载中…（左键暂停 · 右键管理）";
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
            if (narrowed) ImGui::Text("共 %d / %d 个%s", res.shown, res.total, noun);
            else ImGui::Text("共 %d 个%s", res.shown, noun);
        }
        failedSources(snap);
        return res;
    }
}
