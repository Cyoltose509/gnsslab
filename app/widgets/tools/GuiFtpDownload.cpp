#include "GuiFtpDownload.h"

#include "imgui.h"
#include "download/FtpClient.h"
#include "download/ProductMeta.h"
#include "download/DownloadStore.h"

#include "Log.h"
#include <algorithm>

#include <filesystem>

#include <set>
#include <string>
#include <thread>
#include <vector>

namespace Tools::FtpDownload {
    bool s_open = false;


    static void markDownloading(const std::string &remote) {
        std::lock_guard lk(s_mtx);
        s_dlActive[remote] = DlState{};
    }


    static bool leapYear(const int y) { return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0; }

    static int daysInMonth(const int y, const int m) {
        static const int d[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        if (m < 1 || m > 12) return 31;
        return m == 2 && leapYear(y) ? 29 : d[m - 1];
    }

    static void today(int &y, int &m, int &d) {
        const std::time_t t = std::time(nullptr);
        std::tm tm{};
#if defined(_WIN32)
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        y = tm.tm_year + 1900;
        m = tm.tm_mon + 1;
        d = tm.tm_mday;
    }

    static bool tagChip(const char *label, const bool on) {
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

    static float chipWidth(const char *label) {
        return ImGui::CalcTextSize(label).x + ImGui::GetStyle().FramePadding.x * 2.0f;
    }

    static void runScan(std::vector<std::string> urls) {
        try {
            for (size_t i = 0; i < urls.size(); ++i) {
                const std::string &u = urls[i];
                {
                    std::lock_guard lk(s_mtx);
                    s_status = "读取目录 " + u;
                }
                std::string err;
                DirCache dc;
                if (std::vector<std::string> raw; FtpClient::listDir(u, raw, err)) {
                    for (const auto &n: raw) {
                        if (auto m = ProductMeta::parse(n); m && m->type != ProductType::Unknown)
                            dc.items.push_back(*m);
                    }
                    dc.loaded = true;
                } else {
                    dc.err = err;
                    dc.missing = true;
                }
                std::lock_guard lk(s_mtx);
                s_dirs[u] = std::move(dc);
                s_done = static_cast<int>(i) + 1;
            }
        } catch (const std::exception &e) {
            std::lock_guard lk(s_mtx);
            s_status = std::string("扫描异常中断: ") + e.what();
            s_scanning = false;
            return;
        } catch (...) {
            std::lock_guard lk(s_mtx);
            s_status = "扫描异常中断";
            s_scanning = false;
            return;
        }
        {
            std::lock_guard lk(s_mtx);
            s_status = "就绪";
            s_scanning = false;
        }
    }

    static void runDownload(const std::string &dirUrl, const ProductMeta &m) {
        const std::string remote = dirUrl + m.filename;
        const std::string local = DownloadStore::localPath(m.type, m.year, m.doy, m.filename);
        std::error_code ec;
        std::filesystem::create_directories(DownloadStore::dirFor(m.type, m.year, m.doy), ec);

        LOG_INFO << "[下载] 开始 " << m.filename << " ← " << dirUrl;
        markDownloading(remote);
        auto onProgress = [&remote](const double total, const double now) {
            float f = -1.0f;
            if (total > 0.0) {
                const double r = now / total;
                f = static_cast<float>(r < 0.0 ? 0.0 : r > 1.0 ? 1.0 : r);
            }
            std::lock_guard lk(s_mtx);
            if (const auto it = s_dlActive.find(remote); it != s_dlActive.end()) it->second.frac = f;
        };

        std::string log;
        const bool ok = FtpClient::downloadFile(remote, local, onProgress,
                                         [&](const std::string &s) { log = s; }, 3);
        if (ok) {
            const std::string finalPath = DownloadStore::decompressIfNeeded(local);
            const std::string finalName = std::filesystem::path(finalPath).filename().string();
            DownloadEntry e;
            e.type = ProductMeta::typeDir(m.type);
            e.year = m.year;
            e.doy = m.doy;
            e.datacenter = m.datacenter;
            e.file = finalName;
            e.path = finalPath;
            std::error_code sec;
            e.size = static_cast<long long>(std::filesystem::file_size(finalPath, sec));
            DownloadStore::record(e);
            log = "完成: " + finalName;
        }

        {
            std::lock_guard lk(s_mtx);
            s_dlActive.erase(remote);
            s_status = (ok ? "下载成功  " : "下载失败  ") + log;
            s_downloading = false;
        }
        if (ok)
            LOG_INFO << "[下载] 完成 " << m.filename;
        else
            LOG_ERROR << "[下载] 失败 " << m.filename << " — " << log;
    }

    static void startDownload(const Pending &p) {
        s_downloading = true;
        {
            std::lock_guard lk(s_mtx);
            s_status = "下载中…";
        }
        ProductMeta m;
        m.type = p.type;
        m.year = p.year;
        m.doy = p.doy;
        m.spanDays = p.spanDays;
        m.filename = p.remoteName;
        markDownloading(p.dirUrl + p.remoteName);
        if (s_dlThread.joinable()) s_dlThread.join();
        s_dlThread = std::thread(runDownload, p.dirUrl, m);
    }

    struct Match {
        ProductMeta m;
        std::string dirUrl;
    };

    static std::vector<Match> filterMatches(const std::vector<std::string> &dirs,
                                            const std::set<ProductType> &types,
                                            const int year, const int doy) {
        std::set<std::string> dcs;
        for (size_t i = 0; i < s_dcOn.size(); ++i)
            if (s_dcOn[i]) dcs.insert(ProductMeta::knownDatacenters()[i]);

        std::vector<Match> out;
        std::set<std::string> seen;
        for (const auto &d: dirs) {
            auto it = s_dirs.find(d);
            if (it == s_dirs.end() || !it->second.loaded) continue;
            for (const auto &m: it->second.items) {
                if (!types.count(m.type)) continue;
                if (!m.covers(year, doy)) continue;
                if (m.type != ProductType::NAV && !dcs.empty() && !dcs.count(m.datacenter)) continue;
                if (!seen.insert(m.filename).second) continue;
                out.push_back({m, d});
            }
        }
        std::sort(out.begin(), out.end(),
                  [](const Match &a, const Match &b) { return a.m.filename < b.m.filename; });
        return out;
    }

    void Render(bool *p_open) {
        if (!s_open) return;

        if (s_year == 0) today(s_year, s_month, s_day);
        if (s_dcOn.size() != ProductMeta::knownDatacenters().size())
            s_dcOn.assign(ProductMeta::knownDatacenters().size(), 0);

        ImGui::SetNextWindowSize(ImVec2(900, 560), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("产品下载", p_open)) {
            ImGui::End();
            return;
        }

        {
            ImGui::TextUnformatted("产品类型");
            ImGui::SameLine();
            ImGui::TextDisabled("可多选");
            float x = 0.0f;
            const float avail = ImGui::GetContentRegionAvail().x;
            const float gap = ImGui::GetStyle().ItemSpacing.x;
            for (int i = 0; i < 8; ++i) {
                const float w = chipWidth(TYPE_LABELS[i]);
                if (x > 0.0f && x + w > avail) x = 0.0f;
                if (x > 0.0f) ImGui::SameLine();
                if (tagChip(TYPE_LABELS[i], s_typeOn[i])) s_typeOn[i] = !s_typeOn[i];
                x += w + gap;
            }
            if (s_typeOn[7]) {
                ImGui::SameLine();
                ImGui::TextDisabled("(镜像无 ATX 源)");
            }
        }

        {
            const auto &dcs = ProductMeta::knownDatacenters();
            ImGui::TextUnformatted("数据中心");
            ImGui::SameLine();
            ImGui::TextDisabled("不选 = 不限");
            float x = 0.0f;
            const float avail = ImGui::GetContentRegionAvail().x;
            const float gap = ImGui::GetStyle().ItemSpacing.x;
            for (size_t i = 0; i < dcs.size(); ++i) {
                const float w = chipWidth(dcs[i].c_str());
                if (x > 0.0f && x + w > avail) x = 0.0f;
                if (x > 0.0f) ImGui::SameLine();
                if (tagChip(dcs[i].c_str(), s_dcOn[i] != 0)) s_dcOn[i] = s_dcOn[i] ? 0 : 1;
                x += w + gap;
            }
        }

        {
            ImGui::TextUnformatted("日期");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(90);
            if (ImGui::InputInt("##y", &s_year, 0, 0)) {
                if (s_year < 1980) s_year = 1980;
                if (s_year > 2099) s_year = 2099;
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60);
            if (ImGui::InputInt("##m", &s_month, 0, 0)) {
                s_month = std::clamp(s_month, 1, 12);
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60);
            if (ImGui::InputInt("##d", &s_day, 0, 0)) {
                const int mx = daysInMonth(s_year, s_month);
                s_day = std::clamp(s_day, 1, mx);
            }
            s_day = std::clamp(s_day, 1, daysInMonth(s_year, s_month));

            ImGui::SameLine();
            if (ImGui::SmallButton("今天")) today(s_year, s_month, s_day);

            const int doy = ProductMeta::dateToDoy(s_year, s_month, s_day);
            ImGui::SameLine();
            ImGui::TextDisabled("年积日 %03d   第 %d 周", doy, ProductMeta::gpsWeek(s_year, doy));

            ImGui::SameLine();
            ImGui::BeginDisabled(s_scanning.load());
            if (ImGui::SmallButton("刷新目录")) {
                std::lock_guard lk(s_mtx);
                s_dirs.clear();
            }
            ImGui::EndDisabled();
        }

        ImGui::Separator();

        std::set<ProductType> types;
        for (int i = 0; i < 8; ++i)
            if (s_typeOn[i]) types.insert(static_cast<ProductType>(i));

        const int doy = ProductMeta::dateToDoy(s_year, s_month, s_day);
        std::vector<std::string> dirs = ProductMeta::scanDirs(types, s_year, doy);
        {
            std::lock_guard lk(s_mtx);
            std::vector<std::string> missing;
            for (const auto &d: dirs)
                if (!s_dirs.count(d)) missing.push_back(d);
            if (!missing.empty() && !s_scanning) {
                s_scanning = true;
                s_done = 0;
                s_total = static_cast<int>(missing.size());
                s_status = "读取目录…";
                s_scanThread = std::thread(runScan, std::move(missing));
            }
        }

        std::vector<Match> matches;
        int nMissing = 0;
        std::vector<std::string> missingUrls;
        {
            std::lock_guard lk(s_mtx);
            matches = filterMatches(dirs, types, s_year, doy);
            for (const auto &d: dirs) {
                if (auto it = s_dirs.find(d); it != s_dirs.end() && it->second.missing) {
                    ++nMissing;
                    missingUrls.push_back(d);
                }
            }
        }

        ImGui::BeginChild("##files", ImVec2(0, -30), ImGuiChildFlags_Borders);
        if (types.empty()) {
            ImGui::TextDisabled("先勾选上面的产品类型。");
        } else if (dirs.empty()) {
            ImGui::TextDisabled("所选类型在该镜像上没有对应的产品目录。");
        } else if (matches.empty()) {
            if (nMissing > 0) {
                ImGui::TextDisabled("所选日期的部分目录尚未发布（当天/近几日的产品可能还没上架）：");
                for (const auto &u: missingUrls)
                    ImGui::TextDisabled("  • %s", u.c_str());
                ImGui::TextDisabled("已发布的产品会在目录出现后自动显示；点“刷新目录”可重新探测。");
            } else {
                ImGui::TextDisabled("该日期目录下没有匹配文件（数据中心标签可能把结果滤空了，或所选类型在该日无产品）。");
            }
        } else {
            const float avail = ImGui::GetContentRegionAvail().x;
            const float gap = ImGui::GetStyle().ItemSpacing.x;

            std::vector<std::vector<const Match *> > byType(8);
            for (const auto &mt: matches) {
                if (const int t = static_cast<int>(mt.m.type); t >= 0 && t < 8) byType[t].push_back(&mt);
            }

            bool firstGroup = true;
            for (int t = 0; t < 8; ++t) {
                if (byType[t].empty()) continue;
                if (!firstGroup) ImGui::Spacing();
                firstGroup = false;
                ImGui::TextDisabled("%s", TYPE_LABELS[t]);

                float x = 0.0f;
                for (const Match *pmt: byType[t]) {
                    const auto &[m, dirUrl] = *pmt;
                    const std::string local =
                            DownloadStore::findLocalFile(m.type, m.year, m.doy, m.filename);
                    const std::string label = m.filename;

                    float frac;
                    bool downloading = false;
                    {
                        std::lock_guard lk(s_mtx);
                        if (auto it = s_dlActive.find(dirUrl + m.filename); it != s_dlActive.end()) {
                            downloading = true;
                            frac = it->second.frac;
                        }
                    }


                    const float w = downloading
                                        ? chipWidth((label + "  100%").c_str())
                                        : chipWidth(label.c_str());
                    if (x > 0.0f && x + w > avail) x = 0.0f;
                    if (x > 0.0f) ImGui::SameLine();

                    ImGui::PushID(label.c_str());
                    if (downloading) {
                        char pct[16];
                        if (frac < 0.0f) std::snprintf(pct, sizeof(pct), "…");
                        else std::snprintf(pct, sizeof(pct), "%.0f%%", frac * 100.0f);
                        const std::string ov = label + "  " + pct;
                        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 9.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(9.0f, 4.0f));
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                        ImGui::ProgressBar(frac < 0.0f ? 0.0f : frac, ImVec2(w, 0.0f), ov.c_str());
                        ImGui::PopStyleColor();
                        ImGui::PopStyleVar(2);
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("%s\n下载中…（完成后自动变绿）", label.c_str());
                    } else {
                        const bool dl = !local.empty();
                        if (dl) // 已下载：绿字
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.45f, 0.90f, 0.50f, 1.0f));
                        const bool pressed = ImGui::Button(label.c_str());
                        if (dl) ImGui::PopStyleColor();
                        if (ImGui::IsItemHovered()) {
                            const std::string tip =
                                    label + "\n覆盖 " + std::to_string(m.spanDays) + " 天（起始 " +
                                    ProductMeta::doyToDateStr(m.year, m.doy) + "）  采样 " +
                                    (m.interval.empty() ? "-" : m.interval) +
                                    (dl
                                         ? "\n本地：" + local + "\n（点击可重新下载 / 删除）"
                                         : "\n本地：无（点击下载）");
                            ImGui::SetTooltip("%s", tip.c_str());
                        }
                        if (pressed) {
                            s_pending = Pending{
                                m.filename, dirUrl, m.type, m.year, m.doy,
                                m.spanDays, local
                            };
                            s_pendingOpen = true;
                        }
                    }
                    ImGui::PopID();
                    x += ImGui::GetItemRectSize().x + gap;
                }
            }
        }
        ImGui::EndChild();

        // ---- 状态 / 进度 ----
        {
            std::lock_guard lk(s_mtx);
            if (s_scanning)
                ImGui::Text("扫描中 %d/%d  %s", s_done.load(), s_total.load(), s_status.c_str());
            else {
                const std::string suffix = nMissing > 0
                                               ? "（" + std::to_string(nMissing) + " 个目录未发布）"
                                               : "";
                ImGui::Text("共 %zu 个匹配文件%s   %s",
                            matches.size(), suffix.c_str(), s_status.c_str());
            }
        }

        // ---- 下载 / 删除确认 ----
        if (s_pendingOpen) {
            ImGui::OpenPopup("##fileAction");
            s_pendingOpen = false;
        }
        if (ImGui::BeginPopupModal("##fileAction", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextWrapped("%s", s_pending.remoteName.c_str());
            if (s_pending.localPath.empty()) {
                ImGui::TextDisabled("本地尚无此文件，是否下载？");
                if (!s_downloading && ImGui::Button("下载")) {
                    startDownload(s_pending);
                    ImGui::CloseCurrentPopup();
                }
            } else {
                ImGui::TextDisabled("本地已有：\n%s", s_pending.localPath.c_str());
                if (!s_downloading && ImGui::Button("重新下载")) {
                    startDownload(s_pending);
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("删除本地")) {
                    {
                        const bool gone = DownloadStore::removeFile(s_pending.localPath);
                        std::lock_guard lk(s_mtx);
                        s_status = gone ? "已删除本地文件" : "删除失败（文件可能被占用）";
                    }
                    ImGui::CloseCurrentPopup();
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("取消")) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }

        ImGui::End();
    }

    void RequestAutoDownload(ProductType type, const int year, const int doy, const std::string &datacenter) {
        s_open = true;
        LOG_INFO << "[下载] 面板自动定位: " << ProductMeta::typeDir(type) << " "
                << ProductMeta::doyToDateStr(year, doy) << "  数据中心="
                << (datacenter.empty() ? std::string("不限") : datacenter);

        for (int i = 0; i < 8; ++i) s_typeOn[i] = static_cast<int>(type) == i;


        if (int y = year, m = 0, d = 0;ProductMeta::doyToYmd(year, doy, y, m, d)) {
            s_year = y;
            s_month = m;
            s_day = d;
        }

        s_dcOn.assign(ProductMeta::knownDatacenters().size(), 0);
        const auto &dcs = ProductMeta::knownDatacenters();
        for (size_t i = 0; i < dcs.size(); ++i)
            if (dcs[i] == datacenter) s_dcOn[i] = 1;
    }

    void Shutdown() {
        if (s_scanThread.joinable()) s_scanThread.join();
        if (s_dlThread.joinable()) s_dlThread.join();
    }
}
