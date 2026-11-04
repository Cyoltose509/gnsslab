#include "GuiFtpDownload.h"

#include "imgui.h"
#include "ui/GuiColors.h"
#include "RemoteClient.h"
#include "download/ProductMeta.h"
#include "download/DownloadStore.h"
#include "download/ProductCatalog.h"
#include "download/ProductStore.h"
#include "download/AutoSync.h"
#include "core/PathMemory.h"
#include "core/Products.h"
#include "ui/Gui.h"
#include "StringUtils.h"
#include "TimeConvert.h"

#include "Log.h"
#include <algorithm>

#include <cstdio>
#include <filesystem>

#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace Download::Panel {
    bool s_open = false;

    static int s_obsYear = 0, s_obsMonth = 0, s_obsDay = 0;
    static std::set<std::string> s_obsSites;
    static char s_obsSiteFind[32] = "";
    static int s_dlStateFilter = 0;


    static void markDownloading(const std::string &remote) {
        std::lock_guard lk(s_mtx);
        s_dlActive[remote] = DlState{};
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


    static void pushRemoteChipStyle() {
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

    static void popRemoteChipStyle() { ImGui::PopStyleColor(4); }

    static bool passStateFilter(const int f, const bool hasLocal) {
        if (f == 1) return hasLocal;
        if (f == 2) return !hasLocal;
        return true;
    }

    static void stateFilterChips(const char *scope, int *v) {
        static const char *labels[3] = {"全部", "已下载", "未下载"};
        ImGui::PushID(scope);
        for (int i = 0; i < 3; ++i) {
            if (i) ImGui::SameLine();
            if (tagChip(labels[i], *v == i)) *v = i;
        }
        ImGui::PopID();
    }

    static const char *typeLabel(const ProductType t) {
        if (const int i = static_cast<int>(t); i >= 0 && i < kDateTypeCount) return TYPE_LABELS[i];
        if (t == ProductType::OBS) return "观测文件";
        return "未识别";
    }

    static void runScan(std::vector<std::string> urls) {
        try {
            for (std::size_t i = 0; i < urls.size(); ++i) {
                const std::string &u = urls[i];
                {
                    std::lock_guard lk(s_mtx);
                    s_status = "读取目录 " + u;
                }
                std::string err;
                DirCache dc;
                if (std::vector<std::string> raw; RemoteClient::listDir(u, raw, err)) {
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

    static void runDownload(const std::string &dirUrl, const ProductMeta &m) {
        const std::string remote = dirUrl + m.filename;
        const std::string local = DownloadStore::localPath(m.type, m.year, m.doy, m.filename);
        const std::string part = local + ".part";
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
        bool ok = RemoteClient::downloadFile(remote, part, onProgress,
                                             [&](const std::string &s) { log = s; }, 3);
        if (ok) {
            std::error_code mvEc;
            std::filesystem::rename(part, local, mvEc);
            if (mvEc) {
                // 跨卷 / 目标已存在等罕见情况：拷一份再删
                std::error_code cpEc;
                std::filesystem::copy_file(part, local, std::filesystem::copy_options::overwrite_existing, cpEc);
                if (!cpEc) std::filesystem::remove(part, mvEc);
                if (cpEc) {
                    log = "改名失败: " + cpEc.message();
                    ok = false;
                }
            }
        }
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

        s_dlActiveCount.fetch_sub(1);
        {
            std::lock_guard lk(s_mtx);
            s_dlActive.erase(remote);
            s_status = (ok ? "下载成功  " : "下载失败  ") + log;
            if (s_dlActiveCount.load() == 0) s_downloading = false;
        }
        if (ok)
            LOG_INFO << "[下载] 完成 " << m.filename;
        else
            LOG_ERROR << "[下载] 失败 " << m.filename << " — " << log;
    }

    static ProductMeta metaFromPending(const Pending &p) {
        ProductMeta m;
        m.type = p.type;
        m.year = p.year;
        m.doy = p.doy;
        m.spanDays = p.spanDays;
        m.filename = p.remoteName;
        return m;
    }

    static void dlWorker() {
        for (;;) {
            Pending p;
            {
                std::unique_lock lk(s_dlQmtx);
                s_dlCv.wait(lk, [] { return s_dlStop.load() || !s_dlQueue.empty(); });
                if (s_dlStop.load() && s_dlQueue.empty()) return;
                p = std::move(s_dlQueue.front());
                s_dlQueue.pop_front();
            }
            runDownload(p.dirUrl, metaFromPending(p));
            s_dlDone.fetch_add(1);
        }
    }

    static void ensureDlWorkers() {
        if (s_dlStarted) return;
        s_dlStarted = true;
        s_dlWorkers.reserve(kDlWorkers);
        for (int i = 0; i < kDlWorkers; ++i)
            s_dlWorkers.emplace_back(dlWorker);
    }

    static void enqueueDownload(const Pending &p) {
        s_downloading = true;
        {
            std::lock_guard lk(s_mtx);
            s_status = "下载中…";
        }
        {
            std::lock_guard lk(s_dlQmtx);
            s_dlQueue.push_back(p);
            s_dlTotal.fetch_add(1);
            s_dlActiveCount.fetch_add(1);
        }
        s_dlCv.notify_one();
        ensureDlWorkers();
    }

    struct Match {
        ProductMeta m;
        std::string dirUrl;
        std::string localPath;   // 仅「本地独有」项填：真实路径（远端名对不上时靠它）
    };

    static std::vector<Match> filterMatches(const std::vector<std::string> &dirs,
                                            const std::set<ProductType> &types,
                                            const int year, const int doy) {
        std::set<std::string> dcs;
        for (std::size_t i = 0; i < s_dcOn.size(); ++i)
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

    static const std::vector<LocalFile> &cachedLocal(const ProductType t, const int year, const int doy) {
        using Stamp = std::filesystem::file_time_type;
        static const std::vector<LocalFile> empty;
        static std::map<std::string, std::pair<Stamp, std::vector<LocalFile> > > cache;

        const std::string dir = DownloadStore::dirFor(t, year, doy);
        std::error_code ec;
        const Stamp mt = std::filesystem::last_write_time(dir, ec);
        if (ec) return empty;

        if (const auto it = cache.find(dir); it != cache.end() && it->second.first == mt)
            return it->second.second;

        auto &[fst, snd] = cache[dir];
        fst = mt;
        snd = DownloadStore::listLocal(t, year, doy);
        return snd;
    }

    static void appendLocalOnly(std::vector<Match> &list, const ProductType t,
                                const int year, const int doy) {
        std::set<std::string> have;
        for (const auto &mt: list) have.insert(DownloadStore::localNameFor(mt.m.filename));

        for (const auto &lf: cachedLocal(t, year, doy)) {
            if (!have.insert(DownloadStore::localNameFor(lf.name)).second) continue;
            Match mt;
            mt.m.type = t;
            mt.m.year = year;
            mt.m.doy = doy;
            mt.m.filename = lf.name;
            mt.localPath = lf.path;
            list.push_back(std::move(mt));
        }
    }

    enum class FileAction { None, Reveal, Delete };

    static FileAction fileContextMenu(const std::string &localPath) {
        auto act = FileAction::None;
        if (ImGui::BeginPopupContextItem("##fileCtx")) {
            if (ImGui::MenuItem("从资源管理器打开")) act = FileAction::Reveal;
            if (ImGui::MenuItem("删除该文件")) act = FileAction::Delete;
            ImGui::EndPopup();
        }
        if (act == FileAction::Delete) {
            s_confirmDelPath = localPath;
            s_confirmDelOpen = true;
        } else if (act == FileAction::Reveal) {
            DownloadStore::revealFile(localPath);
        }
        return act;
    }

    static void renderDeleteConfirm() {
        if (s_confirmDelOpen) {
            ImGui::OpenPopup("##confirmDel");
            s_confirmDelOpen = false;
        }
        if (ImGui::BeginPopupModal("##confirmDel", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextUnformatted("是否删除该文件？");
            ImGui::TextDisabled("%s", s_confirmDelPath.c_str());
            ImGui::Spacing();
            if (ImGui::Button("删除", ImVec2(90, 0))) {
                {
                    const bool gone = DownloadStore::removeFile(s_confirmDelPath);
                    std::lock_guard lk(s_mtx);
                    s_status = gone ? "已删除本地文件" : "删除失败（文件可能被占用）";
                }
                s_confirmDelPath.clear();
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("取消", ImVec2(90, 0))) {
                s_confirmDelPath.clear();
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }

    void renderDateTab() {
        if (s_year == 0) today(s_year, s_month, s_day);
        if (s_dcOn.size() != ProductMeta::knownDatacenters().size())
            s_dcOn.assign(ProductMeta::knownDatacenters().size(), 0);

        {
            ImGui::TextUnformatted("产品类型");
            ImGui::SameLine();
            ImGui::TextDisabled("可多选");
            float x = 0.0f;
            const float avail = ImGui::GetContentRegionAvail().x;
            const float gap = ImGui::GetStyle().ItemSpacing.x;
            for (int i = 0; i < kDateTypeCount; ++i) {
                const float w = chipWidth(TYPE_LABELS[i]);
                if (x > 0.0f && x + w > avail) x = 0.0f;
                if (x > 0.0f) ImGui::SameLine();
                if (tagChip(TYPE_LABELS[i], s_typeOn[i])) s_typeOn[i] = !s_typeOn[i];
                x += w + gap;
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
            for (std::size_t i = 0; i < dcs.size(); ++i) {
                const float w = chipWidth(dcs[i].c_str());
                if (x > 0.0f && x + w > avail) x = 0.0f;
                if (x > 0.0f) ImGui::SameLine();
                if (tagChip(dcs[i].c_str(), s_dcOn[i] != 0)) s_dcOn[i] = s_dcOn[i] ? 0 : 1;
                x += w + gap;
            }
        }

        {
            ImGui::TextUnformatted("本地状态");
            ImGui::SameLine();
            stateFilterChips("##dateState", &s_dlStateFilter);
            ImGui::SameLine();
            ImGui::TextDisabled("已下载＝本地已有，未下载＝远端才有");
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
                const int mx = Time::daysInMonth(s_year, s_month);
                s_day = std::clamp(s_day, 1, mx);
            }
            s_day = std::clamp(s_day, 1, Time::daysInMonth(s_year, s_month));

            ImGui::SameLine();
            if (ImGui::SmallButton("今天")) today(s_year, s_month, s_day);

            const int doy = Time::YMDToDOY({s_year, s_month, s_day});
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
        for (int i = 0; i < kDateTypeCount; ++i)
            if (s_typeOn[i]) types.insert(static_cast<ProductType>(i));

        const int doy = Time::YMDToDOY({s_year, s_month, s_day});
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
                if (s_scanThread.joinable()) s_scanThread.join();
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

        for (const auto t: types) appendLocalOnly(matches, t, s_year, doy);

        int nAllMatches = static_cast<int>(matches.size());
        if (s_dlStateFilter != 0) {
            std::vector<Match> keep;
            keep.reserve(matches.size());
            for (const auto &mt: matches) {
                const bool has = !mt.localPath.empty() ||
                                 !DownloadStore::findLocalFile(mt.m.type, mt.m.year, mt.m.doy,
                                                               mt.m.filename).empty();
                if (passStateFilter(s_dlStateFilter, has)) keep.push_back(mt);
            }
            matches.swap(keep);
        }

        ImGui::BeginChild("##files", ImVec2(0, -30), ImGuiChildFlags_Borders);
        if (types.empty()) {
            ImGui::TextDisabled("先勾选上面的产品类型。");
        } else if (matches.empty()) {
            if (dirs.empty())
                ImGui::TextDisabled("所选类型在该镜像上没有对应的产品目录。");
            else if (nAllMatches > 0)
                ImGui::TextDisabled("该日期的 %d 个文件都被「本地状态」筛掉了（换「全部」看看）。",
                                    nAllMatches);
            else if (nMissing > 0) {
                ImGui::TextDisabled("所选日期的部分目录尚未发布：");
                for (const auto &u: missingUrls)
                    ImGui::TextDisabled("  • %s", u.c_str());
            } else {
                ImGui::TextDisabled("该日期目录下没有匹配文件。");
            }
        } else {
            const float avail = ImGui::GetContentRegionAvail().x;
            const float gap = ImGui::GetStyle().ItemSpacing.x;

            std::vector<std::vector<const Match *> > byType(kDateTypeCount);
            for (const auto &mt: matches) {
                if (const int t = static_cast<int>(mt.m.type); t >= 0 && t < kDateTypeCount) byType[t].push_back(&mt);
            }

            bool firstGroup = true;
            for (int t = 0; t < kDateTypeCount; ++t) {
                if (byType[t].empty()) continue;
                if (!firstGroup) ImGui::Spacing();
                firstGroup = false;
                ImGui::TextDisabled("%s", TYPE_LABELS[t]);

                float x = 0.0f;
                for (const Match *pmt: byType[t]) {
                    const ProductMeta &m = pmt->m;
                    const std::string &dirUrl = pmt->dirUrl;
                    const std::string local = !pmt->localPath.empty()
                                                  ? pmt->localPath
                                                  : DownloadStore::findLocalFile(m.type, m.year, m.doy,
                                                                                 m.filename);
                    const std::string label = DownloadStore::localNameFor(m.filename);
                    const bool remoteKnown = !dirUrl.empty();

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
                        const std::string tip = label + "\n下载中…（完成后自动变绿）";
                        downloadChip(label.c_str(), frac, w, tip.c_str());
                    } else {
                        const bool dl = !local.empty();
                        if (dl)
                            ImGui::PushStyleColor(ImGuiCol_Text, UI::kColPass);
                        else
                            pushRemoteChipStyle();
                        const bool pressed = ImGui::Button(label.c_str());
                        if (dl) ImGui::PopStyleColor();
                        else popRemoteChipStyle();
                        if (ImGui::IsItemHovered()) {
                            std::string tip = label;
                            if (!dl) {
                                tip += "\n覆盖 " + std::to_string(m.spanDays) + " 天（起始 " +
                                       ProductMeta::doyToDateStr(m.year, m.doy) + "）  采样 " +
                                       (m.interval.empty() ? "-" : m.interval) +
                                       "\n本地：无（点击下载）";
                            } else if (remoteKnown) {
                                tip += "\n覆盖 " + std::to_string(m.spanDays) + " 天（起始 " +
                                       ProductMeta::doyToDateStr(m.year, m.doy) + "）  采样 " +
                                       (m.interval.empty() ? "-" : m.interval) +
                                       "\n本地：" + local + "\n（点击可重新下载 · 右键管理）";
                            } else {
                                tip += "\n本地：" + local +
                                       "\n远端列表未列出（仅本地）\n（右键可打开 / 删除）";
                            }
                            ImGui::SetTooltip("%s", tip.c_str());
                        }
                        if (dl) fileContextMenu(local);

                        if (pressed && remoteKnown) {
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


        {
            std::lock_guard lk(s_mtx);
            if (s_scanning)
                ImGui::Text("扫描中 %d/%d  %s", s_done.load(), s_total.load(), s_status.c_str());
            else if (s_dlActiveCount.load() > 0)
                ImGui::Text("下载中  已完成 %d/%d  %s",
                            s_dlDone.load(), s_dlTotal.load(), s_status.c_str());
            else {
                const std::string suffix = nMissing > 0
                                               ? "（" + std::to_string(nMissing) + " 个目录未发布）"
                                               : "";
                if (s_dlStateFilter != 0)
                    ImGui::Text("共 %zu / %d 个匹配文件（已按本地状态筛选）%s   %s",
                                matches.size(), nAllMatches, suffix.c_str(), s_status.c_str());
                else
                    ImGui::Text("共 %zu 个匹配文件%s   %s",
                                matches.size(), suffix.c_str(), s_status.c_str());
            }
            if (!matches.empty()) {
                ImGui::SameLine();
                if (ImGui::SmallButton(s_dlStateFilter == 2
                                           ? "下载列出的全部##dlAll"
                                           : "下载全部匹配##dlAll")) {
                    for (const auto &mt: matches) {
                        const ProductMeta &m = mt.m;
                        const std::string &dirUrl = mt.dirUrl;
                        if (dirUrl.empty()) continue;   // 仅本地的项没有下载地址
                        if (!DownloadStore::findLocalFile(m.type, m.year, m.doy, m.filename).empty())
                            continue; // 已下过就跳过
                        Pending p{m.filename, dirUrl, m.type, m.year, m.doy, m.spanDays, ""};
                        enqueueDownload(p);
                    }
                }
            }
        }


        if (s_pendingOpen) {
            ImGui::OpenPopup("##fileAction");
            s_pendingOpen = false;
        }
        if (ImGui::BeginPopupModal("##fileAction", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextWrapped("%s", DownloadStore::localNameFor(s_pending.remoteName).c_str());
            if (s_pending.localPath.empty()) {
                ImGui::TextDisabled("本地尚无此文件，是否下载？");
                if (ImGui::Button("下载")) {
                    enqueueDownload(s_pending);
                    ImGui::CloseCurrentPopup();
                }
            } else {
                ImGui::TextDisabled("本地已有：\n%s", s_pending.localPath.c_str());
                if (ImGui::Button("重新下载")) {
                    enqueueDownload(s_pending);
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("删除本地")) {
                    ImGui::CloseCurrentPopup();
                    s_confirmDelPath = s_pending.localPath;
                    s_confirmDelOpen = true;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("取消")) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
    }

    static std::string obsStationOf(const std::string &fn) {
        return Str::toUpper(fn.substr(0, 4));
    }

    void renderObsTab() {
        if (s_obsYear == 0) today(s_obsYear, s_obsMonth, s_obsDay);

        {
            ImGui::TextUnformatted("日期");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(90);
            if (ImGui::InputInt("##oy", &s_obsYear, 0, 0)) s_obsYear = std::clamp(s_obsYear, 1980, 2099);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60);
            if (ImGui::InputInt("##om", &s_obsMonth, 0, 0)) s_obsMonth = std::clamp(s_obsMonth, 1, 12);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60);
            if (ImGui::InputInt("##od", &s_obsDay, 0, 0))
                s_obsDay = std::clamp(s_obsDay, 1, Time::daysInMonth(s_obsYear, s_obsMonth));
            s_obsDay = std::clamp(s_obsDay, 1, Time::daysInMonth(s_obsYear, s_obsMonth));
            ImGui::SameLine();
            if (ImGui::SmallButton("今天##otoday")) today(s_obsYear, s_obsMonth, s_obsDay);
            const int doy = Time::YMDToDOY({s_obsYear, s_obsMonth, s_obsDay});
            ImGui::SameLine();
            ImGui::TextDisabled("年积日 %03d", doy);
        }

        {
            ImGui::TextUnformatted("本地状态");
            ImGui::SameLine();
            stateFilterChips("##obsState", &s_dlStateFilter);
            ImGui::SameLine();
            ImGui::TextDisabled("已下载＝本地已有，未下载＝远端才有");
        }

        const int doy = Time::YMDToDOY({s_obsYear, s_obsMonth, s_obsDay});

        std::set types{ProductType::OBS};
        const std::vector<std::string> dirs = ProductMeta::scanDirs(types, s_obsYear, doy);
        {
            std::lock_guard lk(s_mtx);
            std::vector<std::string> missing;
            for (const auto &d: dirs)
                if (!s_dirs.count(d)) missing.push_back(d);
            if (!missing.empty() && !s_scanning) {
                s_scanning = true;
                s_done = 0;
                s_total = static_cast<int>(missing.size());
                s_status = "读取观测目录…";
                if (s_scanThread.joinable()) s_scanThread.join();
                s_scanThread = std::thread(runScan, std::move(missing));
            }
        }

        std::vector<Match> all;
        {
            std::lock_guard lk(s_mtx);
            std::set<std::string> seen;
            for (const auto &d: dirs) {
                const auto it = s_dirs.find(d);
                if (it == s_dirs.end() || !it->second.loaded) continue;
                for (const auto &m: it->second.items) {
                    if (m.type != ProductType::OBS) continue;
                    if (!m.covers(s_obsYear, doy)) continue;
                    if (!seen.insert(m.filename).second) continue;
                    all.push_back({m, d});
                }
            }
        }
        appendLocalOnly(all, ProductType::OBS, s_obsYear, doy);
        std::sort(all.begin(), all.end(),
                  [](const Match &a, const Match &b) { return a.m.filename < b.m.filename; });

        std::set<std::string> sites;
        for (const auto &mt: all) sites.insert(obsStationOf(mt.m.filename));

        ImGui::TextUnformatted("站点");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140);
        ImGui::InputTextWithHint("##stFind", "筛选站点…", s_obsSiteFind, sizeof s_obsSiteFind);
        ImGui::SameLine();
        if (ImGui::SmallButton("清空选择##stClear") && !s_obsSites.empty()) s_obsSites.clear();
        ImGui::SameLine();
        ImGui::TextDisabled("已选 %zu / 共 %zu 站 %s", s_obsSites.size(), sites.size(),
                            s_obsSites.empty() ? "（未选＝全部）" : "");

        ImGui::BeginChild("##obsSites", ImVec2(0, 76), ImGuiChildFlags_Borders);
        if (sites.empty()) {
            ImGui::TextDisabled(s_scanning ? "扫描中…" : "该日期没有可选的观测站点");
        } else {
            const std::string find = Str::toUpper(s_obsSiteFind);
            const float avail = ImGui::GetContentRegionAvail().x;
            const float gap = ImGui::GetStyle().ItemSpacing.x;
            float x = 0.0f;
            int shown = 0;
            for (const auto &st: sites) {
                if (!find.empty() && st.find(find) == std::string::npos) continue;
                const bool on = s_obsSites.count(st) > 0;
                if (const float w = chipWidth(st.c_str()); x > 0.0f && x + w > avail) x = 0.0f;
                if (x > 0.0f) ImGui::SameLine();
                if (tagChip(st.c_str(), on)) {
                    if (on) s_obsSites.erase(st);
                    else s_obsSites.insert(st);
                }
                x += ImGui::GetItemRectSize().x + gap;
                ++shown;
            }
            if (shown == 0) ImGui::TextDisabled("没有站点匹配「%s」", s_obsSiteFind);
        }
        ImGui::EndChild();

        // 按已选站点过滤（空＝全部）
        std::vector<Match> matches;
        matches.reserve(all.size());
        for (const auto &mt: all)
            if (s_obsSites.empty() || s_obsSites.count(obsStationOf(mt.m.filename)))
                matches.push_back(mt);

        const int nAllMatches = static_cast<int>(matches.size());
        if (s_dlStateFilter != 0) {
            std::vector<Match> keep;
            keep.reserve(matches.size());
            for (const auto &mt: matches) {
                const bool has = !mt.localPath.empty() ||
                                 !DownloadStore::findLocalFile(ProductType::OBS, mt.m.year, mt.m.doy,
                                                               mt.m.filename).empty();
                if (passStateFilter(s_dlStateFilter, has)) keep.push_back(mt);
            }
            matches.swap(keep);
        }

        ImGui::Separator();

        ImGui::BeginChild("##obsFiles", ImVec2(0, -30), ImGuiChildFlags_Borders);
        if (matches.empty()) {
            if (dirs.empty())
                ImGui::TextDisabled("所选日期没有观测目录。");
            else if (s_scanning)
                ImGui::TextDisabled("扫描中…");
            else
                ImGui::TextDisabled("该日期目录下没有匹配的观测文件。");
        } else {
            const float avail = ImGui::GetContentRegionAvail().x;
            const float gap = ImGui::GetStyle().ItemSpacing.x;
            float x = 0.0f;
            for (const auto &[m, dirUrl, localPath]: matches) {
                const std::string local = !localPath.empty()
                                              ? localPath
                                              : DownloadStore::findLocalFile(ProductType::OBS, m.year, m.doy,
                                                                             m.filename);
                const std::string label = DownloadStore::localNameFor(m.filename);
                const bool remoteKnown = !dirUrl.empty();
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
                    const std::string tip = label + "\n下载中…（完成后自动变绿）";
                    downloadChip(label.c_str(), frac, w, tip.c_str());
                } else {
                    const bool dl = !local.empty();
                    if (dl) ImGui::PushStyleColor(ImGuiCol_Text, UI::kColPass);
                    else pushRemoteChipStyle();
                    const bool pressed = ImGui::Button(label.c_str());
                    if (dl) ImGui::PopStyleColor();
                    else popRemoteChipStyle();
                    if (ImGui::IsItemHovered()) {
                        std::string tip = label;
                        if (!dl) tip += "\n本地：无（点击下载）";
                        else if (remoteKnown) tip += "\n本地：" + local + "\n（点击可重新下载 · 右键管理）";
                        else tip += "\n本地：" + local + "\n远端列表未列出（仅本地）\n（右键可打开 / 删除）";
                        ImGui::SetTooltip("%s", tip.c_str());
                    }
                    if (dl) fileContextMenu(local);

                    if (pressed && remoteKnown)
                        enqueueDownload(Pending{
                            m.filename, dirUrl, ProductType::OBS, m.year, m.doy,
                            m.spanDays, local
                        });
                }
                ImGui::PopID();
                x += ImGui::GetItemRectSize().x + gap;
            }
        }
        ImGui::EndChild();

        {
            std::lock_guard lk(s_mtx);
            if (s_scanning)
                ImGui::Text("扫描中 %d/%d  %s", s_done.load(), s_total.load(), s_status.c_str());
            else if (s_dlActiveCount.load() > 0)
                ImGui::Text("下载中  已完成 %d/%d  %s", s_dlDone.load(), s_dlTotal.load(),
                            s_status.c_str());
            else if (s_dlStateFilter != 0)
                ImGui::Text("共 %zu / %d 个观测文件（已按本地状态筛选）   %s", matches.size(),
                            nAllMatches, s_status.c_str());
            else
                ImGui::Text("共 %zu 个观测文件   %s", matches.size(), s_status.c_str());
        }
    }

    // 选本地文件 → 校验 → 入库
    static void openImportDialog(const VersionedProduct &p) {
        const std::wstring desc = Str::ACPToWide(std::string(p.label) + " (" + p.exts + ")");
        const std::wstring pat = Str::ACPToWide(p.exts);
        const GuiFileFilter filters[2] = {{desc.c_str(), pat.c_str()}, {L"所有文件 (*.*)", L"*.*"}};

        std::wstring wpath;
        if (!ShowOpenFileDialog(wpath, filters, 2)) return;

        const std::string path = Str::wideToACP(wpath);
        const auto [ok, file, version, detail] = Catalog::importLocal(p, path);
        s_importOk = ok;
        s_importMsg = ok
                          ? Str::baseName(path) + " → " + version + "/" + file + "（" + detail + "）"
                          : "导入失败：" + detail + "　（" + Str::baseName(path) + "）";
        if (ok)
            AutoSync::noteImported(p.id, version, file, "已导入本地文件");
    }

    void renderAuxTab() {
        const auto &cfg = AppConfig::PathMemory::instance();
        bool autoOn = AutoSync::enabled();
        if (ImGui::Checkbox("启动时自动检查并更新", &autoOn)) {
            AutoSync::setEnabled(autoOn);
            cfg.set("dl_autosync", autoOn ? "1" : "0");
        }
        ImGui::SameLine();
        if (ImGui::Button("全部检查更新"))
            AutoSync::checkNow();

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
                const auto [phase, version, file, detail, progress, gotBytes, totalBytes] = AutoSync::status(p.id);

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
                if (phase == AutoSync::Status::Checking)
                    ImGui::TextDisabled("检查中…");
                else if (!version.empty())
                    ImGui::TextUnformatted(file.c_str());
                else
                    ImGui::TextDisabled("—");

                ImGui::TableSetColumnIndex(3);
                switch (phase) {
                    case AutoSync::Status::Downloading: {
                        char tip[192];
                        if (totalBytes > 0.0)
                            std::snprintf(tip, sizeof tip, "%s\n%.1f / %.1f MB（中断可续传）",
                                          file.c_str(), gotBytes / 1048576.0,
                                          totalBytes / 1048576.0);
                        else
                            std::snprintf(tip, sizeof tip, "%s\n已收 %.1f MB", file.c_str(),
                                          gotBytes / 1048576.0);
                        downloadChip(file.c_str(), progress, -1.0f, tip);
                        break;
                    }
                    case AutoSync::Status::Ready:
                        ImGui::TextColored(UI::kColPass, "%s",
                                           detail.empty() ? "就绪" : detail.c_str());
                        break;
                    case AutoSync::Status::Error:
                        ImGui::TextColored(UI::kColFail, "%s", detail.c_str());
                        break;
                    default:
                        ImGui::TextDisabled("—");
                        break;
                }

                ImGui::TableSetColumnIndex(4);
                const bool busy = phase ==AutoSync::Status::Checking ||
                                  phase == AutoSync::Status::Downloading;
                ImGui::BeginDisabled(busy);
                if (ImGui::SmallButton(("检查##chk_" + std::string(p.id)).c_str()))
                    AutoSync::checkNow(p.id);
                ImGui::EndDisabled();

                ImGui::SameLine();
                if (ImGui::SmallButton(("选取本地文件##imp_" + std::string(p.id)).c_str()))
                    openImportDialog(p);
            }
            ImGui::EndTable();
        }

        ImGui::Spacing();
        if (!s_importMsg.empty())
            ImGui::TextColored(s_importOk
                                   ? UI::kColPass
                                   : UI::kColFail,
                               "%s", s_importMsg.c_str());
        ImGui::TextDisabled("本地目录：%s", Store::root().c_str());
    }

    //「获取」内联面板
    bool renderAcquirePanel(AcquirePanel &p, const char *idScope) {
        if (!p.open) return false;
        if (p.kind == AcquirePanel::Kind::Products && !p.target) {
            p.open = false;
            return false;
        }
        if (p.kind == AcquirePanel::Kind::Single &&
            (!p.singleOut || p.singleType == ProductType::Unknown)) {
            p.open = false;
            return false;
        }
        const bool isObs = p.kind == AcquirePanel::Kind::Obs;
        const bool isSingle = p.kind == AcquirePanel::Kind::Single;

        ImGui::PushID(idScope);
        ImGui::PushID(&p);
        ImGui::Indent(12.0f);

        ImGui::SameLine();
        if (isObs) {
            const std::string cur = AppConfig::PathMemory::instance().get(p.obsKey);
            ImGui::TextDisabled("（当前：%s）", cur.empty() ? "未设" : Str::baseName(cur).c_str());
        } else if (isSingle) {
            const std::string cur = p.singleOut ? *p.singleOut : std::string();
            ImGui::TextDisabled("（当前：%s）", cur.empty() ? "未设" : Str::baseName(cur).c_str());
        } else {
            ImGui::TextDisabled("（已选 %d）", static_cast<int>(p.target->size()));
        }

        if (p.year <= 0 || p.doy <= 0) {
            const auto [fst, snd] = ProductMeta::defaultObsDate();
            p.year = fst;
            p.doy = snd;
        }
        {
            YearMonthDay ymd = Time::DOYToYMD(p.year, p.doy).value_or(YearMonthDay{p.year, 1, 1});
            bool changed = false;
            ImGui::TextUnformatted("日期");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(90.0f);
            if (ImGui::InputInt("##acqYear", &ymd.year, 0, 0)) changed = true;
            ymd.year = std::clamp(ymd.year, 1980, 2099);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(56.0f);
            if (ImGui::InputInt("##acqMonth", &ymd.month, 0, 0)) changed = true;
            ymd.month = std::clamp(ymd.month, 1, 12);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(56.0f);
            if (ImGui::InputInt("##acqDay", &ymd.day, 0, 0)) changed = true;
            ymd.day = std::clamp(ymd.day, 1, Time::daysInMonth(ymd.year, ymd.month));
            ImGui::SameLine();
            if (ImGui::SmallButton("昨日##acqYest")) {
                const auto [fst, snd] = ProductMeta::defaultObsDate();
                if (const auto y = Time::DOYToYMD(fst, snd)) ymd = *y;
                changed = true;
            }
            const int doy = Time::YMDToDOY(ymd);
            ImGui::SameLine();
            ImGui::TextDisabled("年积日 %03d   第 %d 周", doy, ProductMeta::gpsWeek(ymd.year, doy));
            if (changed) {
                p.year = ymd.year;
                p.doy = doy;
            }
        }

        std::set<ProductType> types;
        if (isObs)
            types.insert(ProductType::OBS);
        else if (isSingle)
            types.insert(p.singleType);
        else
            for (int i = 0; i < kDateTypeCount; ++i) types.insert(static_cast<ProductType>(i));
        const std::vector<std::string> dirs = ProductMeta::scanDirs(types, p.year, p.doy);
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
                if (s_scanThread.joinable()) s_scanThread.join();
                s_scanThread = std::thread(runScan, std::move(missing));
            }
        }

        std::vector<Match> all;
        {
            std::lock_guard lk(s_mtx);
            std::set<std::string> seen;
            for (const auto &d: dirs) {
                const auto it = s_dirs.find(d);
                if (it == s_dirs.end() || !it->second.loaded) continue;
                for (const auto &m: it->second.items) {
                    if (!types.count(m.type)) continue;
                    if (!m.covers(p.year, p.doy)) continue;
                    if (!seen.insert(m.filename).second) continue;
                    all.push_back({m, d});
                }
            }
        }
        std::sort(all.begin(), all.end(),
                  [](const Match &a, const Match &b) { return a.m.filename < b.m.filename; });
        for (const auto t: types) appendLocalOnly(all, t, p.year, p.doy);

        const auto &dcs = ProductMeta::knownDatacenters();
        if (!isObs) {
            if (!isSingle) {
                if (p.typeOn.size() != static_cast<std::size_t>(kDateTypeCount))
                    p.typeOn.assign(kDateTypeCount, 1);

                ImGui::TextUnformatted("产品类型");
                float x = 0.0f;
                const float avail = ImGui::GetContentRegionAvail().x;
                const float gap = ImGui::GetStyle().ItemSpacing.x;
                for (int i = 0; i < kDateTypeCount; ++i) {
                    if (const float w = chipWidth(TYPE_LABELS[i]); x > 0.0f && x + w > avail) x = 0.0f;
                    if (x > 0.0f) ImGui::SameLine();
                    if (tagChip(TYPE_LABELS[i], p.typeOn[i] != 0)) p.typeOn[i] = p.typeOn[i] ? 0 : 1;
                    x += ImGui::GetItemRectSize().x + gap;
                }
            } else {
                ImGui::TextDisabled("%s", typeLabel(p.singleType));
                ImGui::SameLine();
            }

            if (isSingle && p.singleType == ProductType::NAV) {
                ImGui::TextDisabled("（广播星历不区分数据中心）");
            } else {
                if (p.dcOn.size() != dcs.size()) p.dcOn.assign(dcs.size(), 0);
                ImGui::TextUnformatted("数据中心");
                ImGui::SameLine();
                ImGui::TextDisabled("不选＝不限");
                float x = 0.0f;
                const float avail = ImGui::GetContentRegionAvail().x;
                const float gap = ImGui::GetStyle().ItemSpacing.x;
                for (std::size_t i = 0; i < dcs.size(); ++i) {
                    if (const float w = chipWidth(dcs[i].c_str()); x > 0.0f && x + w > avail) x = 0.0f;
                    if (x > 0.0f) ImGui::SameLine();
                    if (tagChip(dcs[i].c_str(), p.dcOn[i] != 0)) p.dcOn[i] = p.dcOn[i] ? 0 : 1;
                    x += ImGui::GetItemRectSize().x + gap;
                }
            }
        } else {
            // 站点候选取自当天扫到的文件（与独立面板同源），扫描没回来时是空的
            std::set<std::string> sites;
            for (const auto &mt: all) sites.insert(obsStationOf(mt.m.filename));

            ImGui::TextUnformatted("站点");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(140);
            ImGui::InputTextWithHint("##acqSiteFind", "筛选站点…", p.siteFind, sizeof p.siteFind);
            ImGui::SameLine();
            if (ImGui::SmallButton("清空选择##acqSiteClear") && !p.sites.empty()) p.sites.clear();
            ImGui::SameLine();
            ImGui::TextDisabled("已选 %zu / 共 %zu 站 %s", p.sites.size(), sites.size(),
                                p.sites.empty() ? "（未选＝全部）" : "");

            ImGui::BeginChild("##acqSites", ImVec2(0, 62.0f), ImGuiChildFlags_Borders);
            if (sites.empty()) {
                ImGui::TextDisabled(s_scanning ? "扫描中…" : "该日期没有可选的观测站点");
            } else {
                const std::string find = Str::toUpper(p.siteFind);
                const float avail = ImGui::GetContentRegionAvail().x;
                const float gap = ImGui::GetStyle().ItemSpacing.x;
                float x = 0.0f;
                int shown = 0;
                for (const auto &st: sites) {
                    if (!find.empty() && st.find(find) == std::string::npos) continue;
                    const bool on = p.sites.count(st) > 0;
                    if (const float w = chipWidth(st.c_str()); x > 0.0f && x + w > avail) x = 0.0f;
                    if (x > 0.0f) ImGui::SameLine();
                    if (tagChip(st.c_str(), on)) {
                        if (on) p.sites.erase(st);
                        else p.sites.insert(st);
                    }
                    x += ImGui::GetItemRectSize().x + gap;
                    ++shown;
                }
                if (shown == 0) ImGui::TextDisabled("没有站点匹配「%s」", p.siteFind);
            }
            ImGui::EndChild();
        }

        {
            ImGui::TextUnformatted("本地状态");
            ImGui::SameLine();
            stateFilterChips("##acqState", &p.stateFilter);
        }

        // 应用筛选：类型 → 数据中心 → 站点 → 本地状态
        std::vector<Match> matches;
        matches.reserve(all.size());
        std::set<std::string> dcSel;
        if (!isObs)
            for (std::size_t i = 0; i < p.dcOn.size() && i < dcs.size(); ++i)
                if (p.dcOn[i]) dcSel.insert(dcs[i]);
        for (const auto &mt: all) {
            const auto &m = mt.m;
            if (isObs) {
                if (!p.sites.empty() && !p.sites.count(obsStationOf(m.filename))) continue;
            } else if (isSingle) {
                if (m.type != p.singleType) continue;
                if (m.type != ProductType::NAV && !dcSel.empty() && !dcSel.count(m.datacenter))
                    continue;
            } else {
                if (const auto t = static_cast<std::size_t>(m.type); t >= static_cast<std::size_t>(kDateTypeCount) || !p.typeOn[t]) continue;
                if (m.type != ProductType::NAV && !dcSel.empty() && !dcSel.count(m.datacenter))
                    continue;
            }
            const std::string local = !mt.localPath.empty()
                                          ? mt.localPath
                                          : DownloadStore::findLocalFile(m.type, m.year, m.doy, m.filename);
            if (!passStateFilter(p.stateFilter, !local.empty())) continue;
            matches.push_back(mt);
        }
        float listH = p.height;
        const float availH = ImGui::GetContentRegionAvail().y - 46.0f;
        const float maxH = availH > 120.0f ? availH : 640.0f;
        if (listH > maxH) listH = maxH;
        if (listH < 96.0f) listH = 96.0f;
        ImGui::BeginChild("##acqList", ImVec2(0, listH), ImGuiChildFlags_Borders);
        if (all.empty()) {
            if (s_scanning) ImGui::TextDisabled("扫描中…");
            else
                ImGui::TextDisabled("该日期没有可获取的产品文件。");
        } else if (matches.empty()) {
            ImGui::TextDisabled("当前筛选没有命中。");
        } else {
            std::vector<std::pair<const char *, std::vector<const Match *> > > groups;
            if (isObs || isSingle) {
                std::vector<const Match *> g;
                g.reserve(matches.size());
                for (const auto &mt: matches) g.push_back(&mt);
                groups.emplace_back(nullptr, std::move(g));
            } else {
                groups.resize(kDateTypeCount);
                for (int t = 0; t < kDateTypeCount; ++t) groups[t].first = TYPE_LABELS[t];
                for (const auto &mt: matches) {
                    if (const int t = static_cast<int>(mt.m.type); t >= 0 && t < kDateTypeCount) groups[t].second.push_back(&mt);
                }
            }
            for (auto &[name, vec]: groups) {
                if (vec.empty()) continue;
                if (name) ImGui::TextDisabled("%s", name);
                float x = 0.0f;
                const float avail = ImGui::GetContentRegionAvail().x;
                const float gap = ImGui::GetStyle().ItemSpacing.x;
                for (const Match *pmt: vec) {
                    const ProductMeta &m = pmt->m;
                    const std::string &dirUrl = pmt->dirUrl;
                    const std::string local = !pmt->localPath.empty()
                                                  ? pmt->localPath
                                                  : DownloadStore::findLocalFile(m.type, m.year, m.doy,
                                                                                 m.filename);
                    float frac = -1.0f;
                    bool downloading = false;
                    {
                        std::lock_guard lk(s_mtx);
                        if (auto it = s_dlActive.find(dirUrl + m.filename); it != s_dlActive.end()) {
                            downloading = true;
                            frac = it->second.frac;
                        }
                    }
                    bool selected = false;
                    if (!local.empty()) {
                        if (isObs)
                            selected = AppConfig::PathMemory::instance().get(p.obsKey) == local;
                        else if (isSingle)
                            selected = p.singleOut && *p.singleOut == local;
                        else
                            selected = std::find(p.target->begin(), p.target->end(), local) !=
                                       p.target->end();
                    }
                    const std::string shown = DownloadStore::localNameFor(m.filename);
                    std::string label = shown;
                    if (selected) label.insert(0, "√ ");
                    const float w = downloading
                                        ? chipWidth((shown + "  100%").c_str())
                                        : chipWidth(label.c_str());
                    if (x > 0.0f && x + w > avail) x = 0.0f;
                    if (x > 0.0f) ImGui::SameLine();
                    ImGui::PushID(label.c_str());
                    if (downloading) {
                        const std::string tip = shown + "\n下载中…（完成后自动变绿）";
                        downloadChip(shown.c_str(), frac, w, tip.c_str());
                    } else if (!local.empty()) {
                        if (selected) ImGui::PushStyleColor(ImGuiCol_Text, UI::kColPass);
                        const bool pressed = ImGui::Button(label.c_str());
                        if (selected) ImGui::PopStyleColor();
                        if (!selected && ImGui::IsItemHovered())
                            ImGui::SetTooltip("%s\n%s", shown.c_str(), local.c_str());
                        fileContextMenu(local);
                        if (pressed) {
                            if (isObs) {
                                auto &pm = AppConfig::PathMemory::instance();
                                const std::string pick = selected ? std::string() : local;
                                pm.set(p.obsKey, pick);
                                pm.flush();
                                if (p.obsOut) *p.obsOut = pick;
                            } else if (isSingle) {
                                const std::string pick = selected ? std::string() : local;
                                *p.singleOut = pick;
                                if (p.onPick) p.onPick(pick);
                            } else {
                                if (selected)
                                    p.target->erase(
                                        std::remove(p.target->begin(), p.target->end(), local),
                                        p.target->end());
                                else
                                    p.target->push_back(local);
                                if (p.owner) {
                                    p.owner->syncFromFiles();
                                    p.owner->save();
                                }
                            }
                        }
                    } else {
                        pushRemoteChipStyle();
                        const bool pressed = ImGui::Button(shown.c_str());
                        popRemoteChipStyle();
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("%s\n未下载，点击开始下载", shown.c_str());
                        if (pressed && !dirUrl.empty())
                            enqueueDownload(Pending{
                                m.filename, dirUrl, m.type, m.year, m.doy,
                                m.spanDays, ""
                            });
                    }
                    ImGui::PopID();
                    x += ImGui::GetItemRectSize().x + gap;
                }
                ImGui::Spacing();
            }
        }
        ImGui::EndChild();

        {
            ImGui::InvisibleButton("##acqGrip", ImVec2(ImGui::GetContentRegionAvail().x, 7.0f));
            if (ImGui::IsItemActive()) {
                p.height += ImGui::GetIO().MouseDelta.y;
                if (p.height < 96.0f) p.height = 96.0f;
                if (p.height > maxH) p.height = maxH;
            }
            if (ImGui::IsItemHovered() || ImGui::IsItemActive())
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
            // 中间一根短横线当把手——不画的话没人知道这里能拖
            const ImVec2 a = ImGui::GetItemRectMin();
            const ImVec2 b = ImGui::GetItemRectMax();
            const float cy = (a.y + b.y) * 0.5f;
            const float cx = (a.x + b.x) * 0.5f;
            ImGui::GetWindowDrawList()->AddLine(ImVec2(cx - 26.0f, cy), ImVec2(cx + 26.0f, cy),
                                                ImGui::GetColorU32(ImGuiCol_Separator), 2.0f);
        }

        if (s_scanning) ImGui::TextDisabled("扫描中…");
        else if (s_dlActiveCount.load() > 0)
            ImGui::TextDisabled("下载中  已完成 %d/%d", s_dlDone.load(), s_dlTotal.load());
        else
            ImGui::TextDisabled("%zu / %zu 个文件", matches.size(), all.size());
        ImGui::SameLine();
        if (ImGui::SmallButton("收起##acqClose")) p.open = false;

        renderDeleteConfirm();

        ImGui::Unindent(12.0f);
        ImGui::PopID();
        ImGui::PopID();
        return p.open;
    }

    void Render(bool *p_open) {
        if (!s_open) return;

        ImGui::SetNextWindowSize(ImVec2(900, 560), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("产品下载", p_open)) {
            ImGui::End();
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
            ImGui::EndTabBar();
        }

        renderDeleteConfirm();

        ImGui::End();
    }

    void Shutdown() {
        if (s_scanThread.joinable()) s_scanThread.join();
        s_dlStop = true;
        s_dlCv.notify_all();
        for (auto &w: s_dlWorkers)
            if (w.joinable()) w.join();
    }
}
