#include "download/ui/Panel.h"
#include "FileUtils.h"

#include "download/remote/Fetcher.h"
#include "download/remote/SourceTable.h"
#include "download/remote/Earthdata.h"
#include "download/remote/http/Core.h"
#include "download/ui/Widgets.h"
#include "download/remote/Catalog.h"
#include "download/local/ProductMeta.h"
#include "download/local/Store.h"

#include "io/ProductLoad.h"

#include "core/PathMemory.h"
#include "ui/DatePicker.h"
#include "ui/Gui.h"
#include "ui/GuiColors.h"

#include "StringUtils.h"
#include "TimeStruct.h"
#include "Log.h"
#include "util/Clock.h"

#include "imgui.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <future>
#include <iterator>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace Download::Panel {
    namespace {
        State s_state;

        /// 列表点击的统一动作：远端有地址就排队下载（已下载的再点一次就是重新下载）。
        void enqueueDownload(const Fetcher::Match &m, const std::string &) {
            if (const std::string url = m.remoteUrl(); !url.empty()) Fetcher::enqueue(Fetcher::pendingOf(m));
        }

        constexpr char kEarthdataProbeUrl[] = "https://cddis.nasa.gov/archive/gnss/products/";

        struct EarthdataProbe {
            bool ok = false;
            std::string msg;
        };

        struct EarthdataUi {
            char user[128] = {};
            char pass[128] = {};
            std::string status;
            bool statusOk = false;
            bool busy = false;
            bool loaded = false;
            std::atomic<bool> cancel{false};
            std::future<EarthdataProbe> pending;
        };

        EarthdataUi s_earthdata;

        void copyInto(char (&dst)[128], const std::string &s) {
            const std::size_t n = s.size() < sizeof(dst) - 1 ? s.size() : sizeof(dst) - 1;
            std::memcpy(dst, s.data(), n);
            dst[n] = '\0';
        }
    }

    State &state() {
        return s_state;
    }

    void PageDate::today() {
        UI::DatePicker::today(year, month, day);
    }

    int PageDate::doy() const {
        return YearMonthDay{year, month, day}.toDOY();
    }

    void State::renderDateTab() {
        if (!prod.isSet()) prod.today();
        Widgets::typeChips(typeOn);
        Widgets::searchBox("dateFind", prod.find, sizeof prod.find);

        ImGui::TextUnformatted("本地状态");
        ImGui::SameLine();
        Widgets::stateFilterChips("##dateState", &prod.stateFilter);

        Widgets::dateRow(prod.year, prod.month, prod.day, "date", "今天", UI::DatePicker::today, true);

        ImGui::Separator();

        SourceTable::Query q;
        q.year = prod.year;
        q.doy = prod.doy();
        for (int i = 0; i < kDateTypeCount && i < static_cast<int>(typeOn.size()); ++i)
            if (typeOn[i]) q.types.insert(static_cast<ProductType>(i));

        if (q.types.empty()) {
            ImGui::TextDisabled("先勾选上面的产品类型。");
            return;
        }

        const auto snap = Fetcher::collect(SourceTable::daily(q), q);

        Widgets::ListOptions opt;
        opt.grouping = Widgets::Grouping::ByType;
        opt.stateFilter = prod.stateFilter;
        opt.find = prod.find;
        opt.onClick = enqueueDownload;
        Widgets::remoteList(snap, opt, "##files", "匹配文件");
    }

    void State::renderObsTab() {
        if (!obs.isSet()) obs.today();
        Widgets::dateRow(obs.year, obs.month, obs.day, "obs", "今天", UI::DatePicker::today, false);
        Widgets::searchBox("obsFind", obs.find, sizeof obs.find);

        ImGui::TextUnformatted("本地状态");
        ImGui::SameLine();
        Widgets::stateFilterChips("##obsState", &obs.stateFilter);

        SourceTable::Query q;
        q.types = {ProductType::OBS};
        q.year = obs.year;
        q.doy = obs.doy();
        q.scanText = "读取观测目录…";

        const auto snap = Fetcher::collect(SourceTable::IGS(q.types, q.year, q.doy), q);

        Widgets::ListOptions opt;
        opt.grouping = Widgets::Grouping::None;
        opt.stateFilter = obs.stateFilter;
        opt.find = obs.find;
        opt.onClick = enqueueDownload;
        Widgets::remoteList(snap, opt, "##obsFiles", "观测文件");
    }

    void State::renderLeoTab() {
        if (!leo.isSet()) leo.today();
        Widgets::dateRow(leo.year, leo.month, leo.day, "leo", "今天", UI::DatePicker::today, false);
        Widgets::searchBox("leoFind", leo.find, sizeof leo.find);

        ImGui::TextUnformatted("本地状态");
        ImGui::SameLine();
        Widgets::stateFilterChips("##leoState", &leo.stateFilter);

        SourceTable::Query q;
        q.types = {ProductType::LEO_OBS};
        q.year = leo.year;
        q.doy = leo.doy();
        q.scanText = "读取低轨观测目录…";

        const auto snap = Fetcher::collect(SourceTable::LEOObs(q.year, q.doy), q);

        Widgets::ListOptions opt;
        opt.grouping = Widgets::Grouping::None;
        opt.stateFilter = leo.stateFilter;
        opt.find = leo.find;
        opt.onClick = enqueueDownload;
        Widgets::remoteList(snap, opt, "##leoFiles", "低轨观测文件");
    }

    void State::renderRefTab() {
        if (!ref.isSet()) ref.today();
        Widgets::dateRow(ref.year, ref.month, ref.day, "ref", "今天", UI::DatePicker::today, false);
        Widgets::searchBox("refFind", ref.find, sizeof ref.find);

        ImGui::TextUnformatted("本地状态");
        ImGui::SameLine();
        Widgets::stateFilterChips("##refState", &ref.stateFilter);

        SourceTable::Query q;
        q.types = {ProductType::SNX, ProductType::LEO_SP3};
        q.year = ref.year;
        q.doy = ref.doy();
        q.scanText = "读取参考真值目录…";

        const auto snap = Fetcher::collect(SourceTable::refTruth(q.year, q.doy), q);

        Widgets::ListOptions opt;
        opt.grouping = Widgets::Grouping::ByType;
        opt.stateFilter = ref.stateFilter;
        opt.find = ref.find;
        opt.onClick = enqueueDownload;
        Widgets::remoteList(snap, opt, "##refFiles", "真值文件");
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
                        ? File::baseName(path) + " → " + version + "/" + file + "（" + detail + "）"
                        : "导入失败：" + detail + "　（" + File::baseName(path) + "）";
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

            for (const auto &p: Catalog::kProducts) {
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
    }

    void State::renderLocalTab() {
        const int done = Fetcher::progress().done;
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
                    Fetcher::setStatus(Store::revealFile(e->path)
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
            const std::pair<Page, const char *> tabs[] = {
                {Page::Date, "日精密产品"},
                {Page::Obs, "观测文件"},
                {Page::Leo, "低轨卫星"},
                {Page::Ref, "参考真值"},
                {Page::Aux, "辅助数据"},
                {Page::Local, "本地文件"},
            };
            for (const auto &[id, label]: tabs) {
                // 点击落在一个尚未选中的页签上时，BeginTabItem 当帧返回 false（选中排到下一帧），
                // 所以「取点击」必须在 if 之前，只有 EndTabItem 才受返回值约束。
                const bool selected = ImGui::BeginTabItem(label);
                const bool clicked = ImGui::IsItemClicked();
                if (selected) ImGui::EndTabItem();
                if (clicked) page = id;
            }
            ImGui::EndTabBar();
        }
        ImGui::Separator();

        switch (page) {
            case Page::Date: renderDateTab(); break;
            case Page::Obs: renderObsTab(); break;
            case Page::Aux: renderAuxTab(); break;
            case Page::Local:
                if (!localShown) localDirty = true;
                renderLocalTab();
                break;
            case Page::Leo: renderLeoTab(); break;
            case Page::Ref: renderRefTab(); break;
        }
        localShown = page == Page::Local;

        if (Widgets::deleteConfirm()) {
            localDirty = true;
            localSel.clear();
        }

        ImGui::End();
        open = shown;
    }

    void State::shutdown() {
        Fetcher::shutdown();
    }

    void renderEarthdata(bool *p_open) {
        auto &g = s_earthdata;
        if (!g.loaded) {
            const auto &pm = AppConfig::PathMemory::instance();
            copyInto(g.user, pm.get(Earthdata::kUserKey));
            copyInto(g.pass, pm.get(Earthdata::kPassKey));
            g.loaded = true;
        }

        if (g.pending.valid() &&
            g.pending.wait_for(Clock::milliseconds(0)) == std::future_status::ready) {
            const auto [ok, msg] = g.pending.get();
            g.status = msg;
            g.statusOk = ok;
            g.busy = false;
        }

        ImGui::SetNextWindowSize(ImVec2(520, 240), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Earthdata 账号", p_open)) {
            ImGui::End();
            return;
        }

        ImGui::TextWrapped("CDDIS 需要 NASA Earthdata Login 账号。账号密码以明文保存在 "
                           "gnsslab.ini，仅对 cddis.nasa.gov、"
                           "earthdata.nasa.gov 的请求生效。");
        ImGui::Spacing();
        ImGui::TextDisabled("没有账号？前往");
        ImGui::SameLine();
        ImGui::TextLinkOpenURL("注册", "https://urs.earthdata.nasa.gov/");
        ImGui::Spacing();

        ImGui::InputText("账号", g.user, sizeof(g.user));
        ImGui::InputText("密码", g.pass, sizeof(g.pass));
        ImGui::Spacing();

        if (ImGui::Button("保存")) {
            const auto &pm = AppConfig::PathMemory::instance();
            pm.set(Earthdata::kUserKey, g.user);
            pm.set(Earthdata::kPassKey, g.pass);
            pm.flush();
            Http::setEarthdata(g.user, g.pass);
            g.status = "已保存到 gnsslab.ini，当前会话已生效";
            g.statusOk = true;
            LOG_INFO << "[Earthdata] 账号配置已保存";
        }

        ImGui::SameLine();
        ImGui::BeginDisabled(g.busy);
        if (ImGui::Button("测试连接")) {
            Http::setEarthdata(g.user, g.pass);
            g.cancel = false;
            g.busy = true;
            g.status.clear();
            g.pending = std::async(std::launch::async, [] {
                try {
                    std::string msg;
                    const bool ok = Http::checkEarthdata(kEarthdataProbeUrl, msg, &s_earthdata.cancel);
                    return EarthdataProbe{ok, msg};
                } catch (const std::exception &e) {
                    return EarthdataProbe{false, std::string("测试连接异常: ") + e.what()};
                } catch (...) {
                    return EarthdataProbe{false, "测试连接发生未知异常"};
                }
            });
        }
        ImGui::EndDisabled();
        if (g.busy) {
            ImGui::SameLine();
            ImGui::TextDisabled("正在连接 CDDIS…");
        }

        if (!g.status.empty()) {
            ImGui::Spacing();
            ImGui::TextColored(g.statusOk ? UI::kColPass : UI::kColFail, "%s", g.status.c_str());
        }

        ImGui::End();
    }

    void shutdownEarthdata() {
        s_earthdata.cancel = true;
        if (s_earthdata.pending.valid()) s_earthdata.pending.wait();
    }
}
