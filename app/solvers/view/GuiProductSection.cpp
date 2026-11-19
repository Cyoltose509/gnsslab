#include "ui/GuiColors.h"
#include "FileUtils.h"
#include <windows.h>

#include "GuiHelpers.h"
#include "GuiProductSection.h"

#include "core/PathMemory.h"
#include "io/ProductPaths.h"
#include "io/ProductReader.h"
#include "download/local/Store.h"
#include "io/ObsSource.h"
#include "io/Observer.h"
#include "product/ATXReader.h"
#include "product/sp3/Reader.h"
#include "ui/FileDialog.h"

#include "download/local/FileMeta.h"
#include "io/ObsDate.h"
#include "core/TimeStruct.h"
#include "download/ui/Acquire.h"

#include "StringUtils.h"
#include <algorithm>
#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>
#include "i18n/I18n.h"
#include "ui/ImGuiLabel.h"

namespace {
    namespace fs = std::filesystem;

    // 选中一条路径后的公共动作：写入 PathMemory、触发回调、自动配 NAV
    void adoptPath(const char *iniKey, const std::string &p,
                   const std::function<void(const std::string &)> &onSelect,
                   std::string *autoNavBuf) {
        AppConfig::PathMemory::instance().set(iniKey, p);
        if (onSelect) onSelect(p);
        if (autoNavBuf) {
            if (const std::string d = ObsSource::autoDetectNav(p); !d.empty()) *autoNavBuf = d;
        }
    }

    /// 「获取」按钮的 tooltip
    std::string obsDateTip(const ObsDate &od) {
        std::string tip = I18n::tr("prod.tip_by_date", DOYToDateText(od.year, od.doy));
        if (od.from == ObsDateFrom::Name) return tip;
        tip += I18n::trc("prod.tip_date_from_file");
        return tip;
    }

    /// 同一批精密产品是否混了不同中心/产品线
    std::string productConsistencyNote(const AppConfig::ProductPaths &p) {
        std::set<std::string> lines;
        for (const std::string &path: p.files) {
            const auto m = FileMeta::parse(File::baseName(path));
            if (!m) continue;
            switch (m->type) {
                case ProductType::ORB: case ProductType::CLK:
                case ProductType::OSB: case ProductType::ATT:
                    break;
                default: continue;
            }
            if (const std::string line = FileMeta::productLineOf(path); !line.empty())
                lines.insert(line);
        }
        if (lines.size() < 2) return {};
        std::string joined;
        for (const std::string &l: lines) {
            if (!joined.empty()) joined += ", ";
            joined += l;
        }
        return I18n::tr("prod.version_mismatch", joined);
    }

    std::string s_dateKey;
    std::optional<ObsDate> s_dateCache;

    std::optional<ObsDate> cachedObsDate(const std::string &path) {
        std::error_code ec;
        const auto size = fs::file_size(path, ec);
        std::string key = path + (ec ? "|missing" : "|" + std::to_string(size));
        if (!ec) {
            std::error_code stampEc;
            const auto stamp = fs::last_write_time(path, stampEc);
            key += stampEc ? "|unknown" : "|" + std::to_string(stamp.time_since_epoch().count());
        }
        if (key != s_dateKey) {
            s_dateKey = key;
            s_dateCache = parseObsDate(path);
        }
        return s_dateCache;
    }

    std::vector<std::string> s_verdictFiles;
    std::map<std::string, std::string> s_verdict;

    /// 时间序列产品（轨道/钟差/广播星历/码偏差/姿态/地球自转）按历元合并多份使用，全部「选用」；
    /// 单版本产品（ATX/对流层/电离层）才优选一份、其余「备用」。
    bool isMergedType(const ProductType t) {
        switch (t) {
            case ProductType::ORB: case ProductType::CLK: case ProductType::NAV:
            case ProductType::OSB: case ProductType::ATT: case ProductType::ERP:
            case ProductType::VMF:
                return true;
            default:
                return false;
        }
    }

    /// 「选用/备用」只随文件列表变化：每帧对每个文件重建同类列表再 rank 是 O(n²)。
    const std::string &verdictOf(const std::vector<std::string> &files, const std::string &path) {
        if (files != s_verdictFiles) {
            s_verdictFiles = files;
            s_verdict.clear();
            std::map<ProductType, std::vector<std::string>> byType;
            for (const auto &f: files) {
                if (const auto m = FileMeta::parse(File::baseName(f)); m)
                    byType[m->type].push_back(f);
            }
            for (auto &[t, group]: byType) {
                if (group.size() < 2 || isMergedType(t)) {
                    for (const auto &f: group) s_verdict[f] = I18n::trc("prod.verdict_pick");
                    continue;
                }
                for (const auto &f: group) s_verdict[f] = I18n::trc("prod.verdict_spare");
                const auto ranked = Download::Store::rank(group, t);
                const std::string &win = ranked.empty() ? group.front() : ranked.front().path;
                s_verdict[win] = I18n::trc("prod.verdict_pick");
            }
        }
        static const std::string kUnknown;
        const auto it = s_verdict.find(path);
        return it == s_verdict.end() ? kUnknown : it->second;
    }
}

void Solvers::Helpers::fileRow(const char *label, std::string *buf, const char *iniKey,
                               const std::vector<GuiFileFilter> &filters,
                               const std::function<void(const std::string &)> &onSelect,
                               const char *obsIniKey, std::string *autoNavBuf,
                               const ProductType obsType) {
    static std::map<std::string, std::string> synced;
    // 右侧按钮宽度按字号实测：留死宽度会让按钮浮在右边，输入框和按钮之间空一块。
    const ImGuiStyle &style = ImGui::GetStyle();
    const auto buttonW = [&style](const char *text) {
        return ImGui::CalcTextSize(text).x + style.FramePadding.x * 2.0f;
    };
    const bool isObsRow = obsIniKey && std::string(iniKey) == obsIniKey;
    const ProductType ptype = typeFromIniKey(iniKey);
    // 「获取」可能出现两次（观测页跳转 + 单品下载），留白按实际按钮数算。
    const int nGet = (isObsRow ? 1 : 0) + (ptype != ProductType::Unknown ? 1 : 0);
    float reserve = buttonW(I18n::trc("common.browse")) + style.ItemSpacing.x;
    reserve += static_cast<float>(nGet) * (buttonW(I18n::trc("common.fetch")) + style.ItemSpacing.x);

    ImGui::Text("%s:", label);
    ImGui::PushItemWidth(-reserve);
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
    if (ImGui::Button(UI::idLabel("common.browse", "##", label))) {
        if (std::wstring wpath; FileDialog::open(wpath, filters)) {
            const std::string p = Str::wideToACP(wpath);
            *buf = p;
            adoptPath(iniKey, p, onSelect, autoNavBuf);
        }
    }

    if (isObsRow) {
        static std::map<const void *, Download::Acquire::Panel> panels;
        auto &panel = panels[buf];
        panel.kind = Download::Acquire::Panel::Kind::Obs;
        panel.obsKey = iniKey;
        panel.obsOut = buf;
        panel.obsType = obsType;

        ImGui::SameLine();
        if (ImGui::Button(UI::idLabel("common.fetch", "##", label))) {
            const ObsDate od = cachedObsDate(*buf).value_or(defaultObsDate());
            panel.year = od.year;
            panel.doy = od.doy;
            panel.open = true;
        }
        if (ImGui::IsItemHovered()) {
            const ObsDate od = cachedObsDate(*buf).value_or(defaultObsDate());
            ImGui::SetTooltip("%s", obsDateTip(od).c_str());
        }
        if (panel.open) Download::Acquire::render(panel, iniKey);
    }

    if (ptype != ProductType::Unknown) {
        static std::map<const void *, Download::Acquire::Panel> singlePanels;
        auto &panel = singlePanels[buf];
        panel.kind = Download::Acquire::Panel::Kind::Single;
        panel.singleType = ptype;
        panel.singleOut = buf;
        // onPick 每帧重绑：它捕获的是当前任务自己的回调与缓冲，缓存的旧 lambda 会写进已销毁的任务
        panel.onPick = [iniKey, onSelect, autoNavBuf](const std::string &path) {
            if (path.empty()) return;
            adoptPath(iniKey, path, onSelect, autoNavBuf);
        };

        const std::string obsPath = obsIniKey && *obsIniKey
                                        ? AppConfig::PathMemory::instance().get(obsIniKey)
                                        : "";

        ImGui::SameLine();
        if (ImGui::Button(UI::idLabel("common.fetch", "##", label))) {
            const ObsDate od = cachedObsDate(obsPath).value_or(defaultObsDate());
            panel.year = od.year;
            panel.doy = od.doy;
            panel.open = true;
        }
        if (panel.open) Download::Acquire::render(panel, iniKey);
    }
}

void Solvers::Helpers::renderProductSection(AppConfig::ProductPaths &p, const char *obsIniKey,
                                            const bool usesAtmosphere, const bool crossDayHint) {
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

    if (!ImGui::CollapsingHeader(I18n::trc("prod.files"), ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        ImGui::TextDisabled(I18n::trc("prod.file_count"), static_cast<int>(p.files.size()));
        return;
    }

    if (crossDayHint) {
        ImGui::TextColored(UI::kColWarn, I18n::trc("prod.cross_day_hint"));
    }

    if (ImGui::SmallButton(UI::idLabel("common.browse", "##prod"))) {
        if (std::vector<std::wstring> ws; FileDialog::openMany(ws, FileDialog::fProd())) {
            int added = 0;
            std::string bad;
            for (auto &w: ws) {
                const std::string path = Str::wideToACP(w);
                if (std::find(p.files.begin(), p.files.end(), path) != p.files.end()) continue;
                p.files.push_back(path);
                if (!FileMeta::parse(File::baseName(path))) bad += File::baseName(path) + " ";
                ++added;
            }
            resync();
            ok = bad.empty();
            msg = I18n::tr("prod.added", added);
            if (!bad.empty()) msg += I18n::trc("prod.unknown_types") + bad;
        }
    }
    ImGui::SameLine();
    static std::map<std::string, Download::Acquire::Panel> acqPanels;
    auto &acq = acqPanels[obsIniKey ? obsIniKey : ""];
    acq.kind = Download::Acquire::Panel::Kind::Products;
    acq.target = &p.files;
    acq.owner = &p;
    acq.usesAtmosphere = usesAtmosphere;
    if (ImGui::SmallButton(UI::idLabel("common.fetch", "##prod"))) {
        const std::string obsPath = obsIniKey && *obsIniKey
                                        ? AppConfig::PathMemory::instance().get(obsIniKey)
                                        : "";
        const ObsDate od = cachedObsDate(obsPath).value_or(defaultObsDate());
        acq.year = od.year;
        acq.doy = od.doy;
        acq.open = true;
        ok = true;
        msg = I18n::trc("prod.list_hint");
    }
    ImGui::SameLine();
    if (ImGui::SmallButton(UI::idLabel("common.clear", "##prod"))) {
        p.files.clear();
        resync();
        msg.clear();
    }

    // 内联展开的产品清单（按钮下方），页面整体往下推
    if (acq.open) Download::Acquire::render(acq, obsIniKey ? obsIniKey : "prod");

    ImGui::Spacing();
    if (!p.files.empty())  {
        ImGui::Text(I18n::trc("prod.selected_count"), static_cast<int>(p.files.size()));
        for (std::size_t i = 0; i < p.files.size(); ++i) {
            const std::string &path = p.files[i];
            const auto m = FileMeta::parse(File::baseName(path));

            ImGui::PushID(static_cast<int>(i));
            ImGui::Bullet();
            ImGui::Text("%s", File::baseName(path).c_str());
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", path.c_str());

            ImGui::SameLine();
            if (!m) {
                ImGui::TextColored(UI::kColFail, I18n::trc("prod.unknown_type_tag"));
            } else {
                const std::string use = verdictOf(p.files, path);
                const bool win = use == I18n::trc("prod.verdict_pick");
                ImGui::TextColored(win ? UI::kColPass
                                       : UI::kColMute,
                                   "[%s/%s]", typeLabel(m->type), use.c_str());
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

    if (const std::string note = productConsistencyNote(p); !note.empty()) {
        ImGui::TextColored(UI::kColWarn, "%s", note.c_str());
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", I18n::trc("prod.mixed_ac_hint"));
    }

    if (!msg.empty())
        ImGui::TextColored(ok ? UI::kColPass : UI::kColFail,
                           "%s", msg.c_str());
}
