#include "download/ui/Acquire.h"
#include "FileUtils.h"

#include "download/remote/Fetcher.h"
#include "download/local/ProductType.h"
#include "io/ObsDate.h"
#include "download/remote/SourceTable.h"
#include "download/ui/Widgets.h"
#include "download/local/Store.h"
#include "i18n/I18n.h"

#include "core/PathMemory.h"
#include "io/ProductPaths.h"
#include "Observer.h"

#include "StringUtils.h"
#include "core/TimeStruct.h"

#include "ui/DragBar.h"
#include "imgui.h"

#include <algorithm>
#include <filesystem>
#include <set>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace Download::Acquire {
    namespace {
        /// 昨日（本地时区）：IGS 当日观测要到次日才发布，默认翻到昨天。
        void yesterday(int &y, int &m, int &d) {
            const ObsDate def = defaultObsDate();
            if (const auto ymd = YearMonthDay::fromDOY(def.year, def.doy)) {
                y = ymd->year;
                m = ymd->month;
                d = ymd->day;
            }
        }

        /// 目录下的普通文件，按名字排序；传进来不是目录就返回空。
        std::vector<std::string> dirFiles(const std::string &dir) {
            std::vector<std::string> out;
            std::error_code ec;
            if (!std::filesystem::is_directory(dir, ec)) return out;
            for (const auto &ent: std::filesystem::directory_iterator(dir, ec)) {
                std::error_code fec;
                if (ent.is_regular_file(fec) && !fec) out.push_back(ent.path().generic_string());
            }
            std::sort(out.begin(), out.end());
            return out;
        }

        /// 能当观测读的成员：单文件看它自己，目录看里面的文件，格式一律按内容判，
        /// 归档里附带的 `.HDR` / `.EEF` 这类自然就滤掉了。
        std::vector<std::string> obsMembers(const std::string &local) {
            std::vector<std::string> files;
            std::error_code ec;
            if (std::filesystem::is_directory(local, ec)) files = dirFiles(local);
            else files.push_back(local);

            std::vector<std::string> out;
            for (auto &f: files) {
                if (Observer::classifyObsPath(f) != Observer::Format::Unknown) out.push_back(std::move(f));
            }
            return out;
        }

        /// 非观测去向（产品清单 / 单文件）沿用旧行为：单个成员直接取它，多个成员打开目录让人自己挑。
        std::string memberToPick(const std::string &local) {
            std::error_code ec;
            if (!std::filesystem::is_directory(local, ec)) return local;

            const std::vector<std::string> files = dirFiles(local);
            if (files.size() == 1) return files.front();

            Store::revealFile(local);
            Fetcher::setStatus(I18n::tr("dl.ui.expanded_pick", files.size()));
            return {};
        }

        /// 成员名的公共前缀，回退到最近的分隔符：`podCrx_2026.215.002.01.02.rnx` 要停在
        /// `podCrx_2026.215.`，让 002 / 004 成为分组键，而不是被拆成 "2" 和 "4"。
        std::size_t commonPrefix(const std::vector<std::string> &members) {
            if (members.empty()) return 0;
            const std::string &a = members.front();
            std::size_t n = a.size();
            for (const auto &m: members) {
                std::size_t i = 0;
                while (i < n && i < m.size() && a[i] == m[i]) ++i;
                n = i;
            }
            while (n > 0 && a[n - 1] != '.' && a[n - 1] != '_' && a[n - 1] != '-' && a[n - 1] != '/') --n;
            return n;
        }

        /// 公共前缀之后的第一段当分组键：COSMIC 是星号 002，别的产品是它自己的头一段。
        std::string groupOf(const std::string &member, const std::size_t prefix) {
            const std::string rest = prefix < member.size() ? member.substr(prefix) : std::string();
            const auto cut = rest.find_first_of("._-");
            const std::string key = cut == std::string::npos ? rest : rest.substr(0, cut);
            return key.empty() ? I18n::tr("dl.ui.other") : key;
        }

        bool containsIC(const std::string &hay, const std::string &needle) {
            if (needle.empty()) return true;
            return Str::toLower(hay).find(Str::toLower(needle)) != std::string::npos;
        }

        /// 成员选择：一个产品的展开目录里几百份观测，靠分组 + 搜索收窄，点一条就填入。
        void renderMembers(Panel &p, const float height) {
            const std::size_t prefix = commonPrefix(p.members);

            if (ImGui::SmallButton((std::string(I18n::trc("dl.ui.back")) + "##acqMemBack").c_str())) {
                p.membersOf.clear();
                p.members.clear();
                p.memberGroup.clear();
                return;
            }
            ImGui::SameLine();
            ImGui::TextDisabled("%s", I18n::tr("dl.ui.readable_obs", File::baseName(p.membersOf),
                                               p.members.size()).c_str());

            Widgets::searchBox("acqMemFind", p.memberFind, sizeof p.memberFind);

            std::vector<std::string> groups;
            for (const auto &m: p.members) {
                if (const std::string g = groupOf(m, prefix);
                    std::find(groups.begin(), groups.end(), g) == groups.end()) groups.push_back(g);
            }
            if (groups.size() > 1) {
                if (ImGui::SmallButton((std::string(I18n::trc("common.all")) +
                                        "##acqMemAll").c_str()))
                    p.memberGroup.clear();
                for (const auto &g: groups) {
                    ImGui::SameLine();
                    ImGui::PushID(g.c_str());
                    const bool on = p.memberGroup == g;
                    if (ImGui::SmallButton(((on ? "● " : "") + g + "##acqMemG").c_str()))
                        p.memberGroup = on ? std::string() : g;
                    ImGui::PopID();
                }
            }

            std::string chosen;
            bool picked = false;
            ImGui::BeginChild("##acqMemList", ImVec2(0.0f, height > 0.0f ? height : 220.0f),
                              ImGuiChildFlags_Borders);
            int shown = 0;
            for (const auto &m: p.members) {
                if (!p.memberGroup.empty() && groupOf(m, prefix) != p.memberGroup) continue;
                // 公共前缀（`podCrx_2026.215.`）不重复画，留下的 `002.01.01.rnx` 才是要挑的东西。
                const std::string leaf = prefix < m.size() ? m.substr(prefix) : File::baseName(m);
                if (!containsIC(leaf, p.memberFind)) continue;
                ++shown;
                const bool on = p.isPicked(m);
                ImGui::PushID(m.c_str());
                if (ImGui::Selectable(((on ? "√" : "   ") + leaf).c_str(), on)) {
                    chosen = m;
                    picked = on;
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", m.c_str());
                ImGui::PopID();
            }
            if (shown == 0) ImGui::TextDisabled("%s", I18n::trc("dl.ui.no_obs_match"));
            ImGui::EndChild();

            if (!chosen.empty()) {
                p.pick(chosen, picked);
                p.membersOf.clear();
                p.members.clear();
                p.memberGroup.clear();
            }
        }
    }

    bool Panel::isPicked(const std::string &local) const {
        if (local.empty()) return false;
        if (kind == Kind::Obs) {
            const std::string cur = AppConfig::PathMemory::instance().get(obsKey);
            if (cur == local) return true;
            // 归档产品落成目录，目录里任何一个成员被选入都算这个产品已选。
            return cur.size() > local.size() && cur.compare(0, local.size(), local) == 0 &&
                   (cur[local.size()] == '/' || cur[local.size()] == '\\');
        }
        if (kind == Kind::Single) return singleOut && *singleOut == local;
        return target && std::find(target->begin(), target->end(), local) != target->end();
    }

    void Panel::pick(const std::string &local, const bool selected) const {
        if (kind == Kind::Obs) {
            const auto &pm = AppConfig::PathMemory::instance();
            const std::string pickPath = selected ? std::string() : local;
            pm.set(obsKey, pickPath);
            pm.flush();
            if (obsOut) *obsOut = pickPath;
            return;
        }
        if (kind == Kind::Single) {
            const std::string pickPath = selected ? std::string() : local;
            if (singleOut) *singleOut = pickPath;
            if (onPick) onPick(pickPath);
            return;
        }
        if (!target) return;
        if (selected)
            target->erase(std::remove(target->begin(), target->end(), local), target->end());
        else
            target->push_back(local);
        if (owner) {
            owner->syncFromFiles();
            owner->save();
        }
    }

    bool render(Panel &p, const char *idScope) {
        if (!p.open) return false;
        if (p.kind == Panel::Kind::Products && !p.target) {
            p.open = false;
            return false;
        }
        if (p.kind == Panel::Kind::Single && (!p.singleOut || p.singleType == ProductType::Unknown)) {
            p.open = false;
            return false;
        }
        const bool isObs = p.kind == Panel::Kind::Obs;
        const bool isSingle = p.kind == Panel::Kind::Single;

        ImGui::PushID(idScope);
        ImGui::PushID(&p);
        ImGui::Indent(12.0f);

        ImGui::SameLine();
        if (isObs) {
            const std::string cur = AppConfig::PathMemory::instance().get(p.obsKey);
            ImGui::TextDisabled("%s", I18n::tr("dl.ui.current_path",
                                               cur.empty() ? std::string(I18n::trc("dl.ui.unset"))
                                                           : File::baseName(cur)).c_str());
        } else if (isSingle) {
            const std::string cur = p.singleOut ? *p.singleOut : std::string();
            ImGui::TextDisabled("%s", I18n::tr("dl.ui.current_path",
                                               cur.empty() ? std::string(I18n::trc("dl.ui.unset"))
                                                           : File::baseName(cur)).c_str());
        } else {
            ImGui::TextDisabled("%s", I18n::tr("dl.ui.picked_count", p.target->size()).c_str());
        }

        if (p.year <= 0 || p.doy <= 0) {
            const ObsDate def = defaultObsDate();
            p.year = def.year;
            p.doy = def.doy;
        }
        {
            YearMonthDay ymd = YearMonthDay::fromDOY(p.year, p.doy).value_or(YearMonthDay{p.year, 1, 1});
            Widgets::dateRow(ymd.year, ymd.month, ymd.day, "acq", I18n::trc("dl.ui.yesterday"),
                             yesterday, true);
            p.year = ymd.year;
            p.doy = ymd.toDOY();
        }

        // 观测页只有一种类型可筛；单文件模式只显示它自己那类。
        if (!isObs) {
            if (isSingle) {
                ImGui::TextDisabled("%s", typeLabel(p.singleType));
                ImGui::SameLine();
            } else {
                // 首次展开按模式定默认勾选：低轨卫星在大气层外，对流层延迟与映射函数都用不上。
                if (p.typeOn.size() != static_cast<std::size_t>(kDateTypeCount)) {
                    p.typeOn.assign(kDateTypeCount, 1);
                    if (!p.usesTropo) {
                        p.typeOn[static_cast<int>(ProductType::TRO)] = 0;
                        p.typeOn[static_cast<int>(ProductType::VMF)] = 0;
                    }
                }
                Widgets::typeChips(p.typeOn);
            }
        }

        Widgets::searchBox("acqFind", p.nameFind, sizeof p.nameFind);

        ImGui::TextUnformatted(I18n::trc("dl.ui.local_state"));
        ImGui::SameLine();
        Widgets::stateFilterChips("##acqState", &p.stateFilter);

        SourceTable::Query q;
        q.year = p.year;
        q.doy = p.doy;
        const bool isLeoObs = isObs && p.obsType == ProductType::LEO_OBS;
        if (isObs) {
            q.types = {p.obsType};
            if (isLeoObs) q.scanTextKey = "dl.status.reading_leo_obs";
        } else if (isSingle) {
            q.types = {p.singleType};
        } else {
            for (int i = 0; i < kDateTypeCount && i < static_cast<int>(p.typeOn.size()); ++i)
                if (p.typeOn[i]) q.types.insert(static_cast<ProductType>(i));
        }

        const std::vector<SourceTable::Dir> dirs = isLeoObs
                ? SourceTable::LEOObs(q.year, q.doy)
                : SourceTable::daily(q);
        const auto snap = Fetcher::collect(dirs, q);

        float listH = p.height;
        const float availH = ImGui::GetContentRegionAvail().y - 46.0f;
        const float maxH = availH > 120.0f ? availH : 640.0f;
        if (listH > maxH) listH = maxH;
        if (listH < 96.0f) listH = 96.0f;

        Widgets::ListOptions opt;
        opt.grouping = isObs || isSingle ? Widgets::Grouping::None : Widgets::Grouping::ByType;
        opt.stateFilter = p.stateFilter;
        opt.find = p.nameFind;
        opt.isSelected = [&p](const std::string &local) { return p.isPicked(local); };
        opt.localHint = [isObs](const bool selected) {
            if (isObs)
                return selected ? I18n::tr("dl.ui.hint.remove_obs")
                                : I18n::tr("dl.ui.hint.pick_obs");
            return selected ? I18n::tr("dl.ui.hint.remove") : I18n::tr("dl.ui.hint.add");
        };
        opt.onClick = [&p, isObs](const Fetcher::Match &m, const std::string &local) {
            if (!local.empty()) {
                if (isObs) {
                    // 观测要的是一份文件：单份直接填，多份（COSMIC 按星号 + 弧段切）就地展开成员列表。
                    const std::vector<std::string> members = obsMembers(local);
                    if (members.size() == 1) {
                        p.pick(members.front(), p.isPicked(local));
                    } else if (members.empty()) {
                        Fetcher::setStatus(I18n::tr("dl.ui.no_readable_obs"));
                    } else {
                        p.membersOf = local;
                        p.members = members;
                        p.memberGroup.clear();
                        p.memberFind[0] = '\0';
                    }
                    return;
                }
                if (const std::string target = memberToPick(local); !target.empty())
                    p.pick(target, p.isPicked(local));
                return;
            }
            if (const std::string url = m.remoteUrl(); !url.empty()) Fetcher::enqueue(Fetcher::pendingOf(m));
        };

        if (isObs && !p.membersOf.empty()) {
            renderMembers(p, listH);
        } else {
            Widgets::remoteList(snap, opt, "##acqList", I18n::trc("common.file"), listH);
        }
        UI::heightGrip("##acqGrip", p.height, 96.0f, maxH);

        if (ImGui::SmallButton((std::string(I18n::trc("dl.ui.collapse")) + "##acqClose").c_str()))
            p.open = false;

        Widgets::deleteConfirm();

        ImGui::Unindent(12.0f);
        ImGui::PopID();
        ImGui::PopID();
        return p.open;
    }
}
