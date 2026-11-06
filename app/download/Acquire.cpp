#include "Acquire.h"

#include "Remote.h"
#include "ProductMeta.h"
#include "Widgets.h"
#include "Store.h"

#include "core/PathMemory.h"
#include "io/Products.h"

#include "StringUtils.h"
#include "TimeConvert.h"

#include "imgui.h"

#include <algorithm>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace Download::Acquire {
    namespace {
        /// 昨日（本地时区）：IGS 当日观测要到次日才发布，默认翻到昨天。
        void yesterday(int &y, int &m, int &d) {
            const auto [year, doy] = ProductMeta::defaultObsDate();
            if (const auto ymd = Time::DOYToYMD(year, doy)) {
                y = ymd->year;
                m = ymd->month;
                d = ymd->day;
            }
        }
    }

    bool Panel::isPicked(const std::string &local) const {
        if (local.empty()) return false;
        if (kind == Kind::Obs) return AppConfig::PathMemory::instance().get(obsKey) == local;
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
            Widgets::dateRow(ymd.year, ymd.month, ymd.day, "acq", "昨日", yesterday, true);
            p.year = ymd.year;
            p.doy = Time::YMDToDOY(ymd);
        }

        std::set<ProductType> types;
        if (isObs)
            types.insert(ProductType::OBS);
        else if (isSingle)
            types.insert(p.singleType);
        else
            for (int i = 0; i < kDateTypeCount; ++i) types.insert(static_cast<ProductType>(i));

        const auto &dcs = ProductMeta::knownDatacenters();
        std::set<std::string> dcSel;
        if (!isObs)
            for (std::size_t i = 0; i < p.dcOn.size() && i < dcs.size(); ++i)
                if (p.dcOn[i]) dcSel.insert(dcs[i]);

        const std::vector<std::string> dirs = ProductMeta::scanDirs(types, p.year, p.doy);
        const auto snap = Remote::collect(dirs, types, p.year, p.doy, dcSel);

        if (!isObs) {
            if (!isSingle) {
                if (p.typeOn.size() != static_cast<std::size_t>(kDateTypeCount))
                    p.typeOn.assign(kDateTypeCount, 1);

                ImGui::TextUnformatted("产品类型");
                Widgets::chipWrap([](const int i) { return kTypeLabels[i]; }, p.typeOn);
            } else {
                ImGui::TextDisabled("%s", typeLabel(p.singleType));
                ImGui::SameLine();
            }

            if (isSingle && p.singleType == ProductType::NAV) {
                ImGui::TextDisabled("（广播星历不区分数据中心）");
            } else {
                if (p.dcOn.size() != dcs.size()) p.dcOn.assign(dcs.size(), 0);
                ImGui::TextUnformatted("数据中心");
                Widgets::chipWrap([&dcs](const int i) { return dcs[static_cast<std::size_t>(i)].c_str(); },
                                  p.dcOn);
            }
        }

        Widgets::searchBox("acqFind", p.nameFind, sizeof p.nameFind);

        ImGui::TextUnformatted("本地状态");
        ImGui::SameLine();
        Widgets::stateFilterChips("##acqState", &p.stateFilter);

        // 类型筛选；关键字与本地状态由 Widgets::fileList 统一处理
        std::vector<Remote::Match> matches;
        matches.reserve(snap.matches.size());
        for (const auto &mt: snap.matches) {
            const auto &m = mt.m;
            if (isSingle && m.type != p.singleType) continue;
            if (!isObs && !isSingle) {
                if (const auto t = static_cast<std::size_t>(m.type);
                    t >= static_cast<std::size_t>(kDateTypeCount) || !p.typeOn[t])
                    continue;
            }
            matches.push_back(mt);
        }

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
        opt.localHint = [](const bool selected) {
            return selected ? std::string("（左键点击可移出清单 · 右键管理）")
                            : std::string("（左键点击可加入清单 · 右键管理）");
        };
        opt.onClick = [&p](const Remote::Match &m, const std::string &local) {
            if (!local.empty()) {
                p.pick(local, p.isPicked(local));
                return;
            }
            if (!m.dirUrl.empty())
                Remote::enqueue({m.m.filename, m.dirUrl, m.m.type, m.m.year, m.m.doy, m.m.spanDays});
        };

        Widgets::ListResult res;
        ImGui::BeginChild("##acqList", ImVec2(0, listH), ImGuiChildFlags_Borders);
        res = Widgets::fileList(matches, opt);
        ImGui::EndChild();

        Widgets::heightGrip("##acqGrip", p.height, 96.0f, maxH);

        {
            std::string status;
            if (!Widgets::progressLine(status)) {
                if (p.stateFilter != 0 || *p.nameFind)
                    ImGui::TextDisabled("共 %d / %d 个文件", res.shown, res.total);
                else ImGui::TextDisabled("共 %d 个文件", res.shown);
            }
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("收起##acqClose")) p.open = false;

        Widgets::deleteConfirm();

        ImGui::Unindent(12.0f);
        ImGui::PopID();
        ImGui::PopID();
        return p.open;
    }
}
