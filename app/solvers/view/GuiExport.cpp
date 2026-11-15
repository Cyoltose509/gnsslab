#include "GuiExport.h"

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

#include "ui/FileDialog.h"
#include "FileUtils.h"
#include "Log.h"
#include "solvers/common/Config.h"
#include "solvers/common/Task.h"
#include "imgui.h"
#include "i18n/I18n.h"

namespace Solvers::ExportUI {
    namespace {
        using Export::Field;

        struct State {
            bool wantOpen = false;
            std::shared_ptr<Task> task;
            Export::Format format = Export::Format::CSV;
            int samplingMode = 0; // 0 原样 / 1 每 N 秒 / 2 每 N 历元
            double seconds = 30.0;
            int step = 1;
            int beginIdx = 0;
            int endIdx = 0;
            std::vector<Export::Col> all;
            std::vector<Export::FieldGroup> groups;
            std::vector<char> sel;
            char satId[4] = "L01";
            char marker[24] = "gnssLab";
            bool sp3Vel = false;
            int posForm = 0; // 0 经纬高 / 1 ECEF / 2 ENU
            bool posVel = true;
            bool posHead = true;
            std::string status;
        };

        State g;

        void resetFields() {
            if (!g.task) return;
            g.all = g.task->exportFields(g.format);
            g.groups = Export::groupsFor(g.all);
            const auto defaults = g.task->exportDefaults(g.format);
            g.sel.assign(g.all.size(), 0);
            for (std::size_t k = 0; k < g.all.size(); ++k) {
                const Field f = g.all[k].field;
                const bool on = std::any_of(defaults.begin(), defaults.end(),
                                            [f](const Export::Col &c) { return c.field == f; });
                g.sel[k] = on ? 1 : 0;
            }
        }

        /// 组内各列共用一个勾选框，勾选状态取「只要有一列在勾上就算勾」。
        bool groupOn(const Export::FieldGroup &grp) {
            for (const Export::Col &c: grp.cols)
                for (std::size_t k = 0; k < g.all.size(); ++k)
                    if (g.all[k].field == c.field && g.sel[k]) return true;
            return false;
        }

        void setGroup(const Export::FieldGroup &grp, const bool on) {
            for (const Export::Col &c: grp.cols)
                for (std::size_t k = 0; k < g.all.size(); ++k)
                    if (g.all[k].field == c.field) g.sel[k] = on ? 1 : 0;
        }

        bool matchesDefault() {
            if (!g.task) return true;
            const auto defaults = g.task->exportDefaults(g.format);
            std::vector<Field> picked;
            for (std::size_t k = 0; k < g.all.size(); ++k)
                if (g.sel[k]) picked.push_back(g.all[k].field);
            if (picked.size() != defaults.size()) return false;
            for (std::size_t k = 0; k < picked.size(); ++k)
                if (picked[k] != defaults[k].field) return false;
            return true;
        }

        std::wstring extOf(const Export::Format fmt) {
            return Str::ACPToWide(Export::fileExt(fmt));
        }

        void doExport() {
            if (!g.task) return;
            Export::Spec spec;
            spec.format = g.format;
            spec.sampling.mode = g.samplingMode == 0
                                     ? Export::Sampling::Mode::Native
                                     : g.samplingMode == 1
                                           ? Export::Sampling::Mode::Seconds
                                           : Export::Sampling::Mode::Step;
            spec.sampling.seconds = g.seconds;
            spec.sampling.step = g.step;
            spec.begin = g.beginIdx > 0 ? static_cast<std::size_t>(g.beginIdx) : 0;
            spec.end = g.endIdx > 0 ? static_cast<std::size_t>(g.endIdx) : 0;
            spec.satId = g.satId;
            spec.marker = g.marker;
            spec.sp3Velocity = g.sp3Vel;
            spec.posForm = static_cast<Export::PosForm>(g.posForm);
            spec.posVelocity = g.posVel;
            spec.posHeader = g.posHead;
            if (g.format == Export::Format::CSV) {
                std::vector<Field> picked;
                for (std::size_t k = 0; k < g.all.size(); ++k)
                    if (g.sel[k]) picked.push_back(g.all[k].field);
                if (picked.empty()) { // 空列表会被下游当成「按默认列写」，宁可不出文件
                    g.status = I18n::trc("export.nothing_selected");
                    return;
                }
                if (!matchesDefault()) spec.fields = std::move(picked);
            }

            const std::string dn = defaultName(*g.task, g.format);
            const std::wstring ext = extOf(g.format);
            const std::vector<GuiFileFilter> filters = {FileDialog::make("export.dialog_title", {ext}),
                                                        FileDialog::all()};
            std::wstring path;
            if (!FileDialog::save(dn, path, filters, ext.c_str())) return;

            if (std::string error; !File::safeWrite(std::filesystem::path(path), [&](std::ostream &out) {
                    std::lock_guard lk(g.task->mutex);
                    g.task->exportTo(out, spec);
                }, error)) {
                g.status = error;
                return;
            }
            LOG_INFO << "[EXPORT] " << Export::formatName(g.format) << " → "
                    << std::filesystem::path(path).string();
            g.status = I18n::tr("export.written", std::filesystem::path(path).filename().string());
        }
    }

    std::string defaultName(const Task &task, const Export::Format fmt) {
        std::string base = task.exportStem();
        if (base.empty()) base = "result";
        base += "_" + std::string(modeLabel(task.mode));
        switch (fmt) {
            case Export::Format::POS: base += ".pos";
                break;
            case Export::Format::SP3: base += ".sp3";
                break;
            case Export::Format::ORB: base += "_elements.csv";
                break;
            case Export::Format::TRAJ: base += "_traj.txt";
                break;
            case Export::Format::KML: base += "_traj.kml";
                break;
            case Export::Format::SBET: base += ".sbet";
                break;
            default: base += "_result.csv";
                break;
        }
        return base;
    }

    void open(const std::shared_ptr<Task> &task) {
        g.task = task;
        g.format = Export::Format::CSV;
        g.samplingMode = 0;
        g.seconds = 30.0;
        g.step = 1;
        g.beginIdx = 0;
        g.endIdx = 0;
        g.posForm = 0;
        g.posVel = true;
        g.posHead = true;
        g.status.clear();
        resetFields();
        g.wantOpen = true;
    }

    void render() {
        if (!g.task) return;
        if (g.wantOpen) {
            ImGui::OpenPopup("##export_dialog");
            g.wantOpen = false;
        }
        ImGui::SetNextWindowSize(ImVec2(860, 620), ImGuiCond_Appearing);
        if (!ImGui::BeginPopupModal("##export_dialog", nullptr, ImGuiWindowFlags_NoSavedSettings)) return;

        const Mode mode = g.task->mode;
        const bool hasRef = g.task->refECEF.squaredNorm() > 1.0;

        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(I18n::trc("export.format"));
        const auto fmts = Export::formatsFor(mode);
        for (const auto fmt : fmts) {
            ImGui::SameLine();
            if (ImGui::RadioButton(Export::formatName(fmt), g.format == fmt)) {
                g.format = fmt;
                resetFields();
            }
        }

        ImGui::Separator();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(I18n::trc("export.sampling"));
        ImGui::SameLine();
        ImGui::RadioButton(I18n::trc("export.same_as_solution"), &g.samplingMode, 0);
        ImGui::SameLine();
        ImGui::RadioButton(I18n::trc("export.every_n_sec"), &g.samplingMode, 1);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(72);
        ImGui::InputDouble("##sec", &g.seconds, 0.0, 0.0, "%.1f");
        ImGui::SameLine();
        ImGui::RadioButton(I18n::trc("export.every_n_epoch"), &g.samplingMode, 2);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(72);
        ImGui::InputInt("##step", &g.step, 0, 0);
        if (g.seconds < 0.1) g.seconds = 0.1;
        if (g.step < 1) g.step = 1;
        if (g.format == Export::Format::SP3 && g.samplingMode != 1)
            ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.3f, 1.0f), I18n::trc("export.sp3_grid_hint"));

        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(I18n::trc("export.range"));
        ImGui::SameLine();
        ImGui::TextUnformatted(I18n::trc("export.start_epoch"));
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ImGui::InputInt("##begin", &g.beginIdx, 0, 0);
        ImGui::SameLine();
        ImGui::TextUnformatted(I18n::trc("export.end_epoch"));
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ImGui::InputInt("##end", &g.endIdx, 0, 0);
        ImGui::SameLine();
        ImGui::TextDisabled(I18n::trc("export.zero_end"));
        if (g.beginIdx < 0) g.beginIdx = 0;
        if (g.endIdx < 0) g.endIdx = 0;
        ImGui::Separator();
        if (g.format == Export::Format::SP3) {
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(I18n::trc("export.sp3_header"));
            ImGui::SameLine();
            ImGui::TextUnformatted(I18n::trc("export.track_id"));
            ImGui::SameLine();
            ImGui::SetNextItemWidth(52);
            ImGui::InputText("##satId", g.satId, sizeof(g.satId));
            ImGui::SameLine();
            ImGui::TextUnformatted(I18n::trc("export.agency"));
            ImGui::SameLine();
            ImGui::SetNextItemWidth(96);
            ImGui::InputText("##marker", g.marker, sizeof(g.marker));
            ImGui::Checkbox(I18n::trc("export.write_velocity"), &g.sp3Vel);
        }

        if (g.format == Export::Format::POS) {
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(I18n::trc("export.position_cols"));
            ImGui::SameLine();
            ImGui::RadioButton(I18n::trc("BLH"), &g.posForm, 0);
            ImGui::SameLine();
            ImGui::RadioButton("ECEF", &g.posForm, 1);
            ImGui::SameLine();
            ImGui::RadioButton("ENU", &g.posForm, 2);
            ImGui::Checkbox(I18n::trc("export.write_velocity"), &g.posVel);
            ImGui::SameLine();
            ImGui::Checkbox(I18n::trc("export.file_header"), &g.posHead);
            if (g.posForm == 2 && !hasRef) ImGui::TextDisabled(I18n::trc("export.no_ref_enu_blh"));
        }

        if (g.format == Export::Format::CSV) {
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(I18n::trc("export.elements"));
            ImGui::SameLine();
            if (ImGui::SmallButton(I18n::trc("common.select_all"))) std::fill(g.sel.begin(), g.sel.end(), 1);
            ImGui::SameLine();
            if (ImGui::SmallButton(I18n::trc("common.clear"))) std::fill(g.sel.begin(), g.sel.end(), 0);
            ImGui::SameLine();
            if (ImGui::SmallButton(I18n::trc("export.reset_default"))) resetFields();
            if (const float rowH = std::max(120.0f, ImGui::GetContentRegionAvail().y - 96.0f);
                ImGui::BeginChild("##fields", ImVec2(0, rowH), ImGuiChildFlags_Borders)) {
                constexpr int kCols = 3;
                const int rows = static_cast<int>((g.groups.size() + kCols - 1) / kCols);
                if (ImGui::BeginTable("##fieldTable", kCols)) {
                    for (int r = 0; r < rows; ++r) {
                        ImGui::TableNextRow();
                        for (int c = 0; c < kCols; ++c) {
                            const auto gi = r + c * rows;
                            if (gi >= g.groups.size()) break;
                            ImGui::TableSetColumnIndex(c);
                            const Export::FieldGroup &grp = g.groups[gi];
                            bool on = groupOn(grp);
                            if (ImGui::Checkbox(("##g" + std::to_string(gi)).c_str(), &on)) setGroup(grp, on);
                            ImGui::SameLine();
                            ImGui::TextUnformatted(grp.label);
                            if (grp.cols.size() > 1 && ImGui::IsItemHovered()) {
                                std::string tip = I18n::tr("export.tip_cols", grp.cols.size());
                                for (std::size_t j = 0; j < grp.cols.size(); ++j) {
                                    if (j) tip += I18n::trv("export.list_sep");
                                    tip += grp.cols[j].name;
                                }
                                ImGui::SetTooltip("%s", tip.c_str());
                            }
                        }
                    }
                    ImGui::EndTable();
                }
            }
            ImGui::EndChild();
        } else {
            ImGui::Dummy(ImVec2(0, 240));
        }

        ImGui::Separator();
        if (ImGui::Button(I18n::trc("export.export"), ImVec2(140, 0))) {
            doExport();
            if (!g.status.empty() && g.status.rfind(I18n::trc("export.written_prefix"), 0) == 0) ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button(I18n::trc("common.cancel"), ImVec2(140, 0))) ImGui::CloseCurrentPopup();
        if (!g.status.empty()) ImGui::TextUnformatted(g.status.c_str());
        ImGui::EndPopup();
        // 弹窗关了就放手：任务可能已被关页，导出面板不该再拖住它
        if (!ImGui::IsPopupOpen("##export_dialog")) g.task.reset();
    }
}
