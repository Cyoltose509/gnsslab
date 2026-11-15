#include "GuiHelpers.h"
#include "GuiParamPage.h"

#include "core/PathMemory.h"
#include "io/ProductPaths.h"
#include "core/atmo/Troposphere.h"
#include "solvers/common/Config.h"
#include "ui/ParamPage.h"
#include "download/local/Store.h"
#include "download/remote/Catalog.h"
#include "download/ui/Widgets.h"

#include <array>
#include <string>
#include <vector>
#include "i18n/I18n.h"
#include "ui/ImGuiLabel.h"

template<>
struct UI::ParamPage::Traits<Troposphere::Map> {
    static void store(const std::string &k, const Troposphere::Map v) {
        AppConfig::PathMemory::instance().set(k, Troposphere::keyOf(v));
    }

    static void fetch(const std::string &k, Troposphere::Map &v, const Range &) {
        v = Troposphere::fromKey(
            AppConfig::PathMemory::instance().get(k, Troposphere::keyOf(v)));
    }
};

template<>
struct UI::ParamPage::Traits<SolverOptions::FilterMode> {
    static const char *keyOf(const SolverOptions::FilterMode v) { return Solvers::filterKey(v); }

    static SolverOptions::FilterMode fromKey(const std::string &s, const SolverOptions::FilterMode def) {
        return Solvers::filterFromKey(s, def);
    }

    static void store(const std::string &k, const SolverOptions::FilterMode v) {
        AppConfig::PathMemory::instance().set(k, keyOf(v));
    }

    static void fetch(const std::string &k, SolverOptions::FilterMode &v, const Range &) {
        v = fromKey(AppConfig::PathMemory::instance().get(k, keyOf(v)), v);
    }
};

namespace {
    using SolverField = UI::ParamPage::Field<SolverOptions>;
    using SolverGroup = UI::ParamPage::Group<SolverOptions>;
    using UI::ParamPage::between;
    using UI::ParamPage::field;
    using UI::ParamPage::flag;

    SolverField rtkPhaseWeight(const char *label, const char *key, double Weight::*m,
                               const UI::ParamPage::Range &r = UI::ParamPage::unbounded) {
        return UI::ParamPage::makeField<UI::ParamPage::kValue, SolverOptions, double>(
            label, key, r,
            [m](SolverOptions &o) -> double & { return o.rtk.phaseWeight.*m; },
            [m](const SolverOptions &o) -> const double & { return o.rtk.phaseWeight.*m; });
    }

    const char *tropoMapItem(void *, const int idx) { return Troposphere::kModels[idx].name; }

    SolverField comboTropoMap(const char *label, const char *key,
                              SPPConfig SolverOptions::*owner, Troposphere::Map SPPConfig::*m) {
        auto at = [m, owner](SolverOptions &o) -> Troposphere::Map & { return o.*owner.*m; };
        auto atc = [m, owner](const SolverOptions &o) -> const Troposphere::Map & { return o.*owner.*m; };
        SolverField f;
        f.label = label;
        f.key = key;
        f.draw = [label, at](SolverOptions &o) {
            int cur = 0;
            for (int i = 0; i < Troposphere::kModelCount; ++i)
                if (Troposphere::kModels[i].id == at(o)) cur = i;
            UI::ParamPage::beginField(label);
            if (ImGui::Combo((std::string("##combo_") + label).c_str(), &cur, tropoMapItem, nullptr,
                             Troposphere::kModelCount))
                at(o) = Troposphere::kModels[cur].id;
        };
        f.store = [key, atc](const SolverOptions &o, const std::string &p) {
            UI::ParamPage::Traits<Troposphere::Map>::store(p + key, atc(o));
        };
        f.fetch = [key, at](SolverOptions &o, const std::string &p) {
            UI::ParamPage::Traits<Troposphere::Map>::fetch(p + key, at(o), UI::ParamPage::unbounded);
        };
        f.reset = [at, atc](SolverOptions &o, const SolverOptions &d) { at(o) = atc(d); };
        return f;
    }

    const char *filterItem(void *, const int idx) { return Solvers::filterLabel(Solvers::kFilterChoices[idx].mode); }

    /// 滤波方向下拉：档位表与 `smooth` 键共用一套词表；渲染时把存档里的未知档归一到第一档。
    SolverField comboFilter(const char *label) {
        constexpr int count = static_cast<int>(std::size(Solvers::kFilterChoices));
        SolverField f;
        f.label = label;
        f.key = "smooth";
        f.draw = [label, count](SolverOptions &o) {
            int cur = 0;
            for (int i = 0; i < count; ++i)
                if (Solvers::kFilterChoices[i].mode == o.filter) cur = i;
            o.filter = Solvers::kFilterChoices[cur].mode;
            UI::ParamPage::beginField(label);
            if (ImGui::Combo("##filter", &cur, filterItem, nullptr, count))
                o.filter = Solvers::kFilterChoices[cur].mode;
        };
        f.store = [](const SolverOptions &o, const std::string &p) {
            UI::ParamPage::Traits<SolverOptions::FilterMode>::store(p + "smooth", o.filter);
        };
        f.fetch = [](SolverOptions &o, const std::string &p) {
            UI::ParamPage::Traits<SolverOptions::FilterMode>::fetch(p + "smooth", o.filter,
                                                                    UI::ParamPage::unbounded);
        };
        f.reset = [](SolverOptions &o, const SolverOptions &d) { o.filter = d.filter; };
        return f;
    }

    SolverGroup obsGroup(const bool withPhaseSigma) {
        SolverGroup g{
            "param.group.obs_model", {
                field("param.code_sigma", "CodeSigma", &SPPConfig::codeSigma, &SolverOptions::spp),
            }
        };
        if (withPhaseSigma)
            g.fields.push_back(field("param.phase_sigma", "PhaseSigma", &SPPConfig::phaseSigma, &SolverOptions::spp));
        g.fields.push_back(field("param.elev_weight_floor", "minSinElev", &SPPConfig::minSinElev,
                                 &SolverOptions::spp));
        return g;
    }

    SolverField systemsField() {
        SolverField f;
        f.label = "param.constellations";
        f.key = "systems";
        f.draw = [](SolverOptions &o) {
            UI::ParamPage::beginField("param.constellations", false);
            bool first = true;
            for (const auto &[ch, name]: Solvers::Helpers::kConstellations) {
                if (!first) ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x * 2.0f);
                first = false;
                ImGui::TextUnformatted(name);
                ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
                bool on = o.systems.count(ch) > 0;
                if (ImGui::Checkbox((std::string("##sys_") + name).c_str(), &on)) {
                    if (on) o.systems.insert(ch);
                    else o.systems.erase(ch);
                }
            }
        };
        f.store = [](const SolverOptions &o, const std::string &p) {
            AppConfig::PathMemory::instance().set(p + "systems", Solvers::sysSetToString(o.systems));
        };
        f.fetch = [](SolverOptions &o, const std::string &p) {
            if (const std::string s = AppConfig::PathMemory::instance().get(p + "systems"); !s.empty())
                o.systems = Solvers::stringToSysSet(s);
        };
        f.reset = [](SolverOptions &o, const SolverOptions &d) { o.systems = d.systems; };
        return f;
    }

    /// 解算设置：采样/星座/高度角；滤波方向对所有有跨历元状态的模式统一放这。
    SolverGroup obsConditionGroup(const Solvers::Mode mode) {
        SolverGroup g{
            "param.group.solve", {
                field("param.sample_interval", "interval", &SolverOptions::sampleInterval,
                      between(0.0, 3600.0)),
                systemsField(),
                field("param.elev_cutoff", "cutoff", &SolverOptions::cutoffDeg, between(0.0, 90.0)),
            }
        };
        if (Solvers::modeHasFilter(mode)) g.fields.push_back(comboFilter("param.filter_dir"));
        return g;
    }

    void renderAutoProduct(const char *productId, const bool enabled,
                           const char *offText = "param.auto_off") {
        constexpr ImVec4 kGreen(0.35f, 0.85f, 0.45f, 1.0f);
        constexpr ImVec4 kAmber(0.95f, 0.75f, 0.25f, 1.0f);

        if (!enabled) {
            ImGui::TextDisabled("%s", I18n::trc(offText));
            return;
        }

        if (const auto st = Download::Catalog::updateStatus(productId); st.phase == Download::Catalog::UpdateStatus::Downloading) {
            Download::Widgets::renderProgress("ref.downloading", st.file, st.progress, st.gotBytes, st.totalBytes);
            return;
        }

        if (const auto inst = Download::Store::active(productId)) {
            ImGui::TextColored(kGreen, "%s", I18n::tr("param.auto_loaded", inst->file).c_str());
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", I18n::tr("param.auto_version", inst->path, inst->version,
                                                 inst->fetchedAt).c_str());
            return;
        }

        const auto *meta = Download::Catalog::find(productId);
        ImGui::TextColored(kAmber, "%s", I18n::tr("param.not_found", meta ? typeLabel(meta->type)
                                                                         : productId).c_str());
        ImGui::SameLine();
        if (ImGui::SmallButton(UI::idLabel("common.check", "##get_", productId)))
            Download::Catalog::checkNow(productId);
    }

    SolverGroup corrGroup() {
        return SolverGroup{
            "param.group.correction", {
                flag("AntennaCorrection", "applyAtx", &SPPConfig::applyAtx, &SolverOptions::spp),
                flag("param.phase_windup", "applyWindup", &SPPConfig::applyWindup, &SolverOptions::spp),
                flag("param.solid_tide", "applySolidTide", &SPPConfig::applySolidTide, &SolverOptions::spp),
            }
        };
    }

    SolverGroup tropoGroup(const Solvers::Mode mode, const bool withGrad, const bool withZtd = false) {
        SolverGroup g{"Troposphere", {}};
        if (withGrad)
            g.fields.push_back(flag("param.tropo_gradient", "useTroGrad", &SPPConfig::useTroGrad, &SolverOptions::spp));
        g.fields.push_back(comboTropoMap("MappingFunction", "tropoMap", &SolverOptions::spp, &SPPConfig::tropoMap));
        if (withZtd) {
            g.title = "param.group.tropo_ztd";
            g.fields.push_back(field("param.ztd_init", "ztdInitValue", &SPPConfig::ztdInitValue, &SolverOptions::spp));
            g.fields.push_back(field("param.ztd_init_sigma", "ZtdInitSigma", &SPPConfig::ztdInitSigma, &SolverOptions::spp));
            g.fields.push_back(field("param.ztd_proc_sigma", "ZtdProcSigma", &SPPConfig::ztdProcSigma, &SolverOptions::spp));
            g.fields.push_back(field("param.ztd_prior_sigma", "ztdPriorSigma", &SPPConfig::ztdPriorSigma,
                                     &SolverOptions::spp));
            g.fields.push_back(field("param.ztd_constraint_sigma", "troZtdSigma", &SPPConfig::troZtdSigma, &SolverOptions::spp));
        }
        g.row = [mode](const SolverOptions &o) {
            constexpr ImVec4 kGreen(0.35f, 0.85f, 0.45f, 1.0f);
            constexpr ImVec4 kAmber(0.95f, 0.75f, 0.25f, 1.0f);
            if (o.spp.tropoMap == Troposphere::Map::VMF3) {
                const auto &prod = AppConfig::ProductPaths::of(Solvers::modeKey(mode));
                if (const auto &vmf = prod.vmf; vmf.empty()) {
                    ImGui::TextColored(kAmber, "%s", I18n::trc("param.vmf3_missing"));
                    return;
                }
                const int n = static_cast<int>(prod.VMF3Paths().size());
                ImGui::TextColored(kGreen, "%s", I18n::tr("param.vmf3_loaded", n).c_str());
                if (n < 4)
                    ImGui::TextColored(kAmber, "%s", I18n::tr("param.vmf3_partial", n).c_str());
                renderAutoProduct(Download::Catalog::kGPT3ID, true, "param.gpt3_off_ahaw");
                return;
            }
            if (o.spp.tropoMap == Troposphere::Map::GPT3) {
                renderAutoProduct(Download::Catalog::kGPT3ID, true, "param.gpt3_off");
                return;
            }
            ImGui::TextDisabled("%s", I18n::trc("param.no_grid_needed"));
        };
        g.rowAfterKey = "tropoMap";
        return g;
    }

    SolverGroup filterIterGroup() {
        return {
            "param.group.filter_update", {
                field("param.min_sats_update", "minSatsForUpdate", &SPPConfig::minSatsForUpdate,
                      &SolverOptions::spp),
                field("param.clock_proc_sigma", "ClockProcSigma", &SPPConfig::clockProcSigma, &SolverOptions::spp),
            }
        };
    }

    SolverGroup arGroup() {
        return {
            "param.group.ambiguity", {
                flag("param.use_ar", "useAR", &SPPConfig::useAR, &SolverOptions::spp),
                field("param.amb_init_sigma", "AmbInitSigma", &SPPConfig::ambInitSigma, &SolverOptions::spp),
                field("param.amb_proc_sigma", "AmbProcSigma", &SPPConfig::ambProcSigma, &SolverOptions::spp),
                field("param.ratio_threshold", "arRatioThreshold", &SPPConfig::arRatioThreshold, &SolverOptions::spp),
            }
        };
    }

    SolverGroup codeBiasGroup() {
        return {
            "param.group.code_bias", {
                field("param.code_bias_proc_sigma", "CodeBiasProcSigma", &SPPConfig::codeBiasProcSigma, &SolverOptions::spp),
                field("param.code_bias_init_sigma", "CodeBiasInitSigma", &SPPConfig::codeBiasInitSigma, &SolverOptions::spp),
                field("param.glo_ifcb_proc_sigma", "GloIfcbProcSigma", &SPPConfig::gloIfcbProcSigma, &SolverOptions::spp),
                field("param.glo_ifcb_init_sigma", "GloIfcbInitSigma", &SPPConfig::gloIfcbInitSigma, &SolverOptions::spp),
            }
        };
    }

    /// 位置过程 σ：PPP 静态为 0（位置固定），PPP 动态与 LEO 为 100（逐历元更新）。
    /// 每个模式各存各的值，前缀不同，互不影响。
    SolverGroup posGroup(const bool leo) {
        SolverGroup g{
            "param.group.position", {
                field("param.pos_proc_sigma", "PosProcSigma", &SolverOptions::posProcSigma),
            }
        };
        g.fields.push_back(field("param.first_epoch_pos_sigma", "FirstEpochPosSigma", &SPPConfig::firstEpochPosSigma,
                                 &SolverOptions::spp));
        if (leo) {
            g.fields.push_back(field("param.motion_threshold", "motionThresh", &SolverOptions::motionThresh));
            g.fields.push_back(field("param.spp_pred_threshold", "sppVsPredThresh", &SolverOptions::sppVsPredThresh));
        }
        return g;
    }

    /// SPP 的位置/速度状态模型：加速度 σ>0 时状态含速度三态，位置按常速度预测。
    SolverGroup sppMotionGroup() {
        return {
            "param.group.pos_vel", {
                field("param.pos_proc_sigma_accel", "PosProcSigma", &SolverOptions::posProcSigma),
                field("param.acc_proc_sigma", "AccProcSigma", &SolverOptions::accProcSigma),
                field("param.vel_init_sigma", "VelInitSigma", &SolverOptions::velInitSigma),
                field("param.clock_proc_sigma_m", "ClockProcSigma", &SPPConfig::clockProcSigma, &SolverOptions::spp),
            }
        };
    }

    /// 周跳组：SPP 家族走 SPPConfig；RTK 走 RTKConfig，多一个 GF 开关。
    SolverGroup slipGroup(const bool rtk) {
        SolverGroup g{"param.group.cycle_slip", {}};
        if (rtk) {
            g.fields.push_back(flag("param.use_cycle_slip", "useCycleSlip", &RTKConfig::useCycleSlip,
                                    &SolverOptions::rtk));
            g.fields.push_back(flag("param.use_gf_slip", "useGfCycleSlip", &RTKConfig::useGfCycleSlip,
                                    &SolverOptions::rtk));
            g.fields.push_back(field("param.gf_slip_mult", "gfSlipMult", &RTKConfig::gfSlipMult,
                                     &SolverOptions::rtk));
            return g;
        }
        g.fields.push_back(flag("param.use_cycle_slip", "useCycleSlip", &SPPConfig::useCycleSlip,
                                &SolverOptions::spp));
        g.fields.push_back(field("param.gf_slip_mult", "gfSlipMult", &SPPConfig::gfSlipMult,
                                 &SolverOptions::spp));
        return g;
    }

    SolverGroup healthGroup(const bool rtk) {
        SolverGroup g{"param.group.sat_availability", {}};
        if (rtk) g.fields.push_back(flag("param.drop_unhealthy", "useHealth", &RTKConfig::useHealth,
                                         &SolverOptions::rtk));
        else g.fields.push_back(flag("param.drop_unhealthy", "useHealth", &SPPConfig::useHealth,
                                     &SolverOptions::spp));
        return g;
    }

    SolverGroup isbGroup() {
        return {
            "param.group.isb", {
                field("param.isb_proc_sigma", "IsbProcSigma", &SPPConfig::isbProcSigma, &SolverOptions::spp),
                field("param.isb_init_sigma", "IsbInitSigma", &SPPConfig::isbInitSigma, &SolverOptions::spp),
            }
        };
    }

    SolverField insScaleField(const char *label, const char *key, double ImuScale::*m,
                              const UI::ParamPage::Range &r = UI::ParamPage::unbounded) {
        return UI::ParamPage::makeField<UI::ParamPage::kValue, SolverOptions, double>(
            label, key, r,
            [m](SolverOptions &o) -> double & { return o.ins.manual.*m; },
            [m](const SolverOptions &o) -> const double & { return o.ins.manual.*m; });
    }

    using InsMem = double INSConfig::*;

    /// 三个 XYZ 分量排同一行，各自独立存档，键与 CLI ini 对齐。
    SolverField insVec3Field(const char *title, const std::array<const char *, 3> &keys,
                             const std::array<const char *, 3> &labels,
                             const std::array<InsMem, 3> &mem) {
        SolverField f;
        f.label = title;
        f.key = keys[0];
        f.draw = [title, keys, labels, mem](SolverOptions &o) {
            UI::ParamPage::beginField(title, false);
            ImGui::PushItemWidth(150);
            for (int i = 0; i < 3; ++i) {
                if (i) ImGui::SameLine();
                const std::string id = std::string(labels[i]) + "##" + keys[i];
                ImGui::InputDouble(id.c_str(), &(o.ins.*mem[i]), 0.0, 0.0, "%.10g");
            }
            ImGui::PopItemWidth();
        };
        f.store = [keys, mem](const SolverOptions &o, const std::string &p) {
            for (int i = 0; i < 3; ++i)
                UI::ParamPage::Traits<double>::store(p + keys[i], o.ins.*mem[i]);
        };
        f.fetch = [keys, mem](SolverOptions &o, const std::string &p) {
            for (int i = 0; i < 3; ++i)
                UI::ParamPage::Traits<double>::fetch(p + keys[i], o.ins.*mem[i],
                                                     UI::ParamPage::unbounded);
        };
        f.reset = [mem](SolverOptions &o, const SolverOptions &d) {
            for (int i = 0; i < 3; ++i) o.ins.*mem[i] = d.ins.*mem[i];
        };
        return f;
    }

    SolverField insManualAttitude(SolverField f) {
        f.show = [](const SolverOptions &o) { return o.ins.specifyAttitude; };
        return f;
    }

    SolverField insInitialPosField() {
        SolverField f = insVec3Field("param.ins_init_pos", {"lat", "lon", "h"},
                                     {"φ", "λ", "h"}, {&INSConfig::latDeg, &INSConfig::lonDeg, &INSConfig::hM});
        f.show = [](const SolverOptions &) { return false; };   // 只在文件页渲染
        return f;
    }

    SolverField insInitialVelField() {
        SolverField f = insVec3Field("param.ins_init_vel", {"vn", "ve", "vd"},
                                     {"vN", "vE", "vD"}, {&INSConfig::vN, &INSConfig::vE, &INSConfig::vD});
        f.show = [](const SolverOptions &) { return false; };
        return f;
    }

    const std::vector<SolverGroup> &insSchema() {
        static const std::vector<SolverGroup> s = {
            {"param.group.ins_init_state", {
                insInitialPosField(),
                insInitialVelField(),
            }},
            {"InitialAttitude", {
                flag("param.ins_specify_attitude", "specify_att", &INSConfig::specifyAttitude,
                     &SolverOptions::ins),
                insManualAttitude(field("param.ins_roll", "roll", &INSConfig::rollDeg, &SolverOptions::ins)),
                insManualAttitude(field("param.ins_pitch", "pitch", &INSConfig::pitchDeg, &SolverOptions::ins)),
                insManualAttitude(field("param.ins_heading", "heading", &INSConfig::headingDeg,
                                        &SolverOptions::ins)),
            }},
            {"param.group.decode_scale", {
                insScaleField("param.decode_accel", "accel_lsb", &ImuScale::accelMpsPerLsb),
                insScaleField("param.decode_gyro", "gyro_lsb", &ImuScale::gyroRadPerLsb),
                insScaleField("param.decode_rate", "rate_hz", &ImuScale::rateHz),
            }},
            {"param.group.zupt", {
                field("param.zupt_accel_tol", "zupt_accel_tol", &INSConfig::zuptAccelTol,
                      &SolverOptions::ins),
                field("param.zupt_gyro_tol", "zupt_gyro_tol", &INSConfig::zuptGyroTol,
                      &SolverOptions::ins),
                field("param.zupt_hold", "zupt_hold", &INSConfig::zuptHoldSec,
                      &SolverOptions::ins),
            }},
        };
        return s;
    }

    const std::vector<SolverGroup> &schemaFor(const Solvers::Mode mode) {
        static const std::vector spp = {
            obsConditionGroup(Solvers::Mode::SPP), // SPP 也有滤波器，四档方向对文件解算同样生效
            obsGroup(false),
            sppMotionGroup(),
            tropoGroup(Solvers::Mode::SPP, false),
            isbGroup(),
            healthGroup(false),
        };
        static const std::vector pppStatic = {
            obsConditionGroup(Solvers::Mode::PPPStatic),
            obsGroup(true),
            corrGroup(),
            filterIterGroup(),
            arGroup(),
            tropoGroup(Solvers::Mode::PPPStatic, true, true),
            codeBiasGroup(),
            posGroup(false),
            slipGroup(false),
            isbGroup(),
            healthGroup(false),
        };
        static const std::vector pppKinematic = {
            obsConditionGroup(Solvers::Mode::PPPKinematic),
            obsGroup(true),
            corrGroup(),
            filterIterGroup(),
            arGroup(),
            tropoGroup(Solvers::Mode::PPPKinematic, true, true),
            codeBiasGroup(),
            posGroup(false),
            slipGroup(false),
            isbGroup(),
            healthGroup(false),
        };
        static const std::vector leo = {
            obsConditionGroup(Solvers::Mode::LEO),
            obsGroup(true),
            corrGroup(),
            filterIterGroup(),
            arGroup(),
            codeBiasGroup(),
            posGroup(true),
            slipGroup(false),
            isbGroup(),
            healthGroup(false),
        };
        static const std::vector<SolverGroup> rtk = {
            obsConditionGroup(Solvers::Mode::RTK),
            {
                "param.group.obs_model", {
                    rtkPhaseWeight("param.phase_weight_floor", "phaseWeightMinSinElev", &Weight::minSinElev,
                                   between(1e-6, 1.0)),
                    field("param.code_phase_sigma_ratio", "CodeOverPhaseSigma", &RTKConfig::codeOverPhaseSigma, &SolverOptions::rtk),
                    field("param.dd_min_sats", "minSatsForUpdate", &RTKConfig::minSats, &SolverOptions::rtk),
                    rtkPhaseWeight("param.phase_weight_a", "phaseWeightA", &Weight::a),
                    rtkPhaseWeight("param.phase_weight_b", "phaseWeightB", &Weight::b),
                }
            },
            {
                "param.group.ambiguity", {
                    flag("param.use_ar", "useAR", &RTKConfig::useAR, &SolverOptions::rtk),
                    flag("param.full_covariance", "fullCovariance", &RTKConfig::fullCovariance, &SolverOptions::rtk),
                    field("param.ratio_threshold", "arRatioThreshold", &RTKConfig::arThreshold, &SolverOptions::rtk),
                }
            },
            {
                "param.group.filter_float", {
                    field("param.pos_proc_sigma_m", "PosProcSigma", &SolverOptions::posProcSigma),
                    field("param.amb_proc_sigma_cycle", "AmbProcSigma", &RTKConfig::ambProcSigma, &SolverOptions::rtk),
                    field("param.amb_init_sigma_cycle", "AmbInitSigma", &RTKConfig::ambInitSigma, &SolverOptions::rtk),
                }
            },
            slipGroup(true),
            healthGroup(true),
            {
                "param.group.ref_sat", {
                    flag("param.exclude_geo_ref", "excludeGeoRef", &RTKConfig::excludeGeoRef, &SolverOptions::rtk),
                }
            },
        };

        static const std::vector<SolverGroup> empty;
        switch (mode) {
            case Solvers::Mode::SPP: return spp;
            case Solvers::Mode::PPPStatic: return pppStatic;
            case Solvers::Mode::PPPKinematic: return pppKinematic;
            case Solvers::Mode::LEO: return leo;
            case Solvers::Mode::RTK: return rtk;
            case Solvers::Mode::INS: return insSchema();
            default: return empty;
        }
    }
}

void Solvers::Helpers::renderAdvancedParams(SolverOptions &opts, const Mode mode) {
    UI::ParamPage::render(schemaFor(mode), opts, defaultSolverOptions(mode));
}

void Solvers::Helpers::saveSolverParams(const Mode mode, const SolverOptions &opts) {
    UI::ParamPage::save(schemaFor(mode), opts, std::string(modeKey(mode)) + "_");
}

void Solvers::Helpers::loadSolverParams(const Mode mode, SolverOptions &opts) {
    UI::ParamPage::load(schemaFor(mode), opts, defaultSolverOptions(mode),
                        std::string(modeKey(mode)) + "_");
}

void Solvers::Helpers::renderInsInitialState(SolverOptions &opts) {
    UI::ParamPage::labelWidth = (ImGui::CalcTextSize("param.ins_init_vel").x
                                 + ImGui::GetStyle().ItemSpacing.x * 2.0f);

    // 初始位置 BLH（与验证工具、GNSS 各模式同款，不再用 ECEF XYZ）
    {
        ImGui::TextUnformatted(I18n::trc("param.ins_init_pos_short"));
        double *slot[3] = {&opts.ins.latDeg, &opts.ins.lonDeg, &opts.ins.hM};
        const char *lab[3] = {"φ (°N)", "λ (°E)", "h (m)"};
        ImGui::PushItemWidth(140);
        for (int i = 0; i < 3; ++i) {
            if (i) ImGui::SameLine();
            const std::string id = std::string(lab[i]) + "##ins_init_pos_" + std::to_string(i);
            ImGui::InputDouble(id.c_str(), slot[i], 0.0, 0.0, "%.10g");
        }
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (const PasteResult p = clipboardXYZButton("ins_init_pos"); p.count > 0)
            for (int i = 0; i < p.count; ++i) *slot[i] = p.xyz[i];
    }

    // 初始速度 vNED
    {
        ImGui::TextUnformatted(I18n::trc("param.ins_init_vel_short"));
        double *slot[3] = {&opts.ins.vN, &opts.ins.vE, &opts.ins.vD};
        const char *lab[3] = {"vN", "vE", "vD"};
        ImGui::PushItemWidth(140);
        for (int i = 0; i < 3; ++i) {
            if (i) ImGui::SameLine();
            const std::string id = std::string(lab[i]) + "##ins_init_vel_" + std::to_string(i);
            ImGui::InputDouble(id.c_str(), slot[i], 0.0, 0.0, "%.10g");
        }
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (const PasteResult p = clipboardXYZButton("ins_init_vel"); p.count > 0)
            for (int i = 0; i < p.count; ++i) *slot[i] = p.xyz[i];
    }

    // 是否指定初始姿态：勾选→手填三姿态角；不勾→从开头静止段自动粗对准（默认）
    ImGui::Checkbox(I18n::trc("param.ins_specify_attitude"), &opts.ins.specifyAttitude);
    if (opts.ins.specifyAttitude) {
        double *slot[3] = {&opts.ins.rollDeg, &opts.ins.pitchDeg, &opts.ins.headingDeg};
        const char *lab[3] = {"param.att_roll", "param.att_pitch", "param.att_heading"};
        ImGui::PushItemWidth(140);
        for (int i = 0; i < 3; ++i) {
            if (i) ImGui::SameLine();
            const std::string id = std::string(lab[i]) + "##ins_init_att_" + std::to_string(i);
            ImGui::InputDouble(id.c_str(), slot[i], 0.0, 0.0, "%.10g");
        }
        ImGui::PopItemWidth();
    }
}
