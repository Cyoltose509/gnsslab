#include "GuiHelpers.h"
#include "GuiParamPage.h"

#include "core/PathMemory.h"
#include "io/Products.h"
#include "core/atmo/Troposphere.h"
#include "solvers/common/Config.h"
#include "ui/ParamPage.h"
#include "download/local/Store.h"
#include "download/remote/Catalog.h"
#include "download/ui/Widgets.h"

#include <array>
#include <string>
#include <vector>

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

    const char *filterItem(void *, const int idx) { return Solvers::kFilterChoices[idx].label; }

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
            "观测模型", {
                field("码 σ (m)", "CodeSigma", &SPPConfig::codeSigma, &SolverOptions::spp),
            }
        };
        if (withPhaseSigma)
            g.fields.push_back(field("相位 σ (m)", "PhaseSigma", &SPPConfig::phaseSigma, &SolverOptions::spp));
        g.fields.push_back(field("高度角加权下限 (sinE，0=跟随截止角)", "minSinElev", &SPPConfig::minSinElev,
                                 &SolverOptions::spp));
        return g;
    }

    SolverField systemsField() {
        SolverField f;
        f.label = "参与星座";
        f.key = "systems";
        f.draw = [](SolverOptions &o) {
            UI::ParamPage::beginField("参与星座", false);
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

    /// 解算设置：滤波方向只对有跨历元状态的模式出现，档数按模式给（RTK 两档，其余四档）。
    SolverGroup obsConditionGroup(const Solvers::Mode mode) {
        SolverGroup g{
            "解算设置", {
                field("采样间隔 (s，0=按文件)", "interval", &SolverOptions::sampleInterval,
                      between(0.0, 3600.0)),
                systemsField(),
                field("截止高度角 (°)", "cutoff", &SolverOptions::cutoffDeg, between(0.0, 90.0)),
            }
        };
        // RTK 的滤波档跟 σ 一起放在「滤波（浮点解）」组，其余模式的滤波方向归到解算设置
        if (Solvers::modeHasFilter(mode) && mode != Solvers::Mode::RTK)
            g.fields.push_back(comboFilter("滤波方向 (仅文件生效)"));
        return g;
    }

    void renderAutoProduct(const char *productId, const bool enabled,
                           const char *offText = "已关闭") {
        constexpr ImVec4 kGreen(0.35f, 0.85f, 0.45f, 1.0f);
        constexpr ImVec4 kAmber(0.95f, 0.75f, 0.25f, 1.0f);

        if (!enabled) {
            ImGui::TextDisabled("%s", offText);
            return;
        }

        if (const auto st = Download::Catalog::updateStatus(productId); st.phase == Download::Catalog::UpdateStatus::Downloading) {
            Download::Widgets::renderProgress("正在下载", st.file, st.progress, st.gotBytes, st.totalBytes);
            return;
        }

        if (const auto inst = Download::Store::active(productId)) {
            ImGui::TextColored(kGreen, "自动读取 %s", inst->file.c_str());
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("%s\n版本 %s，%s 更新", inst->path.c_str(),
                                  inst->version.c_str(), inst->fetchedAt.c_str());
            return;
        }

        const auto *meta = Download::Catalog::find(productId);
        ImGui::TextColored(kAmber, "未检测到 %s", meta ? meta->label : productId);
        ImGui::SameLine();
        if (ImGui::SmallButton(("获取##get_" + std::string(productId)).c_str()))
            Download::Catalog::checkNow(productId);
    }

    SolverGroup corrGroup() {
        return SolverGroup{
            "改正模型", {
                flag("天线改正", "applyAtx", &SPPConfig::applyAtx, &SolverOptions::spp),
                flag("相位缠绕", "applyWindup", &SPPConfig::applyWindup, &SolverOptions::spp),
                flag("固体潮", "applySolidTide", &SPPConfig::applySolidTide, &SolverOptions::spp),
            }
        };
    }

    SolverGroup tropoGroup(const Solvers::Mode mode, const bool withGrad, const bool withZtd = false) {
        SolverGroup g{"对流层", {}};
        if (withGrad)
            g.fields.push_back(flag("对流层梯度", "useTroGrad", &SPPConfig::useTroGrad, &SolverOptions::spp));
        g.fields.push_back(comboTropoMap("映射函数", "tropoMap", &SolverOptions::spp, &SPPConfig::tropoMap));
        if (withZtd) {
            g.title = "对流层 / ZTD";
            g.fields.push_back(field("ZTD 初值 (m)", "ztdInitValue", &SPPConfig::ztdInitValue, &SolverOptions::spp));
            g.fields.push_back(field("ZTD 初始 σ", "ZtdInitSigma", &SPPConfig::ztdInitSigma, &SolverOptions::spp));
            g.fields.push_back(field("ZTD 过程 σ", "ZtdProcSigma", &SPPConfig::ztdProcSigma, &SolverOptions::spp));
            g.fields.push_back(field("ZTD 先验 σ", "ztdPriorSigma", &SPPConfig::ztdPriorSigma,
                                     &SolverOptions::spp));
            g.fields.push_back(field("ZTD 约束 σ", "troZtdSigma", &SPPConfig::troZtdSigma, &SolverOptions::spp));
        }
        g.row = [mode](const SolverOptions &o) {
            constexpr ImVec4 kGreen(0.35f, 0.85f, 0.45f, 1.0f);
            constexpr ImVec4 kAmber(0.95f, 0.75f, 0.25f, 1.0f);
            if (o.spp.tropoMap == Troposphere::Map::VMF3) {
                const auto &prod = AppConfig::Products::of(Solvers::modeKey(mode));
                if (const auto &vmf = prod.vmf; vmf.empty()) {
                    ImGui::TextColored(kAmber, "已选 VMF3 但缺 VMF3 格网文件，将回落 GMF");
                    return;
                }
                const int n = static_cast<int>(prod.VMF3Paths().size());
                ImGui::TextColored(kGreen, "已加载 VMF3 格网（%d 个文件）", n);
                if (n < 4)
                    ImGui::TextColored(kAmber, "仅 %d/4 时次：时间向线性插值退化为单历元", n);
                renderAutoProduct(Download::Catalog::kGPT3ID, true, "已关闭：只出 ah/aw，不做原生 zhd/zwd 抬升");
                return;
            }
            if (o.spp.tropoMap == Troposphere::Map::GPT3) {
                renderAutoProduct(Download::Catalog::kGPT3ID, true, "已关闭：不使用 GPT3 格网");
                return;
            }
            ImGui::TextDisabled("Hopfield/NMF/GMF：无需外部格网");
        };
        g.rowAfterKey = "tropoMap";
        return g;
    }

    SolverGroup filterIterGroup() {
        return {
            "滤波更新", {
                field("滤波更新最少卫星数 (0=不限)", "minSatsForUpdate", &SPPConfig::minSatsForUpdate,
                      &SolverOptions::spp),
                field("钟差过程 σ", "ClockProcSigma", &SPPConfig::clockProcSigma, &SolverOptions::spp),
            }
        };
    }

    SolverGroup arGroup() {
        return {
            "模糊度与 AR", {
                flag("使用 AR 固定", "useAR", &SPPConfig::useAR, &SolverOptions::spp),
                field("模糊度初始 σ", "AmbInitSigma", &SPPConfig::ambInitSigma, &SolverOptions::spp),
                field("模糊度过程 σ", "AmbProcSigma", &SPPConfig::ambProcSigma, &SolverOptions::spp),
                field("Ratio 阈值", "arRatioThreshold", &SPPConfig::arRatioThreshold, &SolverOptions::spp),
            }
        };
    }

    SolverGroup codeBiasGroup() {
        return {
            "码偏差 / GLONASS IFCB", {
                field("码偏过程 σ", "CodeBiasProcSigma", &SPPConfig::codeBiasProcSigma, &SolverOptions::spp),
                field("码偏初始 σ", "CodeBiasInitSigma", &SPPConfig::codeBiasInitSigma, &SolverOptions::spp),
                field("GLO IFCB 过程 σ", "GloIfcbProcSigma", &SPPConfig::gloIfcbProcSigma, &SolverOptions::spp),
                field("GLO IFCB 初始 σ", "GloIfcbInitSigma", &SPPConfig::gloIfcbInitSigma, &SolverOptions::spp),
            }
        };
    }

    /// 位置过程 σ：PPP 静态为 0（位置固定），PPP 动态与 LEO 为 100（逐历元更新）。
    /// 每个模式各存各的值，前缀不同，互不影响。
    SolverGroup posGroup(const bool leo) {
        SolverGroup g{
            "位置 / σ", {
                field("位置过程 σ", "PosProcSigma", &SolverOptions::posProcSigma),
            }
        };
        g.fields.push_back(field("首历元位置 σ", "FirstEpochPosSigma", &SPPConfig::firstEpochPosSigma,
                                 &SolverOptions::spp));
        if (leo) {
            g.fields.push_back(field("运动检测阈值", "motionThresh", &SolverOptions::motionThresh));
            g.fields.push_back(field("SPP 预测阈值", "sppVsPredThresh", &SolverOptions::sppVsPredThresh));
        }
        return g;
    }

    /// SPP 的位置/速度状态模型：加速度 σ>0 时状态含速度三态，位置按常速度预测。
    SolverGroup sppMotionGroup() {
        return {
            "位置 / 速度", {
                field("位置过程 σ (m，仅加速度=0 用)", "PosProcSigma", &SolverOptions::posProcSigma),
                field("加速度过程 σ (m/s)", "AccProcSigma", &SolverOptions::accProcSigma),
                field("速度初始 σ (m/s)", "VelInitSigma", &SolverOptions::velInitSigma),
                field("钟差过程 σ (m)", "ClockProcSigma", &SPPConfig::clockProcSigma, &SolverOptions::spp),
            }
        };
    }

    SolverGroup slipGroup() {
        return {
            "周跳探测", {
                flag("使用周跳探测", "useCycleSlip", &SPPConfig::useCycleSlip, &SolverOptions::spp),
                field("GF 周跳阈值倍数", "gfSlipMult", &SPPConfig::gfSlipMult, &SolverOptions::spp),
            }
        };
    }

    SolverGroup healthGroup() {
        return {
            "卫星可用性", {
                flag("剔除电文标记不健康的卫星", "useHealth", &SPPConfig::useHealth, &SolverOptions::spp),
            }
        };
    }

    SolverGroup isbGroup() {
        return {
            "系统间偏差 (ISB)", {
                field("ISB 过程 σ", "IsbProcSigma", &SPPConfig::isbProcSigma, &SolverOptions::spp),
                field("ISB 初始 σ", "IsbInitSigma", &SPPConfig::isbInitSigma, &SolverOptions::spp),
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
        SolverField f = insVec3Field("初始位置 BLH (°/°/m)", {"lat", "lon", "h"},
                                     {"φ", "λ", "h"}, {&INSConfig::latDeg, &INSConfig::lonDeg, &INSConfig::hM});
        f.show = [](const SolverOptions &) { return false; };   // 只在文件页渲染
        return f;
    }

    SolverField insInitialVelField() {
        SolverField f = insVec3Field("初始速度 vNED (m/s)", {"vn", "ve", "vd"},
                                     {"vN", "vE", "vD"}, {&INSConfig::vN, &INSConfig::vE, &INSConfig::vD});
        f.show = [](const SolverOptions &) { return false; };
        return f;
    }

    const std::vector<SolverGroup> &insSchema() {
        static const std::vector<SolverGroup> s = {
            {"初始状态（BLH / vNED）", {
                insInitialPosField(),
                insInitialVelField(),
            }},
            {"初始姿态", {
                flag("指定初始姿态角（不勾则自动粗对准）", "specify_att", &INSConfig::specifyAttitude,
                     &SolverOptions::ins),
                insManualAttitude(field("手填横滚 (°)", "roll", &INSConfig::rollDeg, &SolverOptions::ins)),
                insManualAttitude(field("手填俯仰 (°)", "pitch", &INSConfig::pitchDeg, &SolverOptions::ins)),
                insManualAttitude(field("手填航向 (°)", "heading", &INSConfig::headingDeg,
                                        &SolverOptions::ins)),
            }},
            {"解码当量（0 = 自动：型号自描述 / 开头静置段反推）", {
                insScaleField("加计 (m/s)/LSB", "accel_lsb", &ImuScale::accelMpsPerLsb),
                insScaleField("陀螺 rad/LSB", "gyro_lsb", &ImuScale::gyroRadPerLsb),
                insScaleField("采样率 (Hz)", "rate_hz", &ImuScale::rateHz),
            }},
            {"零速修正（默认解算）", {
                field("比力判静止容差 (m/s²)", "zupt_accel_tol", &INSConfig::zuptAccelTol,
                      &SolverOptions::ins),
                field("角速度容差 (rad/s)", "zupt_gyro_tol", &INSConfig::zuptGyroTol,
                      &SolverOptions::ins),
                field("连续静止时长 (s)", "zupt_hold", &INSConfig::zuptHoldSec,
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
            healthGroup(),
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
            slipGroup(),
            isbGroup(),
            healthGroup(),
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
            slipGroup(),
            isbGroup(),
            healthGroup(),
        };
        static const std::vector leo = {
            obsConditionGroup(Solvers::Mode::LEO),
            obsGroup(true),
            corrGroup(),
            filterIterGroup(),
            arGroup(),
            codeBiasGroup(),
            posGroup(true),
            slipGroup(),
            isbGroup(),
            healthGroup(),
        };
        static const std::vector<SolverGroup> rtk = {
            obsConditionGroup(Solvers::Mode::RTK),
            {
                "模糊度与 AR", {
                flag("使用 AR 固定", "useAR", &RTKConfig::useAR, &SolverOptions::rtk),
                flag("完整协方差", "fullCovariance", &RTKConfig::fullCovariance, &SolverOptions::rtk),
                field("Ratio 阈值", "arThreshold", &RTKConfig::arThreshold, &SolverOptions::rtk),
                }
            },
            {
                "滤波（浮点解）", {
                    comboFilter("滤波方向"),
                    field("位置过程 σ (m)", "PosProcSigma", &SolverOptions::posProcSigma),
                    field("模糊度过程 σ (周)", "AmbProcSigma", &RTKConfig::ambProcSigma, &SolverOptions::rtk),
                    field("模糊度初始 σ (周)", "AmbInitSigma", &RTKConfig::ambInitSigma, &SolverOptions::rtk),
                }
            },
            {
                "周跳探测", {
                    flag("使用周跳探测", "useCycleSlip", &RTKConfig::useCycleSlip, &SolverOptions::rtk),
                    flag("使用 GF 周跳", "useGfCycleSlip", &RTKConfig::useGfCycleSlip, &SolverOptions::rtk),
                    field("GF 周跳阈值倍数", "gfSlipMult", &RTKConfig::gfSlipMult, &SolverOptions::rtk),
                }
            },
            {
                "观测模型", {
                    rtkPhaseWeight("相位高度角加权下限 (sinE)", "phaseWeightMinSinElev", &Weight::minSinElev,
                                   between(1e-6, 1.0)),
                    field("伪距:相位 σ 比", "CodeOverPhaseSigma", &RTKConfig::codeOverPhaseSigma, &SolverOptions::rtk),
                    field("双差最少卫星数", "minSats", &RTKConfig::minSats, &SolverOptions::rtk),
                    rtkPhaseWeight("相位高度角模型 a", "phaseWeightA", &Weight::a),
                    rtkPhaseWeight("相位高度角模型 b", "phaseWeightB", &Weight::b),
                }
            },
            {
                "参考星选择", {
                    flag("排除北斗 GEO 参考星", "excludeGeoRef", &RTKConfig::excludeGeoRef, &SolverOptions::rtk),
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
    UI::ParamPage::labelWidth = (ImGui::CalcTextSize("初始速度 vNED (m/s)").x
                                 + ImGui::GetStyle().ItemSpacing.x * 2.0f);

    // 初始位置 BLH（与验证工具、GNSS 各模式同款，不再用 ECEF XYZ）
    {
        ImGui::Text("初始位置 (BLH)");
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
        ImGui::Text("初始速度 (vNED)");
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
    ImGui::Checkbox("指定初始姿态角（不勾则自动粗对准）", &opts.ins.specifyAttitude);
    if (opts.ins.specifyAttitude) {
        double *slot[3] = {&opts.ins.rollDeg, &opts.ins.pitchDeg, &opts.ins.headingDeg};
        const char *lab[3] = {"横滚 roll (°)", "俯仰 pitch (°)", "航向 heading (°)"};
        ImGui::PushItemWidth(140);
        for (int i = 0; i < 3; ++i) {
            if (i) ImGui::SameLine();
            const std::string id = std::string(lab[i]) + "##ins_init_att_" + std::to_string(i);
            ImGui::InputDouble(id.c_str(), slot[i], 0.0, 0.0, "%.10g");
        }
        ImGui::PopItemWidth();
    }
}
