#include "GuiHelpers.h"
#include "GuiParamPage.h"

#include "core/PathMemory.h"
#include "core/Products.h"
#include "Troposphere.h"
#include "ui/ParamPage.h"
#include "download/ProductStore.h"
#include "download/ProductCatalog.h"
#include "download/AutoSync.h"

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

namespace {
    std::string spPrefix(const Solvers::Mode m) {
        switch (m) {
            case Solvers::Mode::SPP: return "spp_";
            case Solvers::Mode::PPP: return "ppp_";
            case Solvers::Mode::LEO: return "leo_";
            case Solvers::Mode::RTK: return "rtk_";
            default: return "solver_";
        }
    }

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
            if (ImGui::Combo(label, &cur, tropoMapItem, nullptr, Troposphere::kModelCount))
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

    SolverGroup obsGroup(const bool withPhaseSigma) {
        SolverGroup g{
            "观测模型", {
                field("码噪声 sigma (m)", "sigIFCode", &SPPConfig::sigIFCode, &SolverOptions::spp),
            }
        };
        if (withPhaseSigma)
            g.fields.push_back(field("相位噪声 sigma (m)", "sigmaPhase", &SPPConfig::sigmaPhase, &SolverOptions::spp));
        g.fields.push_back(field("sin(el) 下限 (0=跟随截止角)", "minSinElev", &SPPConfig::minSinElev,
                                 &SolverOptions::spp));
        return g;
    }

    SolverGroup corrGroup() {
        return {
            "改正模型", {
                flag("相位缠绕 (windup)", "applyWindup", &SPPConfig::applyWindup, &SolverOptions::spp),
                flag("固体潮改正", "applySolidTide", &SPPConfig::applySolidTide, &SolverOptions::spp),
            }
        };
    }

    void renderAutoProduct(const char *productId, const bool enabled,
                           const char *offText = "已关闭") {
        constexpr ImVec4 kGreen(0.35f, 0.85f, 0.45f, 1.0f);
        constexpr ImVec4 kAmber(0.95f, 0.75f, 0.25f, 1.0f);

        if (!enabled) {
            ImGui::TextDisabled("%s", offText);
            return;
        }

        if (const auto st = Download::AutoSync::status(productId); st.phase == Download::AutoSync::Status::Downloading) {
            if (st.totalBytes > 0.0)
                ImGui::TextColored(kAmber, "正在下载 %s %.0f%% (%.1f/%.1f MB)", st.file.c_str(),
                                   st.progress * 100.0f, st.gotBytes / 1048576.0,
                                   st.totalBytes / 1048576.0);
            else
                ImGui::TextColored(kAmber, "正在获取 %s ...", st.file.c_str());
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
            Download::AutoSync::checkNow(productId);
    }

    SolverGroup atxGroup() {
        SolverGroup g{
            "天线 / 产品", {
                flag("天线改正 (ATX)", "applyAtx", &SPPConfig::applyAtx, &SolverOptions::spp),
            }
        };
        g.row = [](const SolverOptions &o) {
            renderAutoProduct("atx", o.spp.applyAtx, "已关闭：不使用天线相位中心改正");
        };
        return g;
    }

    SolverGroup tropoGroup(const Solvers::Mode mode, const bool withGrad, const bool withZtd = false) {
        SolverGroup g{"对流层", {}};
        if (withGrad)
            g.fields.push_back(flag("对流层梯度", "useTroGrad", &SPPConfig::useTroGrad, &SolverOptions::spp));
        g.fields.push_back(comboTropoMap("映射函数", "tropoMap", &SolverOptions::spp, &SPPConfig::tropoMap));
        if (withZtd) {
            g.title = "对流层 / ZTD";
            g.fields.push_back(field("ZTD 初值 (m)", "ztdInitValue", &SPPConfig::ztdInitValue, &SolverOptions::spp));
            g.fields.push_back(field("ZTD 初始方差", "ztdInitVar", &SPPConfig::ztdInitVar, &SolverOptions::spp));
            g.fields.push_back(field("ZTD 过程噪声", "ztdProcNoise", &SPPConfig::ztdProcNoise, &SolverOptions::spp));
            g.fields.push_back(field("ZTD 先验 sigma (m)", "ztdPriorSigma", &SPPConfig::ztdPriorSigma,
                                     &SolverOptions::spp));
            g.fields.push_back(field("ZTD sigma0 (m)", "troZtdSigma", &SPPConfig::troZtdSigma, &SolverOptions::spp));
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
                renderAutoProduct("gpt3", true, "已关闭：只出 ah/aw，不做原生 zhd/zwd 抬升");
                return;
            }
            if (o.spp.tropoMap == Troposphere::Map::GPT3) {
                renderAutoProduct("gpt3", true, "已关闭：不使用 GPT3 格网");
                return;
            }
            ImGui::TextDisabled("Hopfield/NMF/GMF：无需外部格网");
        };
        return g;
    }

    SolverGroup lsqIterGroup() {
        return {
            "迭代 / 收敛", {
                field("收敛阈值 (m)", "convEps", &SPPConfig::convEps, &SolverOptions::spp),
                field("最大迭代次数", "maxIter", &SPPConfig::maxIter, &SolverOptions::spp),
                field("起始收敛检查迭代", "minCheckIter", &SPPConfig::minCheckIter, &SolverOptions::spp),
            }
        };
    }

    SolverGroup filterIterGroup(const bool withSppResidualThr) {
        SolverGroup g{
            "迭代 / 收敛", {
                field("收敛阈值 (m)", "convEps", &SPPConfig::convEps, &SolverOptions::spp),
                field("滤波更新最少卫星数 (0=不限)", "minSatsForUpdate", &SPPConfig::minSatsForUpdate,
                      &SolverOptions::spp),
                field("钟差过程噪声", "clockNoise", &SPPConfig::clockNoise, &SolverOptions::spp),
                field("重锚定最大迭代", "maxReanchorIter", &SPPConfig::maxReanchorIter, &SolverOptions::spp),
            }
        };
        if (withSppResidualThr)
            g.fields.push_back(field("SPP 残差 RMS 阈值 (m)", "sppResidualThr", &SPPConfig::sppResidualThr,
                                     &SolverOptions::spp));
        return g;
    }

    SolverGroup arGroup() {
        return {
            "模糊度与 AR", {
                flag("使用 AR 固定", "useAR", &SPPConfig::useAR, &SolverOptions::spp),
                field("模糊度初始方差", "ambInitVar", &SPPConfig::ambInitVar, &SolverOptions::spp),
                field("模糊度过程噪声", "ambNoise", &SPPConfig::ambNoise, &SolverOptions::spp),
                field("最大保持模糊度间隙 (s)", "keepAmbMaxGap", &SPPConfig::keepAmbMaxGap, &SolverOptions::spp),
                field("LAMBDA 成功率阈值", "arRatioThreshold", &SPPConfig::arRatioThreshold, &SolverOptions::spp),
                field("进 LAMBDA 的模糊度 std 上限 (m)", "arSigGate", &SPPConfig::arSigGate, &SolverOptions::spp),
            }
        };
    }

    SolverGroup codeBiasGroup() {
        return {
            "码偏差 / GLONASS IFCB", {
                field("码偏过程噪声", "codeBiasProcNoise", &SPPConfig::codeBiasProcNoise, &SolverOptions::spp),
                field("码偏初始方差", "codeBiasInitVar", &SPPConfig::codeBiasInitVar, &SolverOptions::spp),
                field("GLO IFCB 过程噪声", "gloIfcbProcNoise", &SPPConfig::gloIfcbProcNoise, &SolverOptions::spp),
                field("GLO IFCB 初始方差", "gloIfcbInitVar", &SPPConfig::gloIfcbInitVar, &SolverOptions::spp),
            }
        };
    }

    SolverGroup posGroup(const bool leo) {
        SolverGroup g{
            "位置 / 噪声", {
                field(leo ? "位置过程噪声" : "动态位置过程噪声", "posNoise", &SolverOptions::posNoise),
            }
        };
        if (!leo) g.fields.push_back(field("静态位置过程噪声", "staticPosNoise", &SolverOptions::staticPosNoise));
        g.fields.push_back(field("首历元位置方差", "firstEpochPosVar", &SPPConfig::firstEpochPosVar,
                                 &SolverOptions::spp));
        g.fields.push_back(field("位置方差下界", "posVarFloor", &SPPConfig::posVarFloor, &SolverOptions::spp));
        if (leo) {
            g.fields.push_back(field("运动检测阈值", "motionThresh", &SolverOptions::motionThresh));
            g.fields.push_back(field("SPP 预测阈值", "sppVsPredThresh", &SolverOptions::sppVsPredThresh));
        }
        return g;
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
                field("ISB 过程噪声", "isbProcNoise", &SPPConfig::isbProcNoise, &SolverOptions::spp),
                field("ISB 初始方差", "isbInitVar", &SPPConfig::isbInitVar, &SolverOptions::spp),
            }
        };
    }

    const std::vector<SolverGroup> &schemaFor(const Solvers::Mode mode) {
        static const std::vector spp = {
            obsGroup(false),
            tropoGroup(Solvers::Mode::SPP, false),
            lsqIterGroup(),
            isbGroup(),
            healthGroup(),
        };
        static const std::vector ppp = {
            obsGroup(true),
            corrGroup(),
            atxGroup(),
            filterIterGroup(false),
            arGroup(),
            tropoGroup(Solvers::Mode::PPP, true, true),
            codeBiasGroup(),
            posGroup(false),
            slipGroup(),
            isbGroup(),
            healthGroup(),
        };
        static const std::vector leo = {
            obsGroup(true),
            corrGroup(),
            atxGroup(),
            filterIterGroup(true),
            arGroup(),
            codeBiasGroup(),
            posGroup(true),
            slipGroup(),
            isbGroup(),
            healthGroup(),
        };
        static const std::vector<SolverGroup> rtk = {
            {
                "模糊度与 AR", {
                    flag("使用 AR 固定", "useAR", &RTKConfig::useAR, &SolverOptions::rtk),
                    flag("完整协方差", "fullCovariance", &RTKConfig::fullCovariance, &SolverOptions::rtk),
                    field("LAMBDA 成功率阈值", "arThreshold", &RTKConfig::arThreshold, &SolverOptions::rtk),
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
                "迭代 / 收敛", {
                    field("收敛阈值 (m)", "convEps", &RTKConfig::convEps, &SolverOptions::rtk),
                    field("浮点解最大迭代", "maxIterFloat", &RTKConfig::maxIterFloat, &SolverOptions::rtk),
                    field("固定解最大迭代", "maxIterFix", &RTKConfig::maxIterFix, &SolverOptions::rtk),
                    field("双差最少卫星数", "minSats", &RTKConfig::minSats, &SolverOptions::rtk),
                }
            },
            {
                "观测模型", {
                    rtkPhaseWeight("sin(el) 下限", "phaseWeightMinSinElev", &Weight::minSinElev,
                                   between(1e-6, 1.0)),
                    field("伪距:相位 方差比", "codeOverPhaseVar", &RTKConfig::codeOverPhaseVar, &SolverOptions::rtk),
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
            case Solvers::Mode::PPP: return ppp;
            case Solvers::Mode::LEO: return leo;
            case Solvers::Mode::RTK: return rtk;
            default: return empty;
        }
    }
}

void Solvers::Helpers::renderAdvancedParams(SolverOptions &opts, const Mode mode) {
    UI::ParamPage::render(schemaFor(mode), opts, defaultSolverOptions(mode));
}

void Solvers::Helpers::saveSolverParams(const Mode mode, const SolverOptions &opts) {
    UI::ParamPage::save(schemaFor(mode), opts, spPrefix(mode));
}

void Solvers::Helpers::loadSolverParams(const Mode mode, SolverOptions &opts) {
    UI::ParamPage::load(schemaFor(mode), opts, defaultSolverOptions(mode), spPrefix(mode));
}
