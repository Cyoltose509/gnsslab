#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include "GuiHelpers.h"
#include "SolverTask.h"
#include "core/PathMemory.h"
#include "ui/ParamPage.h"
#include "Log.h"

#include "download/ProductMeta.h"
#include "download/DownloadStore.h"
#include "GuiFtpDownload.h"

#include "StringUtils.h"
#include <algorithm>
#include <functional>
#include <map>
#include <string>
#include <type_traits>
#include <vector>


void Solvers::Helpers::renderRealtimeConfig(GnssTask::SolverTaskBase *task,
                                            std::vector<GnssTask::RealtimeEndpoint> *endpoints,
                                            const std::vector<std::string> &labels,
                                            double *cutoffDeg, std::set<char> *systems,
                                            double *sampleInterval,
                                            const std::function<void()> &onStart,
                                            const std::function<void()> &extraContent) {
    const size_t n = endpoints ? endpoints->size() : 0;
    ImGui::Text("连接到 Oem7 实时流 (socket)：");
    ImGui::PushItemWidth(200);
    for (size_t i = 0; i < n && i < labels.size(); ++i) {
        if (!labels[i].empty()) ImGui::Text("%s：", labels[i].c_str());
        inputTextStd(("IP 地址##rt_ep_" + std::to_string(i)).c_str(), &(*endpoints)[i].ip);
        inputTextStd(("端口##rt_ep_" + std::to_string(i)).c_str(), &(*endpoints)[i].port);
    }
    ImGui::PopItemWidth();
    ImGui::Separator();

    renderCutoffAndSystems(cutoffDeg, systems, sampleInterval);

    if (extraContent) {
        ImGui::Separator();
        extraContent();
    }
    ImGui::Separator();
    if (ImGui::Button("开始解算", ImVec2(200, 40))) {
        if (onStart) onStart();
    }
    ImGui::SameLine();
    renderCancelButton(task);
}

Solvers::Helpers::RobustResRange Solvers::Helpers::computeRobustResRange(const std::map<SatID, std::vector<double> > &satRes,
                                                                         const double minHalf) {
    std::vector<double> all_v;
    for (const auto &[_, vs]: satRes)
        for (double v: vs) all_v.push_back(v);
    RobustResRange r;
    if (all_v.size() < 2) return r;
    std::sort(all_v.begin(), all_v.end());
    const auto n = static_cast<long>(all_v.size());
    const double lo = all_v[static_cast<size_t>(0.01 * n)];
    const double hi = all_v[static_cast<size_t>(0.99 * n)];
    double half = 0.5 * (hi - lo);
    if (!(half > 0.0)) half = minHalf;
    half = std::max(minHalf, std::min(half, 20.0));
    r.lo = -half;
    r.hi = half;
    return r;
}

static int inputTextResizeCallback(ImGuiInputTextCallbackData *data) {
    if (data->EventFlag == ImGuiInputTextFlags_CallbackResize) {
        auto *str = static_cast<std::string *>(data->UserData);
        str->resize(data->BufSize - 1);
        data->Buf = const_cast<char *>(str->c_str());
    }
    return 0;
}

bool Solvers::Helpers::inputTextStd(const char *label, std::string *str, const ImGuiInputTextFlags flags) {
    if (str->capacity() < 512) str->reserve(512);
    const bool changed = ImGui::InputText(
        label, str->data(), str->capacity() + 1,
        flags | ImGuiInputTextFlags_CallbackResize, inputTextResizeCallback, str);
    if (const size_t len = std::strlen(str->data()); len != str->size()) str->resize(len);
    return changed;
}


void Solvers::Helpers::fileRow(const char *label, std::string *buf, const char *iniKey,
                               const GuiFileFilter *filters, const int nFilters,
                               const std::function<void(const std::string &)> &onSelect,
                               const char *obsIniKey, std::string *autoNavBuf) {
    static std::map<std::string, std::string> synced;
    ImGui::Text("%s:", label);
    ImGui::PushItemWidth(-200.0f);
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
    if (ImGui::Button(("浏览##" + std::string(label)).c_str())) {
        if (std::wstring wpath; ShowOpenFileDialog(wpath, filters, nFilters)) {
            const std::string p = wideToAcp(wpath);
            *buf = p;
            if (onSelect) onSelect(p);
            if (autoNavBuf) {
                if (const std::string d = GnssTask::AutoDetectNav(p); !d.empty()) *autoNavBuf = d;
            }
            AppConfig::PathMemory::instance().addRecent(iniKey, p);
            AppConfig::PathMemory::instance().set(iniKey, p);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button(("历史##" + std::string(label)).c_str()))
        ImGui::OpenPopup(("##hist_" + std::string(label)).c_str());
    if (ImGui::BeginPopup(("##hist_" + std::string(label)).c_str())) {
        const auto rec = AppConfig::PathMemory::instance().getRecent(iniKey);
        if (rec.empty()) ImGui::TextDisabled("(暂无)");
        for (auto &r: rec)
            if (ImGui::Selectable(r.c_str())) {
                *buf = r;
                if (onSelect) onSelect(r);
                if (autoNavBuf) {
                    if (const std::string d = GnssTask::AutoDetectNav(r); !d.empty()) *autoNavBuf = d;
                }
                AppConfig::PathMemory::instance().set(iniKey, r);
                AppConfig::PathMemory::instance().addRecent(iniKey, r);
            }
        ImGui::EndPopup();
    }

    if (const ProductType ptype = ProductMeta::typeFromIniKey(iniKey); ptype != ProductType::Unknown && ptype != ProductType::ATX) {

        const std::string obsPath = obsIniKey && *obsIniKey
                                        ? AppConfig::PathMemory::instance().get(obsIniKey)
                                        : "";
        const auto yd = ProductMeta::parseObsDate(obsPath);

        ImGui::SameLine();
        if (ImGui::Button(("获取##" + std::string(label)).c_str())) {
            if (!yd) {
                LOG_WARN << "[获取] 无法从观测文件名推断日期（" << label << "），请先填观测文件路径";
                ImGui::OpenPopup(("##getfail_" + std::string(label)).c_str());
            } else {
                if (const auto local = DownloadStore::findLocal(ptype, yd->first, yd->second); !local.empty()) {
                    const std::string p =
                            DownloadStore::dirFor(ptype, yd->first, yd->second) + local.front();
                    *buf = p;
                    if (onSelect) onSelect(p);
                    if (autoNavBuf) {
                        if (const std::string d = GnssTask::AutoDetectNav(p); !d.empty()) *autoNavBuf = d;
                    }
                    AppConfig::PathMemory::instance().set(iniKey, p);
                    AppConfig::PathMemory::instance().addRecent(iniKey, p);
                    LOG_INFO << "[获取] 本地命中 " << ProductMeta::typeDir(ptype) << " "
                            << ProductMeta::doyToDateStr(yd->first, yd->second) << " → " << p;
                } else {
                    LOG_INFO << "[获取] 本地未命中 " << ProductMeta::typeDir(ptype) << " "
                            << ProductMeta::doyToDateStr(yd->first, yd->second)
                            << "，弹出自动下载确认";
                    ImGui::OpenPopup(("##getdl_" + std::string(label)).c_str());
                }
            }
        }

        if (yd && ImGui::BeginPopupModal(("##getdl_" + std::string(label)).c_str(), nullptr,
                                         ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("本地 downloads/ 未找到 %s（COD / %d-%03d）。",
                        ProductMeta::typeDir(ptype).c_str(), yd->first, yd->second);
            ImGui::Text("是否打开下载面板自动获取？");
            if (ImGui::Button("是")) {
                Tools::FtpDownload::RequestAutoDownload(ptype, yd->first, yd->second, "COD");
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("否")) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
        if (!yd && ImGui::BeginPopupModal(("##getfail_" + std::string(label)).c_str(), nullptr,
                                          ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("无法从观测文件名推断日期，请先填写观测文件路径。");
            if (ImGui::Button("知道了")) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
    }
}

void Solvers::Helpers::renderCutoffAndSystems(double *cutoffDeg, std::set<char> *sysMask,
                                              double *sampleInterval,
                                              const std::function<void()> &extraSameLine) {
    ImGui::PushItemWidth(120);
    ImGui::InputDouble("截止高度角(°)", cutoffDeg);
    if (sampleInterval) {
        ImGui::SameLine();
        if (*sampleInterval < 0.0) *sampleInterval = 0.0;
        ImGui::InputDouble("采样间隔(s)", sampleInterval);
    }
    ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::Text("选择星座:");
    struct {
        char ch;
        const char *label;
    } cons[] = {{'G', "GPS"}, {'R', "GLO"}, {'E', "GAL"}, {'C', "BDS"}};
    for (auto &[ch, label]: cons) {
        bool on = sysMask->count(ch) > 0;
        ImGui::SameLine();
        ImGui::Checkbox(label, &on);
        if (on) sysMask->insert(ch);
        else sysMask->erase(ch);
    }
    if (extraSameLine) {
        ImGui::SameLine();
        extraSameLine();
    }
    ImGui::NewLine();
}


std::string Solvers::Helpers::baseName(const std::string &p) {
    const auto pos = p.find_last_of("\\/");
    return pos != std::string::npos ? p.substr(pos + 1) : p;
}


void Solvers::Helpers::beginConfigBody(const std::string &idBase, const GnssTask::SolverTaskBase &task) {
    const std::string id = idBase + std::to_string(task.cfgIndex);
    ImGui::BeginChild(id.c_str(), ImVec2(0.0f, 0.0f), ImGuiChildFlags_None);
}

void Solvers::Helpers::endConfigBody() {
    ImGui::EndChild();
}

bool Solvers::Helpers::saveFileDialog(const std::string &defaultName, std::wstring &outPath,
                                      const GuiFileFilter *filters, const int nFilters,
                                      const wchar_t *defExt) {
    const std::wstring wDef = acpToWide(defaultName);
    return ShowSaveFileDialog(outPath, filters, nFilters, wDef.c_str(), defExt);
}

bool Solvers::Helpers::saveCSVDialog(const std::string &defaultName, std::wstring &outPath) {
    return saveFileDialog(defaultName, outPath, fCsv, 2, L"csv");
}

template<>
struct Ui::ParamPage::Traits<TropoMap> {
    static void store(const std::string &k, const TropoMap v) {
        AppConfig::PathMemory::instance().set(k, Troposphere::tropoMapName(v));
    }

    static void fetch(const std::string &k, TropoMap &v, const Range &) {
        v = Troposphere::tropoMapFromName(
            AppConfig::PathMemory::instance().get(k, Troposphere::tropoMapName(v)));
    }
};

namespace {
    std::string spPrefix(const GnssTask::SolverMode m) {
        switch (m) {
            case GnssTask::SolverMode::Spp: return "spp_";
            case GnssTask::SolverMode::Ppp: return "ppp_";
            case GnssTask::SolverMode::Leo: return "leo_";
            case GnssTask::SolverMode::Rtk: return "rtk_";
            default: return "solver_";
        }
    }

    using SolverField = Ui::ParamPage::Field<SolverOptions>;
    using SolverGroup = Ui::ParamPage::Group<SolverOptions>;
    using Ui::ParamPage::between;
    using Ui::ParamPage::field;
    using Ui::ParamPage::flag;


    SolverField rtkPhaseWeight(const char *label, const char *key, double Weight::*m,
                               const Ui::ParamPage::Range r = Ui::ParamPage::unbounded) {
        return Ui::ParamPage::makeField<Ui::ParamPage::kValue, SolverOptions, double>(
            label, key, r,
            [m](SolverOptions &o) -> double & { return o.rtk.phaseWeight.*m; },
            [m](const SolverOptions &o) -> const double & { return o.rtk.phaseWeight.*m; });
    }

    const char *const sTropoMapItems[] = {"Hopfield", "NMF (Niell 1996)", "GMF (Boehm 2006)"};
    constexpr int kTropoMapCount = std::size(sTropoMapItems);


    SolverField combo(const char *label, const char *key, const char *const items[], int nItems,
                      SppConfig SolverOptions::*owner, TropoMap SppConfig::*m) {
        auto at = [m, owner](SolverOptions &o) -> TropoMap & { return o.*owner.*m; };
        auto atc = [m, owner](const SolverOptions &o) -> const TropoMap & { return o.*owner.*m; };
        SolverField f;
        f.label = label;
        f.key = key;
        f.draw = [label, items, nItems, at](SolverOptions &o) {
            int cur = static_cast<int>(at(o));
            if (ImGui::Combo(label, &cur, items, nItems)) at(o) = static_cast<TropoMap>(cur);
        };
        f.store = [key, atc](const SolverOptions &o, const std::string &p) {
            Ui::ParamPage::Traits<TropoMap>::store(p + key, atc(o));
        };
        f.fetch = [key, at](SolverOptions &o, const std::string &p) {
            Ui::ParamPage::Traits<TropoMap>::fetch(p + key, at(o), Ui::ParamPage::unbounded);
        };
        f.reset = [at, atc](SolverOptions &o, const SolverOptions &d) { at(o) = atc(d); };
        return f;
    }

    SolverGroup obsGroup(const bool withPhaseSigma) {
        SolverGroup g{
            "观测模型", {
                field("码噪声 sigma (m)", "sigIFCode", &SppConfig::sigIFCode, &SolverOptions::spp),
            }
        };
        if (withPhaseSigma)
            g.fields.push_back(field("相位噪声 sigma (m)", "sigmaPhase", &SppConfig::sigmaPhase, &SolverOptions::spp));
        g.fields.push_back(field("sin(el) 下限 (0=跟随截止角)", "minSinElev", &SppConfig::minSinElev,
                                 &SolverOptions::spp));
        return g;
    }

    SolverGroup corrGroup() {
        return {
            "改正模型", {
                flag("相位缠绕 (windup)", "applyWindup", &SppConfig::applyWindup, &SolverOptions::spp),
                flag("固体潮改正", "applySolidTide", &SppConfig::applySolidTide, &SolverOptions::spp),
            }
        };
    }

    SolverGroup tropoGroup(const bool withGrad, const bool withZtd = false) {
        SolverGroup g{"对流层", {}};
        if (withGrad)
            g.fields.push_back(flag("对流层梯度", "useTroGrad", &SppConfig::useTroGrad, &SolverOptions::spp));
        g.fields.push_back(combo("映射函数", "tropoMap",
                                 sTropoMapItems, kTropoMapCount, &SolverOptions::spp, &SppConfig::tropoMap));
        if (withZtd) {
            g.title = "对流层 / ZTD";
            g.fields.push_back(field("ZTD 初值 (m)", "ztdInitValue", &SppConfig::ztdInitValue, &SolverOptions::spp));
            g.fields.push_back(field("ZTD 初始方差", "ztdInitVar", &SppConfig::ztdInitVar, &SolverOptions::spp));
            g.fields.push_back(field("ZTD 过程噪声", "ztdProcNoise", &SppConfig::ztdProcNoise, &SolverOptions::spp));
            g.fields.push_back(field("ZTD 先验 sigma (m)", "ztdPriorSigma", &SppConfig::ztdPriorSigma,
                                     &SolverOptions::spp));
            g.fields.push_back(field("ZTD sigma0 (m)", "troZtdSigma", &SppConfig::troZtdSigma, &SolverOptions::spp));
        }
        return g;
    }

    SolverGroup lsqIterGroup() {
        return {
            "迭代 / 收敛", {
                field("收敛阈值 (m)", "convEps", &SppConfig::convEps, &SolverOptions::spp),
                field("最大迭代次数", "maxIter", &SppConfig::maxIter, &SolverOptions::spp),
                field("起始收敛检查迭代", "minCheckIter", &SppConfig::minCheckIter, &SolverOptions::spp),
            }
        };
    }

    SolverGroup filterIterGroup(const bool withSppResidualThr) {
        SolverGroup g{
            "迭代 / 收敛", {
                field("收敛阈值 (m)", "convEps", &SppConfig::convEps, &SolverOptions::spp),
                field("滤波更新最少卫星数 (0=不限)", "minSatsForUpdate", &SppConfig::minSatsForUpdate,
                      &SolverOptions::spp),
                field("钟差过程噪声", "clockNoise", &SppConfig::clockNoise, &SolverOptions::spp),
                field("重锚定最大迭代", "maxReanchorIter", &SppConfig::maxReanchorIter, &SolverOptions::spp),
            }
        };
        if (withSppResidualThr)
            g.fields.push_back(field("SPP 残差 RMS 阈值 (m)", "sppResidualThr", &SppConfig::sppResidualThr,
                                     &SolverOptions::spp));
        return g;
    }

    SolverGroup arGroup() {
        return {
            "模糊度与 AR", {
                flag("使用 AR 固定", "useAR", &SppConfig::useAR, &SolverOptions::spp),
                field("模糊度初始方差", "ambInitVar", &SppConfig::ambInitVar, &SolverOptions::spp),
                field("模糊度过程噪声", "ambNoise", &SppConfig::ambNoise, &SolverOptions::spp),
                field("最大保持模糊度间隙 (s)", "keepAmbMaxGap", &SppConfig::keepAmbMaxGap, &SolverOptions::spp),
                field("LAMBDA 成功率阈值", "arRatioThreshold", &SppConfig::arRatioThreshold, &SolverOptions::spp),
                field("进 LAMBDA 的模糊度 std 上限 (m)", "arSigGate", &SppConfig::arSigGate, &SolverOptions::spp),
            }
        };
    }

    SolverGroup codeBiasGroup() {
        return {
            "码偏差 / GLONASS IFCB", {
                field("码偏过程噪声", "codeBiasProcNoise", &SppConfig::codeBiasProcNoise, &SolverOptions::spp),
                field("码偏初始方差", "codeBiasInitVar", &SppConfig::codeBiasInitVar, &SolverOptions::spp),
                field("GLO IFCB 过程噪声", "gloIfcbProcNoise", &SppConfig::gloIfcbProcNoise, &SolverOptions::spp),
                field("GLO IFCB 初始方差", "gloIfcbInitVar", &SppConfig::gloIfcbInitVar, &SolverOptions::spp),
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
        g.fields.push_back(field("首历元位置方差", "firstEpochPosVar", &SppConfig::firstEpochPosVar,
                                 &SolverOptions::spp));
        g.fields.push_back(field("位置方差下界", "posVarFloor", &SppConfig::posVarFloor, &SolverOptions::spp));
        if (leo) {
            g.fields.push_back(field("运动检测阈值", "motionThresh", &SolverOptions::motionThresh));
            g.fields.push_back(field("SPP 预测阈值", "sppVsPredThresh", &SolverOptions::sppVsPredThresh));
        }
        return g;
    }

    SolverGroup slipGroup() {
        return {
            "周跳探测", {
                flag("使用周跳探测", "useCycleSlip", &SppConfig::useCycleSlip, &SolverOptions::spp),
                field("GF 周跳阈值倍数", "gfSlipMult", &SppConfig::gfSlipMult, &SolverOptions::spp),
            }
        };
    }

    SolverGroup isbGroup() {
        return {
            "系统间偏差 (ISB)", {
                field("ISB 过程噪声", "isbProcNoise", &SppConfig::isbProcNoise, &SolverOptions::spp),
                field("ISB 初始方差", "isbInitVar", &SppConfig::isbInitVar, &SolverOptions::spp),
            }
        };
    }

    const std::vector<SolverGroup> &schemaFor(const GnssTask::SolverMode mode) {
        static const std::vector spp = {
            obsGroup(false),
            tropoGroup(false),
            lsqIterGroup(),
            isbGroup(),
        };
        static const std::vector ppp = {
            obsGroup(true),
            corrGroup(),
            filterIterGroup(false),
            arGroup(),
            tropoGroup(true, true),
            codeBiasGroup(),
            posGroup(false),
            slipGroup(),
            isbGroup(),
        };
        static const std::vector leo = {
            obsGroup(true),
            corrGroup(),
            filterIterGroup(true),
            arGroup(),
            codeBiasGroup(),
            posGroup(true),
            slipGroup(),
            isbGroup(),
        };
        static const std::vector<SolverGroup> rtk = {
            {
                "模糊度与 AR", {
                    flag("使用 AR 固定", "useAR", &RtkConfig::useAR, &SolverOptions::rtk),
                    flag("完整协方差", "fullCovariance", &RtkConfig::fullCovariance, &SolverOptions::rtk),
                    field("LAMBDA 成功率阈值", "arThreshold", &RtkConfig::arThreshold, &SolverOptions::rtk),
                }
            },
            {
                "周跳探测", {
                    flag("使用周跳探测", "useCycleSlip", &RtkConfig::useCycleSlip, &SolverOptions::rtk),
                    flag("使用 GF 周跳", "useGfCycleSlip", &RtkConfig::useGfCycleSlip, &SolverOptions::rtk),
                    field("GF 周跳阈值倍数", "gfSlipMult", &RtkConfig::gfSlipMult, &SolverOptions::rtk),
                }
            },
            {
                "迭代 / 收敛", {
                    field("收敛阈值 (m)", "convEps", &RtkConfig::convEps, &SolverOptions::rtk),
                    field("浮点解最大迭代", "maxIterFloat", &RtkConfig::maxIterFloat, &SolverOptions::rtk),
                    field("固定解最大迭代", "maxIterFix", &RtkConfig::maxIterFix, &SolverOptions::rtk),
                    field("双差最少卫星数", "minSats", &RtkConfig::minSats, &SolverOptions::rtk),
                }
            },
            {
                "观测模型", {
                    rtkPhaseWeight("sin(el) 下限", "phaseWeightMinSinElev", &Weight::minSinElev,
                                   between(1e-6, 1.0)),
                    field("伪距:相位 方差比", "codeOverPhaseVar", &RtkConfig::codeOverPhaseVar, &SolverOptions::rtk),
                    rtkPhaseWeight("相位高度角模型 a", "phaseWeightA", &Weight::a),
                    rtkPhaseWeight("相位高度角模型 b", "phaseWeightB", &Weight::b),
                }
            },
            {
                "参考星选择", {
                    flag("排除北斗 GEO 参考星", "excludeGeoRef", &RtkConfig::excludeGeoRef, &SolverOptions::rtk),
                }
            },
        };

        static const std::vector<SolverGroup> empty;
        switch (mode) {
            case GnssTask::SolverMode::Spp: return spp;
            case GnssTask::SolverMode::Ppp: return ppp;
            case GnssTask::SolverMode::Leo: return leo;
            case GnssTask::SolverMode::Rtk: return rtk;
            default: return empty;
        }
    }
}

void Solvers::Helpers::renderAdvancedParams(SolverOptions &opts, const GnssTask::SolverMode mode) {
    Ui::ParamPage::render(schemaFor(mode), opts, GnssTask::defaultSolverOptions(mode));
}

void Solvers::Helpers::saveSolverParams(const GnssTask::SolverMode mode, const SolverOptions &opts) {
    Ui::ParamPage::save(schemaFor(mode), opts, spPrefix(mode));
}

void Solvers::Helpers::loadSolverParams(const GnssTask::SolverMode mode, SolverOptions &opts) {
    Ui::ParamPage::load(schemaFor(mode), opts, GnssTask::defaultSolverOptions(mode), spPrefix(mode));
}
