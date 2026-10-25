#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <map>
#include <set>
#include <utility>
#include <functional>

#include "GnssStruct.h"
#include "SPP.h"
#include "CoordConvert.h"
#include "QualityControl.h"
#include "EphemerisTable.h"
#include "Csv.h"        // 通用 CSV 轮子（读写）；导出内容在本文件定义，与轮子解耦
#include "Const.h"      // RAD_TO_DEG

namespace GnssTask {
    // 天顶图点：历元序号 + 方位/高度角(度) + 是否参与解算
    struct SkyPoint {
        int epIdx = 0;
        float azDeg = 0, elDeg = 0;
        bool used = true;
    };

    // 单颗 GNSS 卫星当前位置（ECEF, m）+ 是否参与解算
    struct SatVis {
        SatID sat;
        char sys = 0;
        Eigen::Vector3d pos{0, 0, 0};
        bool used = true;
    };

    // 一条历史轨迹点：历元序号 + ECEF 位置(m)
    struct TrajPoint {
        int epIdx = 0;
        Eigen::Vector3d pos{0, 0, 0};
    };

    // 星座集合 ↔ 字符串（ini 持久化用，顺序固定 G/R/E/C）
    inline std::string sysSetToString(const std::set<char> &s) {
        std::string out;
        for (char c: {'G', 'R', 'E', 'C'}) if (s.count(c)) out += c;
        return out;
    }

    inline std::set<char> stringToSysSet(const std::string &s) {
        std::set<char> out;
        for (char c: s) if (c == 'G' || c == 'R' || c == 'E' || c == 'C') out.insert(c);
        return out.empty() ? std::set{'G', 'R', 'E', 'C'} : out;
    }

    struct RealtimeEndpoint {
        std::string ip;
        std::string port;
    };

    inline int NextCfgIndex() {
        static std::atomic seq{0};
        return seq.fetch_add(1) + 1;
    }

    struct EpochData;
    struct ObsItem;

    struct SolverTaskBase : std::enable_shared_from_this<SolverTaskBase> {
        enum class State { Config, Running, Done };

        std::atomic<State> state{State::Config};

        std::thread worker;
        std::atomic<bool> loading{false};
        std::atomic<bool> done{false};
        std::atomic<bool> stop{false};

        std::string fileName;
        bool isRealtime = false;

        bool errorFlag() const;

        std::string errorMessage() const;

        void setError(std::string msg);

        void clearError();

        void syncErrorFrom(const SolverTaskBase &src);

        virtual std::shared_ptr<SolveTask> solveResult() = 0;

        virtual void renderConfigPanel() = 0;


        // 加载产品 / 配噪声。返回 false 表示初始化失败、本次解算放弃。
        virtual bool initSolver(void *solver) { return true; }

        // processEpoch 之前触发：SPP 需在此注入逐历元星历（须早于解算）。
        virtual void beforeSolveEpoch(void *solver, ObsItem *item) {
        }

        // processEpoch 之后写各家差异（参考原点、共享状态）。data 已解算并回填好。
        // 返回 false 表示该历元作废。
        virtual bool afterSolveEpoch(void *solver, EpochData *data, ObsItem *item, int index) { return true; }

        int cfgIndex = NextCfgIndex();

        virtual ~SolverTaskBase() {
            stop = true;
            if (worker.joinable()) worker.join();
        }

    private:
        // 只保护 errorMsg；hasError 自带原子保护，读取方无需加锁。
        mutable std::mutex errorMutex;
        std::atomic<bool> hasError{false};
        std::string errorMsg;
    };


    struct EpochData {
        unsigned int week = 0;
        double sow = 0;
        bool solved = false;

        int numObs = 0;
        std::vector<SatID> satIds;
        std::vector<PVT> satPVTs;
        std::vector<double> elevations;
        std::vector<double> azimuths;
        std::vector<bool> rejected;
        std::vector<TypeValueMap> allObs;
        Result sppResult;
        int numSatsResult = 0; // 必须初始化：未解出历元不会走到赋值分支，否则写出未初始化内存

        Eigen::Vector3d refECEF = Eigen::Vector3d::Zero();
        double rtnR = 0, rtnT = 0, rtnN = 0;

        Eigen::Vector3d rtkBaseFloat{0, 0, 0};
        Eigen::Vector3d rtkFloatXyz = Eigen::Vector3d::Zero(); // 浮点解 ECEF(=base+mBaselineFloat)，供 ENU 浮点对比线
        double rtkRatio = 0.0;
        int rtkStat = 0; // 0=无效, 1=Fixed, 2=Float

        void getFromObs(const ObsData &obs) {
            week = obs.weekSecond.week;
            sow = obs.weekSecond.sow;
            satIds.clear();
            elevations.clear();
            azimuths.clear();
            satPVTs.clear();
            rejected.clear();
            allObs.clear();
            for (auto &[sat, typeMap]: obs.satTypeValueData) {
                satIds.push_back(sat);
                elevations.push_back(0.0);
                azimuths.push_back(0.0);
                satPVTs.emplace_back();
                rejected.push_back(false);
                allObs.push_back(typeMap);
            }
            numObs = static_cast<int>(satIds.size());
        }

        void getFromSPP(const SPP &spp) {
            const auto &result = spp.result;
            const double r = result.xyz.norm();
            if (const bool posValid = r > 6.35e6 && r < 8.0e6; result.numSats > 0 && posValid) {
                solved = true;
                sppResult = result;
                const int n = static_cast<int>(satIds.size());
                for (int i = 0; i < n; i++) {
                    if (auto it = spp.satElevData.find(satIds[i]); it != spp.satElevData.end()) elevations[i] = it->second;
                    if (auto it2 = spp.satAzimData.find(satIds[i]); it2 != spp.satAzimData.end()) azimuths[i] = it2->second;
                    if (auto it3 = spp.satPVTTransTime.find(satIds[i]); it3 != spp.satPVTTransTime.end()) satPVTs[i] = it3->second;
                }
                // 排除判据：解算器显式排除的卫星，或没有有效星历/位置的卫星
                numSatsResult = 0;
                for (int i = 0; i < n; i++) {
                    const bool ok = spp.satRejected.count(satIds[i]) == 0 &&
                              satPVTs[i].p.squaredNorm() > 1.0;
                    rejected[i] = !ok;
                    if (ok) ++numSatsResult;
                }
            } else {
                solved = false;
                // 未收敛历元仍需合法写出：把 xyz 置零，杜绝未初始化内存流入 CSV/GUI。
                sppResult.reset();
            }
        }
    };

    struct PlotData {
        std::vector<double> times;
        std::vector<double> sigmaPs;
        std::vector<double> sigmaPFloatVals; // 浮点解 σP 时序（RTK 有，SPP/PPP/LEO 为空 → 仅画单线）
        std::vector<double> sigmaPFixedVals; // 固定解 σP 时序（RTK 固定历元有，其余为 NaN 断点）
        std::vector<double> sigmaVs;
        std::vector<double> pdops;
        std::vector<double> gdops, hdops, vdops, tdops; // 完整 DOP 时序（P·G·H·V·T），未解算填 NaN 断点
        std::vector<double> enu_e;
        std::vector<double> enu_n;
        std::vector<double> enu_u;
        bool newed = false;

        std::vector<double> ratioVals; // RTK:rtkRatio / PPP·LEO:sppResult.ratio
        std::vector<double> float_e, float_n, float_u; // 浮点解 ENU（与固定解对比）

        bool resRangeReady = false;
        double resYlo = -8.0, resYhi = 8.0;

        std::map<SatID, std::vector<double> > satResTimes;
        std::map<SatID, std::vector<double> > satResVals;
        std::map<SatID, std::vector<double> > satResPhaseTimes;
        std::map<SatID, std::vector<double> > satResPhaseVals;

        void insert(int index, const EpochData &ep, const XYZ &refECEF);

        void refreshENU(const std::vector<EpochData> &ep, const XYZ &refECEF);

        void clear();
    };

    struct ObsItem {
        ObsData obs;
        std::shared_ptr<EphemerisTable> eph;
    };

    // 扫 obsPath 同前缀的伴生星历文件（RINEX3 .??N/.??G/...）。做成 inline：
    // 既被 GUI（SolverTask.cpp 的 AutoDetectNav）调用，也被 CLI（spp_cli）调用，
    // 而 SolverTask.cpp 不进 CLI 链接，故定义必须随头文件可见，否则 CLI 链接期报未解析符号。
    inline std::vector<std::string> ScanNavFiles(const std::string &obsPath) {
        std::vector<std::string> result;
        if (obsPath.size() < 4) return result;
        const std::string base = obsPath.substr(0, obsPath.size() - 1);
        // RINEX 3: .??N, .??G, .??C, .??F 等
        for (auto &ext: {"N", "G", "C", "F", "E", "J", "I", "L"}) {
            std::string np = base + ext;
            if (std::ifstream t(np); t.good()) {
                t.close();
                result.push_back(np);
            }
        }
        return result;
    }

    std::string AutoDetectNav(const std::string &obsPath);

    enum class SolverMode { Spp, Ppp, Leo, Rtk, Ins };

    inline const char *modeLabel(const SolverMode m) {
        switch (m) {
            case SolverMode::Spp: return "SPP";
            case SolverMode::Ppp: return "PPP";
            case SolverMode::Leo: return "LEO";
            case SolverMode::Rtk: return "RTK";
            case SolverMode::Ins: return "INS";
        }
        return "SPP";
    }

    inline void exportCsvSpp(std::ostream &out, const std::vector<EpochData> &epochs,
                             const Eigen::Vector3d &ref, const bool skipUnsolved) {
        Csv::Writer w;
        w.row("Wk", "SOW", "ECEF-X/m", "ECEF-Y/m", "ECEF-Z/m",
              "REF-X/m", "REF-Y/m", "REF-Z/m", "EAST/m", "NORTH/m", "UP/m",
              "B/deg", "L/deg", "H/m", "VX/m", "VY/m", "VZ/m",
              "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "SigmaP", "SigmaV",
              "SatCount", "3dRMS/m").endRow();
        const int cols = w.colCount();
        for (const auto &r: epochs) {
            if (!r.solved) {
                // 未解出历元：只写 Wk/SOW/Status，其余留空。
                // 旧实现仅在 skipUnsolved=true(GUI) 时留空；CLI 传 false，于是把从未被赋值的
                // sppResult 整体写出 → 未初始化内存(1e203 量级)流入 CSV/绘图，表现为"解发散"。
                // 这里统一按未解出处理，杜绝脏值外泄（与 skipUnsolved 取值无关）。
                w.cell(r.week).cell(Csv::fx(r.sow, 3)).cell("Invalid");
                for (int i = 3; i < cols; ++i) w.cell("");
                w.endRow();
                continue;
            }
            const auto &res = r.sppResult;
            const ENU enu = XYZToENU(res.xyz, ref);
            const double rms3d = std::sqrt(enu.E() * enu.E() + enu.N() * enu.N() + enu.U() * enu.U());
            w.row(r.week, Csv::fx(r.sow, 3),
                  Csv::f(res.xyz[0], 11), Csv::f(res.xyz[1], 11), Csv::f(res.xyz[2], 11),
                  Csv::f(ref.x(), 11), Csv::f(ref.y(), 11), Csv::f(ref.z(), 11),
                  Csv::f(enu.E(), 6), Csv::f(enu.N(), 6), Csv::f(enu.U(), 6),
                  Csv::f(res.blh[0] * RAD_TO_DEG, 10), Csv::f(res.blh[1] * RAD_TO_DEG, 10), Csv::f(res.blh[2], 4),
                  Csv::f(res.vel[0], 6), Csv::f(res.vel[1], 6), Csv::f(res.vel[2], 6),
                  Csv::f(res.pdop, 4), Csv::f(res.gdop, 4), Csv::f(res.hdop, 4), Csv::f(res.vdop, 4), Csv::f(res.tdop, 4),
                  Csv::f(res.sigmaP, 4), Csv::f(res.sigmaV, 4),
                  r.numSatsResult,
                  Csv::f(rms3d, 6));
        }
        out << w.str();
    }

    // 统一浮点/固定块：给定 ECEF 解与参考点，写出 X/Y/Z + E/N/U + 3dRMS + SigmaP（8 列）。
    // skip=true 时整块写空（未固定历元用），保持列对齐。3dRMS = |ENU(解, ref)|。
    inline void csvWriteSolBlock(Csv::Writer &w, const Eigen::Vector3d &ecef,
                                 const Eigen::Vector3d &ref, double sigma, bool skip) {
        if (skip) {
            for (int i = 0; i < 8; ++i) w.cell("");
            return;
        }
        const ENU enu = XYZToENU(ecef, ref);
        const double rms = std::sqrt(enu.E() * enu.E() + enu.N() * enu.N() + enu.U() * enu.U());
        w.cell(Csv::f(ecef.x(), 11)).cell(Csv::f(ecef.y(), 11)).cell(Csv::f(ecef.z(), 11))
                .cell(Csv::f(enu.E(), 6)).cell(Csv::f(enu.N(), 6)).cell(Csv::f(enu.U(), 6))
                .cell(Csv::f(rms, 6)).cell(Csv::f(sigma, 4));
    }

    // PPP：浮点解(Float-) + 固定解(Fix-) 双块，各自 3dRMS 与 SigmaP；以 RTK 为准统一布局。
    // LEO 继承 PPPKinematic→PPPStatic，字段同源，故此处布局 PPP/LEO 通用。
    inline void exportCsvPpp(std::ostream &out, const std::vector<EpochData> &epochs,
                             const Eigen::Vector3d &ref, bool skipUnsolved) {
        Csv::Writer w;
        w.row("Wk", "SOW", "Status",
              "Float-X/m", "Float-Y/m", "Float-Z/m", "Float-E/m", "Float-N/m", "Float-U/m", "Float-3dRMS/m", "Float-SigmaP/m",
              "Fix-X/m", "Fix-Y/m", "Fix-Z/m", "Fix-E/m", "Fix-N/m", "Fix-U/m", "Fix-3dRMS/m", "Fix-SigmaP/m",
              "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "Ratio", "SatCount",
              "REF-X/m", "REF-Y/m", "REF-Z/m", "Vel-X/m", "Vel-Y/m", "Vel-Z/m").endRow();
        const int cols = w.colCount();
        for (const auto &r: epochs) {
            if (!r.solved) {
                w.cell(r.week).cell(Csv::fx(r.sow, 3)).cell("Invalid");
                for (int i = 3; i < cols; ++i) w.cell("");
                w.endRow();
                continue;
            }
            const auto &res = r.sppResult;
            const bool fixed = (res.stat == 1);
            const char *statStr = fixed ? "Fixed" : (res.stat == 2 ? "Float" : "Invalid");
            const Eigen::Vector3d fixE = fixed ? res.xyzFixed : Eigen::Vector3d::Zero();
            w.cell(r.week).cell(Csv::fx(r.sow, 3)).cell(statStr);
            csvWriteSolBlock(w, res.xyzFloat, ref, res.sigmaPFloat, false);
            csvWriteSolBlock(w, fixE, ref, res.sigmaPFixed, !fixed);
            w.cell(Csv::f(res.pdop, 4)).cell(Csv::f(res.gdop, 4)).cell(Csv::f(res.hdop, 4)).cell(Csv::f(res.vdop, 4)).cell(
                        Csv::f(res.tdop, 4))
                    .cell(res.ratio).cell(r.numSatsResult);
            w.cell(Csv::f(ref.x(), 11)).cell(Csv::f(ref.y(), 11)).cell(Csv::f(ref.z(), 11))
                    .cell(Csv::f(res.vel[0], 6)).cell(Csv::f(res.vel[1], 6)).cell(Csv::f(res.vel[2], 6));
            w.endRow();
        }
        out << w.str();
    }

    // LEO：逐历元参考轨道 REF + RTN(径向/沿迹/法向)；浮点 + 固定解双块（以 RTK 为准统一布局）。
    // 有参考轨道额外给 RTN；无参考轨道省 RTN（但保留 REF-ECEF 作为绝对参考）。
    inline void exportCsvLeo(std::ostream &out, const std::vector<EpochData> &epochs,
                             const Eigen::Vector3d &ref, bool hasRefOrbit, bool skipUnsolved) {
        Csv::Writer w;
        if (hasRefOrbit)
            w.row("Wk", "SOW", "Status",
                  "Float-X/m", "Float-Y/m", "Float-Z/m", "Float-E/m", "Float-N/m", "Float-U/m", "Float-3dRMS/m", "Float-SigmaP/m",
                  "Fix-X/m", "Fix-Y/m", "Fix-Z/m", "Fix-E/m", "Fix-N/m", "Fix-U/m", "Fix-3dRMS/m", "Fix-SigmaP/m",
                  "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "Ratio", "SatCount",
                  "REF-X/m", "REF-Y/m", "REF-Z/m", "R/m", "T/m", "N/m", "Vel-X/m", "Vel-Y/m", "Vel-Z/m").endRow();
        else
            w.row("Wk", "SOW", "Status",
                  "Float-X/m", "Float-Y/m", "Float-Z/m", "Float-E/m", "Float-N/m", "Float-U/m", "Float-3dRMS/m", "Float-SigmaP/m",
                  "Fix-X/m", "Fix-Y/m", "Fix-Z/m", "Fix-E/m", "Fix-N/m", "Fix-U/m", "Fix-3dRMS/m", "Fix-SigmaP/m",
                  "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "Ratio", "SatCount",
                  "REF-X/m", "REF-Y/m", "REF-Z/m", "Vel-X/m", "Vel-Y/m", "Vel-Z/m").endRow();
        const int cols = w.colCount();
        for (const auto &r: epochs) {
            if (!r.solved) {
                w.cell(r.week).cell(Csv::fx(r.sow, 3)).cell("Invalid");
                for (int i = 3; i < cols; ++i) w.cell("");
                w.endRow();
                continue;
            }
            const auto &res = r.sppResult;
            const bool fixed = (res.stat == 1);
            const char *statStr = fixed ? "Fixed" : (res.stat == 2 ? "Float" : "Invalid");
            const Eigen::Vector3d fixE = fixed ? res.xyzFixed : Eigen::Vector3d::Zero();
            // LEO 的"参考"是随时间运动的轨道，逐历元值存放在 r.refECEF（LEO 解算回填，
            // 与 rtnR/T/N 同源）；函数参数 ref 只是首历元（或配置文件）的静态参考。
            // 用静态 ref 会让 REF 三列全天冻结在同一组坐标，且 E/N/U、3dRMS 变成
            // 相对首历元参考的 10^6 m 量级垃圾值，故这里一律改用逐历元的 refRow。
            const Eigen::Vector3d &refRow =
                    (hasRefOrbit && r.refECEF.squaredNorm() > 1.0) ? r.refECEF : ref;
            w.cell(r.week).cell(Csv::fx(r.sow, 3)).cell(statStr);
            csvWriteSolBlock(w, res.xyzFloat, refRow, res.sigmaPFloat, false);
            csvWriteSolBlock(w, fixE, refRow, res.sigmaPFixed, !fixed);
            w.cell(Csv::f(res.pdop, 4)).cell(Csv::f(res.gdop, 4)).cell(Csv::f(res.hdop, 4)).cell(Csv::f(res.vdop, 4)).cell(
                        Csv::f(res.tdop, 4))
                    .cell(res.ratio).cell(r.numSatsResult);
            w.cell(Csv::f(refRow.x(), 11)).cell(Csv::f(refRow.y(), 11)).cell(Csv::f(refRow.z(), 11));
            if (hasRefOrbit)
                w.cell(Csv::f(r.rtnR, 6)).cell(Csv::f(r.rtnT, 6)).cell(Csv::f(r.rtnN, 6));
            w.cell(Csv::f(res.vel[0], 6)).cell(Csv::f(res.vel[1], 6)).cell(Csv::f(res.vel[2], 6));
            w.endRow();
        }
        out << w.str();
    }

    // RTK：绝对坐标 = 基准站(ref) + 基线；浮点解 = base + 浮动基线，固定解 = 绝对固定位置。
    // 浮点/固定均给出 X/Y/Z + E/N/U + 3dRMS + SigmaP。双差消去接收机钟差 → GDOP==PDOP、TDOP==0
    //（与 RTKLIB 一致，非 bug）；5 个 DOP 全列以显式呈现该性质。
    inline void exportCsvRtk(std::ostream &out, const std::vector<EpochData> &epochs,
                             const Eigen::Vector3d &ref, bool skipUnsolved) {
        Csv::Writer w;
        w.row("Wk", "SOW", "Status",
              "Float-X/m", "Float-Y/m", "Float-Z/m", "Float-E/m", "Float-N/m", "Float-U/m", "Float-3dRMS/m", "Float-SigmaP/m",
              "Fix-X/m", "Fix-Y/m", "Fix-Z/m", "Fix-E/m", "Fix-N/m", "Fix-U/m", "Fix-3dRMS/m", "Fix-SigmaP/m",
              "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "Ratio", "NSats",
              "REF-X/m", "REF-Y/m", "REF-Z/m").endRow();
        const int cols = w.colCount();
        for (const auto &r: epochs) {
            if (!r.solved) {
                w.cell(r.week).cell(Csv::fx(r.sow, 3)).cell("Invalid");
                for (int i = 3; i < cols; ++i) w.cell("");
                w.endRow();
                continue;
            }
            const auto &res = r.sppResult;
            const bool fixed = (res.stat == 1);
            const char *statStr = fixed ? "Fixed" : (res.stat == 2 ? "Float" : "Invalid");
            const Eigen::Vector3d base(ref.x(), ref.y(), ref.z());
            const Eigen::Vector3d floatE = base + r.rtkBaseFloat; // 浮点绝对坐标 = 基准站 + 浮动基线
            const Eigen::Vector3d fixE = fixed ? res.xyz : Eigen::Vector3d::Zero(); // 固定绝对坐标 = 解算输出（未固定留空）
            w.cell(r.week).cell(Csv::fx(r.sow, 3)).cell(statStr);
            csvWriteSolBlock(w, floatE, ref, res.sigmaPFloat, false);
            csvWriteSolBlock(w, fixE, ref, res.sigmaPFixed, !fixed);
            w.cell(Csv::f(res.pdop, 4)).cell(Csv::f(res.gdop, 4)).cell(Csv::f(res.hdop, 4)).cell(Csv::f(res.vdop, 4)).cell(
                        Csv::f(res.tdop, 4))
                    .cell(r.rtkRatio).cell(r.numSatsResult);
            w.cell(Csv::f(ref.x(), 11)).cell(Csv::f(ref.y(), 11)).cell(Csv::f(ref.z(), 11));
            w.endRow();
        }
        out << w.str();
    }

    // 统一入口：按 mode 派发到各解算自己的导出实现。
    inline void exportCsv(std::ostream &out, const std::vector<EpochData> &epochs,
                          SolverMode mode, const Eigen::Vector3d &ref, bool hasRefOrbit,
                          bool skipUnsolved) {
        switch (mode) {
            case SolverMode::Ppp: exportCsvPpp(out, epochs, ref, skipUnsolved);
                break;
            case SolverMode::Leo: exportCsvLeo(out, epochs, ref, hasRefOrbit, skipUnsolved);
                break;
            case SolverMode::Rtk: exportCsvRtk(out, epochs, ref, skipUnsolved);
                break;
            default: exportCsvSpp(out, epochs, ref, skipUnsolved);
                break; // SPP
        }
    }

    struct SolveTask : SolverTaskBase {
        std::string filePath;
        bool isRinex = false;

        std::mutex mutex;

        std::set<char> enabledSystems{'G', 'R', 'E', 'C'};
        double cutoffDeg = 15.0;
        std::string brdcPathBuf; // 广播星历（SPP 单文件版，与 PPP/LEO/RTK 字段对齐）

        std::atomic<float> readProgress{0.0f};
        std::atomic<int> solvingProgress{0};
        std::atomic<int> solvedCount{0};
        std::atomic<int> totalEpochs{0};

        std::mutex queueMutex;
        std::condition_variable queueCv;
        std::queue<ObsItem> obsQueue;
        std::atomic<bool> navReady{false};
        std::atomic<bool> readDone{false};
        std::atomic<bool> solvingDone{false};

        // ---- 解算结果----
        std::vector<EpochData> epochs;
        PlotData plotData;
        std::mutex plotMutex;
        XYZ refECEF{0, 0, 0};
        bool initializedRefECEF = false;
        bool hasRefOrbit = false;

        bool hasLeo = false;
        Eigen::Vector3d leoPos3d{0, 0, 0};
        Eigen::Vector3d leoRef3d{0, 0, 0};
        std::vector<TrajPoint> leoTraj;
        std::vector<TrajPoint> leoRefTraj;
        std::vector<SatVis> gnssVis;
        std::map<SatID, std::vector<TrajPoint> > gnssTraj;
        std::vector<double> rtnTimes, rtnR, rtnT, rtnN, rtnD3;

        std::mutex qcInputMutex;
        std::vector<QC::QCObsEpoch> qcInput;

        // 天顶图轨迹增量缓存
        using SkyTrackPt = SkyPoint;
        std::map<SatID, std::vector<SkyTrackPt> > skyTracks;
        int skyTracksBuilt = 0;

        int selectedEpoch = -1;
        int selectedSatIdx = -1;

        std::string obsPathBuf;
        std::vector<RealtimeEndpoint> rtEndpoints;

        SolverMode mode{SolverMode::Spp};

        std::shared_ptr<QualityReport> qcReport;
        std::atomic<bool> qcReady{false};
        std::mutex qcMutex;
        std::thread qcWorker;

        std::atomic<bool> qcComputing{false};
        bool qcLazy = false;
        bool hasNav = false;
        bool noEphSolve = false;

        std::string qcTabLabel = "质量分析";
        std::vector<std::pair<std::string, std::shared_ptr<SolveTask> > > extraQcTabs;

        ~SolveTask() override {
            stop = true;
            if (qcWorker.joinable()) {
                if (qcWorker.get_id() == std::this_thread::get_id()) qcWorker.detach();
                else qcWorker.join();
            }
        }

        // 结果即自己：SPP / 实时流没有外层容器，solveResult() 就是自身。
        std::shared_ptr<SolveTask> solveResult() override { return std::static_pointer_cast<SolveTask>(shared_from_this()); }

        // 把本任务结果导出为 CSV（委托给上面共享的 GnssTask::exportCsv）。
        void exportCsv(std::ostream &out, bool skipUnsolved = true) const {
            GnssTask::exportCsv(out, epochs, mode, refECEF, hasRefOrbit, skipUnsolved);
        }

        void renderConfigPanel() override {
        }
    };

    int ReadRinexObsToQueue(const std::shared_ptr<SolveTask> &core, const std::atomic<bool> &stopFlag);

    // 观测文件读取分发（与 app/core/ObsFormat.h classifyObsPath 配对）：
    // isRinex 由各面板开始解算前经 classifyObsPath 置位，这里只按标志分派，
    // 不再各自判断后缀。返回值语义同 ReadRinexObsToQueue：
    //   -1 = 打开或解析失败（已置 hasError/errorMsg/readDone 并 notify_all）；
    //    0  = 无历元（已置 hasError/"无历元"，仍继续走调用方 QC 收尾分支）；
    //   >0  = 入队历元数。
    int ReadObsToQueue(const std::shared_ptr<SolveTask> &core, const std::atomic<bool> &stopFlag);

    bool WaitNavReady(const std::shared_ptr<SolveTask> &core);

    bool NextObsQueue(const std::shared_ptr<SolveTask> &core, ObsItem &out);

    void CommitEpoch(const std::shared_ptr<SolveTask> &core, EpochData &&item,
                     const std::function<void(int, EpochData &)> &onStored = nullptr);


    void SyncTaskStatus(const std::shared_ptr<SolverTaskBase> &task, const std::shared_ptr<SolveTask> &core);

    // 单站解算（SPP / PPP / LEO）的解算线程主体：三家共用同一份骨架，
    // 差异全部落在 job 的三个虚函数钩子上（initSolver / beforeSolveEpoch / afterSolveEpoch），
    // 故这里只传「解算器实例 + 任务对象」，不再组装任何回调。
    // Solver 仍需模板化：它决定用哪个解算器类去 processEpoch，编译期即确定。
    template<typename Solver>
    void RunSingleStationSolve(const std::shared_ptr<SolveTask> &core,
                               Solver &solver,
                               const std::shared_ptr<SolverTaskBase> &job);
}
