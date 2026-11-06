#include "Task.h"

#include <future>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "CoordConvert.h"
#include "LEO.h"
#include "Log.h"
#include "Observer.h"
#include "SPP.h"
#include "StringUtils.h"
#include "io/ProductLoad.h"
#include "solvers/common/CSV.h"

namespace Solvers::LEO {
    void Task::applyConfig(const SolverOptions &cfg) {
        opts = cfg;
        filePath = cfg.obsPath;
        fileName = Str::baseName(cfg.obsPath);
        isRINEX = Observer::isRINEX(Observer::classifyObsPath(cfg.obsPath));
        view.obsPathBuf = cfg.obsPath;
        refSp3PathBuf = cfg.refSp3;
        auxFiles = Str::split(cfg.aux, kIniListSep, false);
        // LEO 的 GNSS 产品与观测分属两套路径，统一收进 opts 供 ProductLoad 取用
        opts.sp3 = cfg.gnssSp3;
        opts.clk = cfg.gnssClk;
        opts.navPath = cfg.gnssBrdc;
        refPrn = cfg.refPrn;
        refSat = SatID('L', refPrn);
    }

    bool Task::initSolver(::SPP &solver) {
        ::LEO &leo = static_cast<::LEO &>(solver);
        hasRefOrbit = false;
        bool loadOk = true;

        {
            std::vector<std::future<bool> > jobs;
            ProductLoad::core(*this, leo, jobs, "LEO", modeKey(Mode::LEO));
            for (auto &j: jobs) {
                if (!j.get()) loadOk = false;
            }
        }
        if (!refSp3PathBuf.empty()) {
            LOG_INFO << "LEO 读取参考轨道 SP3: " << refSp3PathBuf;
            refRdr.read(refSp3PathBuf);
            const auto &rdata = refRdr.getData();
            bool refFound = false;
            for (const auto &[sat, _]: rdata)
                if (sat.system == 'L') {
                    refPrn = sat.id;
                    refFound = true;
                    break;
                }
            if (!refFound && !rdata.empty()) refPrn = rdata.begin()->first.id;
            refSat = SatID('L', refPrn);
            if (!leo.loadLeoReference(refSp3PathBuf, refSat)) {
                LOG_ERROR << "LEO loadLeoReference 失败";
                loadOk = false;
            } else hasRefOrbit = true;
        } else {
            LOG_INFO << "LEO: 未提供参考轨道 SP3，跳过 RTN 评估与参考轨迹（解算仍依赖精密星历/钟差）";
        }

        if (!auxFiles.empty()) {
            for (const auto &f: auxFiles) leo.loadAuxFile(f);
        } else {
            LOG_INFO << "LEO: 未加载辅助文件(.HDR 杠杆臂)，CoG→ARP 偏移为 0";
        }

        const auto products = ProductLoad::resolve(opts, modeKey(Mode::LEO));
        const bool srcOk = ProductLoad::requireSatellite("LEO", products.paths.sp3, products.paths.brdc);
        return (loadOk && srcOk) ? true : ProductLoad::fail(this);
    }

    bool Task::afterSolveEpoch(::SPP &, Solvers::Epoch &data,
                               Observer::Item &item, const int index) {
        if (!data.solved) return false;
        const Eigen::Vector3d leoPos = data.sppResult.xyz;
        if (leoPos.squaredNorm() <= 1e12) return false;

        Eigen::Vector3d refPos = leoPos;
        {
            std::lock_guard lk(mutex);
            view.leoPos3d = leoPos;
            view.leoTraj.push_back({index, leoPos});
            view.hasLeo = true;

            if (hasRefOrbit) {
                const PVT ref = refRdr.getPVT(refSat, item.obs.epoch);
                refPos = ref.p.squaredNorm() > 1e12 ? ref.p : leoPos;
                const Eigen::Vector3d rtn = Coord::XYZToRTN(leoPos, refPos, ref.v);
                data.refECEF = refPos;
                data.rtnR = rtn(0);
                data.rtnT = rtn(1);
                data.rtnN = rtn(2);
                view.leoRef3d = refPos;
                view.leoRefTraj.push_back({index, refPos});

                view.rtnTimes.push_back(index);
                view.rtnR.push_back(rtn(0));
                view.rtnT.push_back(rtn(1));
                view.rtnN.push_back(rtn(2));
                view.rtnD3.push_back((leoPos - refPos).norm());
            }
            if (!initializedRefECEF) {
                refECEF = refPos;
                initializedRefECEF = true;
            }
        }
        return true;
    }

    std::unique_ptr<::SPP> Task::makeSolver() {
        return std::make_unique<::LEO>();
    }

    void Task::exportCSV(std::ostream &out) const {
        const Eigen::Vector3d &ref = refECEF;
        const bool withRefOrbit = hasRefOrbit;
        CSV::Writer w;
        if (withRefOrbit)
            w.row("Wk", "SOW", "Status",
                  "Float-X/m", "Float-Y/m", "Float-Z/m", "Float-E/m", "Float-N/m", "Float-U/m",
                  "Float-3dRMS/m", "Float-SigmaP/m",
                  "Fix-X/m", "Fix-Y/m", "Fix-Z/m", "Fix-E/m", "Fix-N/m", "Fix-U/m",
                  "Fix-3dRMS/m", "Fix-SigmaP/m",
                  "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "Ratio", "SatCount",
                  "REF-X/m", "REF-Y/m", "REF-Z/m", "R/m", "T/m", "N/m",
                  "Vel-X/m", "Vel-Y/m", "Vel-Z/m").endRow();
        else
            w.row("Wk", "SOW", "Status",
                  "Float-X/m", "Float-Y/m", "Float-Z/m", "Float-E/m", "Float-N/m", "Float-U/m",
                  "Float-3dRMS/m", "Float-SigmaP/m",
                  "Fix-X/m", "Fix-Y/m", "Fix-Z/m", "Fix-E/m", "Fix-N/m", "Fix-U/m",
                  "Fix-3dRMS/m", "Fix-SigmaP/m",
                  "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "Ratio", "SatCount",
                  "REF-X/m", "REF-Y/m", "REF-Z/m", "Vel-X/m", "Vel-Y/m", "Vel-Z/m").endRow();
        const int cols = w.colCount();
        for (const auto &r: epochs) {
            if (!r.solved) {
                csvInvalidRow(w, r, cols);
                continue;
            }
            const auto &res = r.sppResult;
            const bool fixed = r.isFixed(mode);
            const Eigen::Vector3d fixE = fixed ? res.xyzFixed : Eigen::Vector3d::Zero();
            const Eigen::Vector3d &refRow =
                    withRefOrbit && r.refECEF.squaredNorm() > 1.0 ? r.refECEF : ref;
            w.cell(r.week).cell(CSV::fx(r.sow, 3)).cell(csvStatStr(res.stat));
            csvWriteSolBlock(w, res.xyzFloat, refRow, res.sigmaPFloat, false);
            csvWriteSolBlock(w, fixE, refRow, res.sigmaPFixed, !fixed);
            csvDopCells(w, res).cell(res.ratio).cell(r.numSatsResult);
            w.cell(CSV::f(refRow.x(), 11)).cell(CSV::f(refRow.y(), 11)).cell(CSV::f(refRow.z(), 11));
            if (withRefOrbit)
                w.cell(CSV::f(r.rtnR, 6)).cell(CSV::f(r.rtnT, 6)).cell(CSV::f(r.rtnN, 6));
            w.cell(CSV::f(res.vel[0], 6)).cell(CSV::f(res.vel[1], 6)).cell(CSV::f(res.vel[2], 6));
            w.endRow();
        }
        out << w.str();
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->applyConfig(cfg);
        return task;
    }
}
