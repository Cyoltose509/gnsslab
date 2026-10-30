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
#include "Csv.h"
#include "Const.h"
#include "SolverOptions.h"

namespace GnssTask {
    struct SkyPoint {
        int epIdx = 0;
        float azDeg = 0, elDeg = 0;
        bool used = true;
    };

    struct SatVis {
        SatID sat;
        char sys = 0;
        Eigen::Vector3d pos{0, 0, 0};
        bool used = true;
    };

    struct TrajPoint {
        int epIdx = 0;
        Eigen::Vector3d pos{0, 0, 0};
    };

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


        virtual bool initSolver(void *solver) { return true; }

        virtual void beforeSolveEpoch(void *solver, ObsItem *item);


        virtual bool wantsKinematic() const { return false; }

        std::shared_ptr<EphemerisTable> lastGoodEph;

        virtual bool afterSolveEpoch(void *solver, EpochData *data, ObsItem *item, int index);

        int cfgIndex = NextCfgIndex();

        virtual ~SolverTaskBase() {
            stop = true;
            if (worker.joinable()) worker.join();
        }

    private:
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
        int numSatsResult = 0;

        Eigen::Vector3d refECEF = Eigen::Vector3d::Zero();
        double rtnR = 0, rtnT = 0, rtnN = 0;

        Eigen::Vector3d rtkBaseFloat{0, 0, 0};
        Eigen::Vector3d rtkFixedBaseline{0, 0, 0};
        Eigen::Vector3d rtkFloatXyz = Eigen::Vector3d::Zero();
        double rtkRatio = 0.0;
        int rtkStat = 0;

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
                numSatsResult = 0;
                for (int i = 0; i < n; i++) {
                    const bool ok = spp.satRejected.count(satIds[i]) == 0 &&
                              satPVTs[i].p.squaredNorm() > 1.0;
                    rejected[i] = !ok;
                    if (ok) ++numSatsResult;
                }
            } else {
                solved = false;
                sppResult.reset();
            }
        }
    };

    struct PlotData {
        std::vector<double> times;
        std::vector<double> sigmaPs;
        std::vector<double> sigmaPFloatVals;
        std::vector<double> sigmaPFixedVals;
        std::vector<double> sigmaVs;
        std::vector<double> pdops;
        std::vector<double> gdops, hdops, vdops, tdops;
        std::vector<double> enu_e;
        std::vector<double> enu_n;
        std::vector<double> enu_u;
        bool newed = false;

        std::vector<double> ratioVals;
        std::vector<double> float_e, float_n, float_u;

        bool resRangeReady = false;
        double resYlo = -8.0, resYhi = 8.0;

        bool resPhaseRangeReady = false;
        double resPhaseYlo = -0.5, resPhaseYhi = 0.5;

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

    inline std::vector<std::string> ScanNavFiles(const std::string &obsPath) {
        std::vector<std::string> result;
        if (obsPath.size() < 4) return result;
        const std::string base = obsPath.substr(0, obsPath.size() - 1);
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


    inline SolverOptions defaultSolverOptions(const SolverMode mode) {
        SolverOptions o;
        switch (mode) {
            case SolverMode::Spp:
                o.spp.tropoMap = TropoMap::Hopfield;
                break;
            case SolverMode::Leo:
                o.spp.sigmaPhase = 0.003;
                o.spp.firstEpochPosVar = 1.0;
                o.spp.ambNoise = 1.0E-7;
                o.spp.minSatsForUpdate = 4;
                o.spp.useCycleSlip = false;
                o.spp.applySolidTide = false;
                o.motionThresh = 100.0;
                break;
            default: break;
        }
        return o;
    }

    inline void exportCsvSpp(ostream &out, const vector<EpochData> &epochs,
                             const Eigen::Vector3d &ref) {
        Csv::Writer w;
        w.row("Wk", "SOW", "ECEF-X/m", "ECEF-Y/m", "ECEF-Z/m",
              "REF-X/m", "REF-Y/m", "REF-Z/m", "EAST/m", "NORTH/m", "UP/m",
              "B/deg", "L/deg", "H/m", "VX/m", "VY/m", "VZ/m",
              "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "SigmaP", "SigmaV",
              "SatCount", "3dRMS/m").endRow();
        const int cols = w.colCount();
        for (const auto &r: epochs) {
            if (!r.solved) {
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

    inline void csvWriteSolBlock(Csv::Writer &w, const Eigen::Vector3d &ecef,
                                 const Eigen::Vector3d &ref, const double sigma, const bool skip) {
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

    inline void exportCsvPpp(ostream &out, const vector<EpochData> &epochs,
                             const Eigen::Vector3d &ref) {
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
            const bool fixed = res.stat == 1;
            const char *statStr = fixed ? "Fixed" : res.stat == 2 ? "Float" : "Invalid";
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

    inline void exportCsvLeo(ostream &out, const vector<EpochData> &epochs,
                             const Eigen::Vector3d &ref, const bool hasRefOrbit) {
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
            const bool fixed = res.stat == 1;
            const char *statStr = fixed ? "Fixed" : res.stat == 2 ? "Float" : "Invalid";
            const Eigen::Vector3d fixE = fixed ? res.xyzFixed : Eigen::Vector3d::Zero();
            const Eigen::Vector3d &refRow =
                    hasRefOrbit && r.refECEF.squaredNorm() > 1.0 ? r.refECEF : ref;
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

    inline void exportCsvRtk(ostream &out, const vector<EpochData> &epochs,
                             const Eigen::Vector3d &ref) {
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
            const bool fixed = res.stat == 1;
            const char *statStr = fixed ? "Fixed" : res.stat == 2 ? "Float" : "Invalid";
            const Eigen::Vector3d base(ref.x(), ref.y(), ref.z());
            const Eigen::Vector3d floatE = r.rtkFloatXyz.squaredNorm() > 1.0
                                               ? r.rtkFloatXyz
                                               : base + r.rtkBaseFloat;
            const Eigen::Vector3d fixE = fixed ? res.xyz : Eigen::Vector3d::Zero();
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

    inline void exportCsv(ostream &out, const vector<EpochData> &epochs,
                          const SolverMode mode, const Eigen::Vector3d &ref, const bool hasRefOrbit) {
        switch (mode) {
            case SolverMode::Ppp: exportCsvPpp(out, epochs, ref);
                break;
            case SolverMode::Leo: exportCsvLeo(out, epochs, ref, hasRefOrbit);
                break;
            case SolverMode::Rtk: exportCsvRtk(out, epochs, ref);
                break;
            default: exportCsvSpp(out, epochs, ref);
                break;
        }
    }

    struct SolveTask : SolverTaskBase {
        std::string filePath;
        bool isRinex = false;

        std::mutex mutex;

        std::set<char> enabledSystems{'G', 'R', 'E', 'C'};
        double cutoffDeg = 15.0;
        double sampleInterval = 0.0;




        SolverOptions opts;

        std::string brdcPathBuf;

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


        std::vector<EpochData> epochs;
        PlotData plotData;
        std::mutex plotMutex;
        XYZ refECEF{0, 0, 0};
        bool initializedRefECEF = false;
        bool hasRefOrbit = false;

        bool isKinematic = false;

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


        std::map<SatID, std::vector<SkyPoint> > skyTracks;
        int skyTracksBuilt = 0;

        int selectedEpoch = -1;
        int selectedSatIdx = -1;
        int resMode = 0;

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


        std::shared_ptr<SolveTask> solveResult() override { return std::static_pointer_cast<SolveTask>(shared_from_this()); }


        void exportCsv(ostream &out) const {
            GnssTask::exportCsv(out, epochs, mode, refECEF, hasRefOrbit);
        }

        void renderConfigPanel() override {
        }
    };

    int ReadRinexObsToQueue(const std::shared_ptr<SolveTask> &core, const std::atomic<bool> &stopFlag);

    int ReadObsToQueue(const std::shared_ptr<SolveTask> &core, const std::atomic<bool> &stopFlag);

    bool WaitNavReady(const std::shared_ptr<SolveTask> &core);

    bool NextObsQueue(const std::shared_ptr<SolveTask> &core, ObsItem &out);

    void CommitEpoch(const std::shared_ptr<SolveTask> &core, EpochData &&item,
                     const std::function<void(int, EpochData &)> &onStored = nullptr);


    void SyncTaskStatus(const std::shared_ptr<SolverTaskBase> &task, const std::shared_ptr<SolveTask> &core);

    template<typename Solver>
    void RunSingleStationSolve(const std::shared_ptr<SolveTask> &core,
                               Solver &solver,
                               const std::shared_ptr<SolverTaskBase> &job);

    template <class Task>
    void RunSingleStationReader(const std::shared_ptr<Task> &task, const char *tag);

    template <class Task>
    void RunDualThreadJob(const std::shared_ptr<Task> &task, const char *tag,
                          void (*solveFn)(const std::shared_ptr<Task> &));
}
