#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Config.h"
#include "core/geom/Ephemeris.h"
#include "core/obs/ObsStruct.h"
#include "core/equ/Result.h"
#include "core/SatID.h"
#include "Observer.h"
#include "QCProcessor.h"
#include "product/sp3/Reader.h"
#include "util/Clock.h"
#include "i18n/I18n.h"

class SPP;
class RTK;

namespace Solvers {
    struct SkyPoint {
        int epIdx = 0;
        float azDeg = 0, elDeg = 0;
        bool used = true;
    };

    struct TrajPoint {
        int epIdx = 0;
        Eigen::Vector3d pos{0, 0, 0};
    };


    struct ObsSnapshot {
        std::shared_ptr<const std::vector<std::string>> types;

        std::vector<int> start;

        std::vector<unsigned short> slots;
        std::vector<double> values;


        [[nodiscard]] TypeValueMap of(std::size_t satIndex) const;
    };


    struct Truth {
        RefForm form = RefForm::None;
        Eigen::Vector3d fixed = Eigen::Vector3d::Zero();
        std::string sp3Path;
        SatID sat = SatID('L', 49);
        bool fileReady = false;
        SP3::Reader reader;
    };

    struct Epoch {
        unsigned int week = 0;
        double sow = 0;
        bool solved = false;

        int numObs = 0;
        std::vector<SatID> satIds;

        std::vector<Eigen::Vector3d> satPos;
        std::vector<double> elevations;
        std::vector<double> azimuths;
        std::vector<bool> rejected;
        ObsSnapshot rawObs;
        Result sppResult;
        int numSatsResult = 0;

        Eigen::Vector3d refECEF = Eigen::Vector3d::Zero();
        double rtnR = 0, rtnT = 0, rtnN = 0;

        Eigen::Vector3d rtkBaseFloat{0, 0, 0};
        Eigen::Vector3d rtkFixedBaseline{0, 0, 0};
        Eigen::Vector3d rtkFloatXyz = Eigen::Vector3d::Zero();
        double rtkRatio = 0.0;
        int rtkStat = 0;

        void getFromObs(const ObsData &obs);

        void getFromSPP(const ::SPP &spp);

        void getFromRTK(const ::RTK &rtk);


        [[nodiscard]] bool isFixed(Mode mode) const;

    private:
        void fillSatFields(const SatValueMap &elev, const SatValueMap &azim,
                           const std::map<SatID, PVT> &pvts, const std::set<SatID> &usedSats);
    };


    struct PlotData {
        std::vector<double> times;
        std::vector<double> sigmaPs;
        std::vector<double> sigmaPFloatVals;
        std::vector<double> sigmaVs;
        std::vector<double> pdops;
        std::vector<double> gdops, hdops, vdops, tdops;
        std::vector<double> enu_e;
        std::vector<double> enu_n;
        std::vector<double> enu_u;

        std::vector<Eigen::Vector3d> ecef;
        std::vector<Eigen::Vector3d> floatEcef;

        std::vector<Eigen::Vector3d> refs;
        std::vector<double> solvedFlags;

        std::vector<double> float_e, float_n, float_u;

        std::map<SatID, std::vector<double> > satResTimes;
        std::map<SatID, std::vector<double> > satResVals;
        std::map<SatID, std::vector<double> > satResPhaseTimes;
        std::map<SatID, std::vector<double> > satResPhaseVals;

        int solvedEpochs = 0;
        int velEpochs = 0;
        int fixedEpochs = 0;
        bool sigmaVNonZero = false;
        unsigned revision = 0;


        unsigned gen = 0;

        void insert(int index, const Epoch &ep, const XYZ &refECEF, Mode mode);


        void replaceAll(const std::vector<Epoch> &epochs, const XYZ &refECEF, Mode mode);


        void appendFrom(const PlotData &src, std::size_t from);

        void refreshENU(const XYZ &refECEF);


        [[nodiscard]] double codeResidual(SatID sat, int idx) const;

        [[nodiscard]] double phaseResidual(SatID sat, int idx) const;

        void clear();
    };

    struct QcTab;


    struct BrokenAxis {
        std::vector<double> t;
        std::vector<int> kept;
        double threshold = 0.0;
        int consumed = 0;
        double lastTime = 0.0;
        unsigned srcTag = 0;

        void build(const std::vector<double> &times, double thrHint = 0.0, unsigned tag = 0);

        [[nodiscard]] std::vector<double> align(const std::vector<double> &y) const;
    };


    struct PlotSeries {
        unsigned revision = 0;
        bool newed = false;

        PlotData data;
        BrokenAxis axis;
        std::map<SatID, BrokenAxis> resAxisCode;
        std::map<SatID, BrokenAxis> resAxisPhase;


        std::vector<double> planE, planN;
        double planR = 1e-6;

        bool planTrack = false;
        Eigen::Vector3d planOrigin{0, 0, 0};
        bool planOriginSet = false;

        bool hasResidual = false;

        bool hasResRange = false, hasResPhaseRange = false;
        double resYlo = -8.0, resYhi = 8.0;
        double resPhaseYlo = -0.5, resPhaseYhi = 0.5;

        void sync(const PlotData &pp);


        void setPlanTrack(bool track);


        void setPlanOrigin(const Eigen::Vector3d &origin);
    };


    struct View {
        PlotData plotData;
        PlotSeries plotSeries;
        std::mutex plotMutex;

        bool hasLeo = false;
        Eigen::Vector3d leoPos3d{0, 0, 0};
        Eigen::Vector3d leoRef3d{0, 0, 0};
        std::vector<TrajPoint> leoTraj;
        std::vector<TrajPoint> leoRefTraj;
        std::vector<double> rtnTimes, rtnR, rtnT, rtnN, rtnD3;

        void pushQCInput(const ObsData &epoch, double sow);


        void markQcDirty();

        void clearQCInput(double nominalInterval);

        void endQCInput();

        [[nodiscard]] QC::QualityReport qcSnapshot();

        std::shared_ptr<QC::QualityReport> qcReport;
        std::atomic<bool> qcReady{false};
        std::mutex qcMutex;
        std::thread qcWorker;
        std::atomic<bool> qcComputing{false};
        std::atomic<bool> qcDirty{false};
        Clock::Point qcDirtyAt{};
        int qcDirtyTick = 0;

        const char *qcTabKey = "QualityControl";
        std::vector<std::shared_ptr<QcTab> > extraQcTabs;

        std::shared_ptr<void> uiTicket = std::make_shared<int>(0);

        std::map<SatID, std::vector<SkyPoint> > skyTracks;
        int skyTracksBuilt = 0;

        void ensureSkyTracks(const std::vector<Epoch> &epochs);


        void rebuildTruthTrails(const std::vector<Epoch> &epochs);

        int selectedEpoch = -1;

        int lastSolveCursor = -1;
        int selectedSatIdx = -1;
        int resMode = 0;

        std::string obsPathBuf;

        ~View() {
            if (qcWorker.joinable()) {
                if (qcWorker.get_id() == std::this_thread::get_id()) qcWorker.detach();
                else qcWorker.join();
            }
        }

    private:
        std::mutex qcInputMutex;
        QC::Stream qcStream{0.0};
    };

    struct QcTab {
        const char *labelKey = "QualityControl";
        std::string fileName;
        View view;
    };

    struct Core {
        enum class State { Config, Running, Done };

        Mode mode{Mode::SPP};

        std::string filePath;

        std::string fileName;

        const char *nameKey{};

        Observer::Format obsFormat = Observer::Format::Unknown;
        bool isRealtime = false;

        std::atomic<State> state{State::Config};
        std::atomic<bool> loading{false};
        std::atomic<bool> done{false};
        std::atomic<bool> stop{false};

        bool errorFlag() const;

        std::string errorMessage() const;

        void setError(std::string msg);

        void clearError();


        const char *displayName() const { return nameKey ? I18n::trc(nameKey) : fileName.c_str(); }

        std::mutex mutex;

        SolverOptions opts;

        std::mutex queueMutex;
        std::condition_variable queueCv;
        std::condition_variable queueSpaceCv;
        std::size_t queueDepth = 128;
        std::queue<Observer::Item> obsQueue;
        std::atomic<bool> navReady{false};
        std::atomic<bool> readDone{false};
        std::atomic<bool> solvingDone{false};

        enum class SolvePhase { Forward, Scan, Backward };

        std::atomic<int> solveEpoch{0};
        std::atomic<SolvePhase> solvePhase{SolvePhase::Forward};

        std::vector<Epoch> epochs;
        XYZ refECEF{0, 0, 0};
        std::atomic<bool> initializedRefECEF{false};
        Truth truth;


        Frame::Ref orbitFrame = Frame::Ref::IGS14;
        bool hasOrbitFrame = false;

        View view;


        void setupTruth(const SolverOptions &cfg);


        void applyObsPath(const std::string &obsPath);


        bool loadTruth();


        void warnFrameMismatch() const;


        void applyTruthAtEpoch(Epoch &ep, int index);


        void refreshTruthAtEpoch(Epoch &ep, const CommonTime &epoch);


        [[nodiscard]] bool enPlanStatic() const {
            return mode == Mode::PPPStatic || truth.form == RefForm::Fixed;
        }

        [[nodiscard]] bool truthReady() const {
            if (truth.form == RefForm::Fixed) return truth.fixed.squaredNorm() > 1.0;
            return truth.form == RefForm::File && truth.fileReady;
        }

        std::atomic<bool> hasNav{false};
        std::atomic<bool> noEphSolve{false};


        std::vector<RealtimeEndpoint> rtEndpoints;

        bool waitNavReady();

        bool nextObs(Observer::Item &out);

        void commitEpoch(Epoch &&item,
                         const std::function<void(int, Epoch &)> &onStored = nullptr);

        std::pair<int, int> readEpochView();

    private:
        mutable std::mutex errorMutex;
        std::atomic<bool> hasError{false};
        std::string errorMsg;
    };
}
