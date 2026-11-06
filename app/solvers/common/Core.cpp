#include "Core.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include "Const.h"
#include "CoordConvert.h"
#include "RTK.h"
#include "SPP.h"
#include "solvers/leo/Task.h"
#include "solvers/ppp_kinematic/Task.h"
#include "solvers/ppp_static/Task.h"
#include "solvers/spp/Task.h"

namespace Solvers {
    namespace {
        constexpr double kEnuNaN = std::numeric_limits<double>::quiet_NaN();
        constexpr int kQcDirtyEvery = 200;
        constexpr auto kQcDirtyInterval = std::chrono::milliseconds(500);

        /// 一个历元的两套 ENU：实解用于轨迹，浮点解用于「浮点-固定」对比。
        /// 未出解时实解取 0、浮点取 NaN（图上断线）；插入与换基准重算共用这一套取法。
        struct EnuOfEpoch {
            ENU solved{0.0, 0.0, 0.0};
            ENU floatSol{kEnuNaN, kEnuNaN, kEnuNaN};
        };

        EnuOfEpoch enuFrom(const bool solved, const Eigen::Vector3d &xyz,
                           const Eigen::Vector3d &floatXyz, const XYZ &ref) {
            EnuOfEpoch out;
            if (!solved) return out;
            out.solved = Coord::XYZToENU(xyz, ref);
            out.floatSol = Coord::XYZToENU(floatXyz, ref);
            return out;
        }

        /// 实解与浮点解各自的 ECEF：RTK 的浮点解在历元自己身上，其余模式看 sppResult。
        void ecefOfEpoch(const Epoch &ep, Eigen::Vector3d &xyz, Eigen::Vector3d &floatXyz) {
            if (!ep.solved) {
                xyz.setZero();
                floatXyz.setZero();
                return;
            }
            const auto &r = ep.sppResult;
            xyz = r.xyz;
            if (ep.rtkStat != 0) floatXyz = ep.rtkFloatXyz;
            else if (r.xyzFloat.squaredNorm() > 1.0) floatXyz = r.xyzFloat;
            else floatXyz = r.xyz;
        }

        /// 逐卫星残差序列里的第 idx 个历元：时间序列存历元号且升序，二分找到就是。
        double residualAt(const std::map<SatID, std::vector<double> > &times,
                          const std::map<SatID, std::vector<double> > &vals,
                          const SatID sat, const int idx) {
            const auto itT = times.find(sat);
            if (itT == times.end()) return std::numeric_limits<double>::quiet_NaN();
            const auto itV = vals.find(sat);
            if (itV == vals.end()) return std::numeric_limits<double>::quiet_NaN();
            const auto &ts = itT->second;
            const auto hit = std::lower_bound(ts.begin(), ts.end(), static_cast<double>(idx));
            if (hit == ts.end() || *hit != static_cast<double>(idx)) return std::numeric_limits<double>::quiet_NaN();
            const auto k = static_cast<std::size_t>(hit - ts.begin());
            return k < itV->second.size() ? itV->second[k] : std::numeric_limits<double>::quiet_NaN();
        }

        /// 丢掉数据里已经不存在的卫星的轴，免得换了观测源后旧序列一直挂着。
        void pruneAxes(std::map<SatID, BrokenAxis> &axes, const std::map<SatID, std::vector<double> > &src) {
            for (auto it = axes.begin(); it != axes.end();) {
                if (src.count(it->first)) ++it;
                else it = axes.erase(it);
            }
        }
    }

    TypeValueMap ObsSnapshot::of(const std::size_t satIndex) const {
        TypeValueMap out;
        if (!types || satIndex + 1 >= start.size()) return out;
        for (int k = start[satIndex]; k < start[satIndex + 1]; ++k)
            out.emplace((*types)[slots[k]], values[k]);
        return out;
    }

    void Epoch::getFromObs(const ObsData &data) {
        week = data.weekSecond.week;
        sow = data.weekSecond.sow;
        satIds.clear();
        elevations.clear();
        azimuths.clear();
        satPos.clear();
        rejected.clear();
        rawObs.start.clear();
        rawObs.slots.clear();
        rawObs.values.clear();

        std::vector<std::string> names;
        std::map<std::string, std::size_t> columns;
        for (const auto &[sat, typeMap]: data.satTypeValueData) {
            satIds.push_back(sat);
            elevations.push_back(0.0);
            azimuths.push_back(0.0);
            satPos.emplace_back(Eigen::Vector3d::Zero());
            rejected.push_back(false);
            rawObs.start.push_back(static_cast<int>(rawObs.values.size()));
            for (const auto &[name, value]: typeMap) {
                const auto [col, added] = columns.emplace(name, columns.size());
                if (added) names.push_back(name);
                rawObs.slots.push_back(static_cast<unsigned short>(col->second));
                rawObs.values.push_back(value);
            }
        }
        rawObs.start.push_back(static_cast<int>(rawObs.values.size()));
        rawObs.types = std::make_shared<const std::vector<std::string> >(std::move(names));
        numObs = static_cast<int>(satIds.size());
    }

    void Epoch::fillSatFields(const SatValueMap &elev, const SatValueMap &azim,
                              const std::map<SatID, PVT> &pvts, const std::set<SatID> &usedSats) {
        const int n = static_cast<int>(satIds.size());
        for (int i = 0; i < n; i++) {
            if (auto it = elev.find(satIds[i]); it != elev.end()) elevations[i] = it->second;
            if (auto it2 = azim.find(satIds[i]); it2 != azim.end()) azimuths[i] = it2->second;
            if (auto it3 = pvts.find(satIds[i]); it3 != pvts.end()) satPos[i] = it3->second.p;
        }
        numSatsResult = 0;
        for (int i = 0; i < n; i++) {
            const bool ok = usedSats.count(satIds[i]) > 0 && satPos[i].squaredNorm() > 1.0;
            rejected[i] = !ok;
            if (ok) ++numSatsResult;
        }
    }

    void Epoch::getFromSPP(const ::SPP &spp) {
        const auto &result = spp.result;
        const double r = result.xyz.norm();
        if (const bool posValid = r > 6.35e6 && r < 8.0e6; result.numSats > 0 && posValid) {
            solved = true;
            sppResult = result;
            std::set<SatID> used;
            for (const auto &[sat, pvt]: spp.satPVTTransTime)
                if (!spp.satRejected.count(sat)) used.insert(sat);
            fillSatFields(spp.satElevData, spp.satAzimData, spp.satPVTTransTime, used);
        } else {
            solved = false;
            sppResult.reset();
        }
    }

    void Epoch::getFromRTK(const ::RTK &rtk) {
        const Result &r = rtk.result;
        if (r.numSats <= 0) {
            solved = false;
            return;
        }
        solved = true;
        sppResult = r;
        rtkStat = r.stat;
        rtkRatio = r.ratio;
        rtkBaseFloat = rtk.mBaselineFloat;
        if (r.stat == 1) rtkFixedBaseline = rtk.mBaselineFixed;
        rtkFloatXyz = rtk.mBaseXyz + rtk.mBaselineFloat;
        fillSatFields(rtk.satElevData, rtk.satAzimData, rtk.satPVTTransTime, rtk.mUsedSats);
    }

    bool Epoch::isFixed(const Mode mode) const {
        return mode == Mode::RTK ? rtkStat == 1 : sppResult.stat == 1;
    }

    void PlotData::insert(const int index, const Epoch &ep, const XYZ &refECEF, const Mode mode) {
        ++revision;
        times.push_back(index);
        const auto &result = ep.sppResult;
        const bool solved = ep.solved;
        solvedFlags.push_back(solved ? 1.0 : 0.0);
        if (solved) {
            ++solvedEpochs;
            if (result.vel.squaredNorm() > 1e-4) ++velEpochs;
            if (ep.isFixed(mode)) ++fixedEpochs;
        }
        constexpr double dopNaN = std::numeric_limits<double>::quiet_NaN();
        sigmaPs.push_back(solved ? result.sigmaP : 0.0);
        sigmaPFloatVals.push_back(solved && result.sigmaPFloat > 0 ? result.sigmaPFloat : dopNaN);
        sigmaVs.push_back(solved ? result.sigmaV : 0.0);
        if (solved && result.sigmaV > 1e-6) sigmaVNonZero = true;
        pdops.push_back(solved ? result.pdop : dopNaN);
        gdops.push_back(solved ? result.gdop : dopNaN);
        hdops.push_back(solved ? result.hdop : dopNaN);
        vdops.push_back(solved ? result.vdop : dopNaN);
        tdops.push_back(solved ? result.tdop : dopNaN);

        Eigen::Vector3d xyz, floatXyz;
        ecefOfEpoch(ep, xyz, floatXyz);
        ecef.push_back(xyz);
        floatEcef.push_back(floatXyz);

        const EnuOfEpoch enu = enuFrom(solved, xyz, floatXyz, refECEF);
        enu_e.push_back(enu.solved[0]);
        enu_n.push_back(enu.solved[1]);
        enu_u.push_back(enu.solved[2]);
        float_e.push_back(enu.floatSol[0]);
        float_n.push_back(enu.floatSol[1]);
        float_u.push_back(enu.floatSol[2]);

        if (solved) {
            for (auto &[satID, res]: result.postRes) {
                satResTimes[satID].push_back(index);
                satResVals[satID].push_back(res);
            }
            for (auto &[satID, res]: result.postResPhase) {
                satResPhaseTimes[satID].push_back(index);
                satResPhaseVals[satID].push_back(res);
            }
        }
    }

    void PlotData::appendFrom(const PlotData &src, const std::size_t from) {
        const auto tail = [from](auto &dst, const auto &s) {
            const std::size_t b = std::min(from, s.size());
            dst.insert(dst.end(), s.begin() + static_cast<std::ptrdiff_t>(b), s.end());
        };
        // 时间序列存历元号，按历元号找断点；数值序列与它逐点对齐，只能按长度接。
        const auto tailSeries = [from](auto &dstT, const auto &srcT, auto &dstV, const auto &srcV) {
            for (const auto &[sat, ts]: srcT) {
                const auto it = std::lower_bound(ts.begin(), ts.end(), static_cast<double>(from));
                auto &dt = dstT[sat];
                dt.insert(dt.end(), it, ts.end());
                const auto found = srcV.find(sat);
                if (found == srcV.end()) continue;
                auto &dv = dstV[sat];
                const std::size_t b = std::min(dv.size(), found->second.size());
                dv.insert(dv.end(), found->second.begin() + static_cast<std::ptrdiff_t>(b), found->second.end());
            }
        };
        tail(times, src.times);
        tail(solvedFlags, src.solvedFlags);
        tail(sigmaPs, src.sigmaPs);
        tail(sigmaPFloatVals, src.sigmaPFloatVals);
        tail(sigmaVs, src.sigmaVs);
        tail(pdops, src.pdops);
        tail(gdops, src.gdops);
        tail(hdops, src.hdops);
        tail(vdops, src.vdops);
        tail(tdops, src.tdops);
        tail(ecef, src.ecef);
        tail(floatEcef, src.floatEcef);
        tail(enu_e, src.enu_e);
        tail(enu_n, src.enu_n);
        tail(enu_u, src.enu_u);
        tail(float_e, src.float_e);
        tail(float_n, src.float_n);
        tail(float_u, src.float_u);
        tailSeries(satResTimes, src.satResTimes, satResVals, src.satResVals);
        tailSeries(satResPhaseTimes, src.satResPhaseTimes, satResPhaseVals, src.satResPhaseVals);
        solvedEpochs = src.solvedEpochs;
        velEpochs = src.velEpochs;
        fixedEpochs = src.fixedEpochs;
        sigmaVNonZero = src.sigmaVNonZero;
        revision = src.revision;
        gen = src.gen;
    }

    void PlotData::refreshENU(const XYZ &refECEF) {
        ++revision;
        ++gen;
        for (int i = 0; i < static_cast<int>(enu_e.size()); i++) {
            const EnuOfEpoch enu = enuFrom(solvedFlags[i] != 0.0, ecef[i], floatEcef[i], refECEF);
            enu_e[i] = enu.solved[0];
            enu_n[i] = enu.solved[1];
            enu_u[i] = enu.solved[2];
            float_e[i] = enu.floatSol[0];
            float_n[i] = enu.floatSol[1];
            float_u[i] = enu.floatSol[2];
        }
    }

    void PlotData::clear() {
        ++revision;
        ++gen;
        times.clear();
        solvedFlags.clear();
        sigmaPs.clear();
        sigmaPFloatVals.clear();
        sigmaVs.clear();
        pdops.clear();
        gdops.clear();
        hdops.clear();
        vdops.clear();
        tdops.clear();
        enu_e.clear();
        enu_n.clear();
        enu_u.clear();
        ecef.clear();
        floatEcef.clear();
        float_e.clear();
        float_n.clear();
        float_u.clear();
        satResTimes.clear();
        satResVals.clear();
        satResPhaseTimes.clear();
        satResPhaseVals.clear();
        solvedEpochs = 0;
        velEpochs = 0;
        fixedEpochs = 0;
        sigmaVNonZero = false;
    }

    void View::pushQCInput(const ObsData &epoch, const double sow) {
        std::lock_guard lk(qcInputMutex);
        qcStream.feed(QC::makeQCObsEpoch(epoch, sow));
    }

    void View::markQcDirty() {
        const auto now = std::chrono::steady_clock::now();
        if (++qcDirtyTick % kQcDirtyEvery != 0 && now - qcDirtyAt < kQcDirtyInterval) return;
        qcDirtyAt = now;
        qcDirty = true;
    }

    void View::clearQCInput(const double nominalInterval) {
        std::lock_guard lk(qcInputMutex);
        qcStream.reset(nominalInterval);
    }

    void View::endQCInput() {
        bool has = false;
        {
            std::lock_guard lk(qcInputMutex);
            qcStream.finish();
            has = qcStream.epochs() > 0;
        }
        if (has) qcDirty = true;
        else qcReady = true;
    }

    void View::ensureSkyTracks(const std::vector<Epoch> &epochs) {
        const int totalEp = static_cast<int>(epochs.size());
        for (int e = skyTracksBuilt; e < totalEp; ++e) {
            const auto &ep = epochs[e];
            for (int j = 0; j < static_cast<int>(ep.satIds.size()); ++j) {
                const double el = ep.elevations[j] * RAD_TO_DEG;
                if (el <= 0.0) continue;
                skyTracks[ep.satIds[j]].push_back(
                    {e, static_cast<float>(ep.azimuths[j] * RAD_TO_DEG), static_cast<float>(el),
                     ep.solved && !ep.rejected[j]});
            }
        }
        skyTracksBuilt = totalEp;
    }

    QC::QualityReport View::qcSnapshot() {
        std::lock_guard lk(qcInputMutex);
        return qcStream.report();
    }

    void BrokenAxis::build(const std::vector<double> &times, const double thrHint, const unsigned tag) {
        const int n = static_cast<int>(times.size());
        const bool resume = thrHint > 0.0 && tag == srcTag && thrHint == threshold && consumed > 0 &&
                            consumed <= n && times[consumed - 1] == lastTime;
        if (!resume) {
            t.clear();
            kept.clear();
            t.reserve(n);
            kept.reserve(n);
            consumed = 0;
            srcTag = tag;
            if (thrHint > 0.0) {
                threshold = thrHint;
            } else {
                double step = 1.0;
                std::vector<double> d;
                d.reserve(n > 1 ? n - 1 : 0);
                for (int i = 1; i < n; ++i) {
                    if (std::isfinite(times[i]) && std::isfinite(times[i - 1]) && times[i] > times[i - 1])
                        d.push_back(times[i] - times[i - 1]);
                }
                if (!d.empty()) {
                    std::sort(d.begin(), d.end());
                    step = d[d.size() / 2];
                }
                threshold = 2.5 * step;
            }
        }
        for (int i = consumed; i < n; ++i) {
            if (!std::isfinite(times[i])) continue;
            if (!kept.empty() && times[i] - t.back() > threshold) {
                t.push_back(t.back());
                kept.push_back(-1);
            }
            t.push_back(times[i]);
            kept.push_back(i);
        }
        consumed = n;
        if (n > 0) lastTime = times[n - 1];
    }

    std::vector<double> BrokenAxis::align(const std::vector<double> &y) const {
        std::vector<double> vv;
        vv.reserve(kept.size());
        for (const int k: kept) {
            if (k < 0 || k >= static_cast<int>(y.size()) || !std::isfinite(y[k]))
                vv.push_back(std::numeric_limits<double>::quiet_NaN());
            else
                vv.push_back(y[k]);
        }
        return vv;
    }

    double PlotSeries::codeResidual(const SatID sat, const int idx) const {
        return residualAt(data.satResTimes, data.satResVals, sat, idx);
    }

    double PlotSeries::phaseResidual(const SatID sat, const int idx) const {
        return residualAt(data.satResPhaseTimes, data.satResPhaseVals, sat, idx);
    }

    void PlotSeries::sync(const PlotData &pp) {
        newed = pp.revision != revision;
        if (!newed) return;
        if (pp.gen == data.gen && pp.times.size() >= data.times.size()) data.appendFrom(pp, data.times.size());
        else data = pp;
        revision = pp.revision;

        axis.build(data.times, 0.0, data.gen);
        for (const auto &[sat, ts]: data.satResTimes) resAxisCode[sat].build(ts, axis.threshold, data.gen);
        for (const auto &[sat, ts]: data.satResPhaseTimes) resAxisPhase[sat].build(ts, axis.threshold, data.gen);
        pruneAxes(resAxisCode, data.satResTimes);
        pruneAxes(resAxisPhase, data.satResPhaseTimes);
        hasResidual = !data.satResVals.empty() || !data.satResPhaseVals.empty();

        planE.clear();
        planN.clear();
        planR = 1e-6;
        const int n = static_cast<int>(data.enu_e.size());
        for (int i = 0; i < n; ++i) {
            if (i >= static_cast<int>(data.solvedFlags.size()) || data.solvedFlags[i] < 0.5) continue;
            const double e = data.enu_e[i];
            const double nn = i < static_cast<int>(data.enu_n.size()) ? data.enu_n[i] : 0.0;
            if (!std::isfinite(e) || !std::isfinite(nn)) continue;
            planE.push_back(e);
            planN.push_back(nn);
            planR = std::max(planR, std::max(std::abs(e), std::abs(nn)));
        }
        planR *= 1.15;
    }

    bool Core::errorFlag() const { return hasError.load(); }

    std::string Core::errorMessage() const {
        std::lock_guard lk(errorMutex);
        return errorMsg;
    }

    void Core::setError(std::string msg) {
        std::lock_guard lk(errorMutex);
        errorMsg = std::move(msg);
        hasError.store(true);
    }

    void Core::clearError() {
        std::lock_guard lk(errorMutex);
        errorMsg.clear();
        hasError.store(false);
    }

    SolverOptions defaultSolverOptions(const Mode mode) {
        switch (mode) {
            case Mode::SPP: return SPP::defaultOptions();
            case Mode::PPPStatic: return PPPStatic::defaultOptions();
            case Mode::PPPKinematic: return PPPKinematic::defaultOptions();
            case Mode::LEO: return LEO::defaultOptions();
            case Mode::RTK: break;
        }
        return {};
    }

    bool Core::waitNavReady() {
        std::unique_lock lk(queueMutex);
        queueCv.wait(lk, [this] {
            return navReady.load() || readDone.load() || stop.load();
        });
        return !stop.load();
    }

    bool Core::nextObs(Observer::Item &out) {
        {
            std::unique_lock lk(queueMutex);
            queueCv.wait(lk, [this] {
                return !obsQueue.empty() || readDone.load() || stop.load();
            });
            if (obsQueue.empty()) return false;
            out = std::move(obsQueue.front());
            obsQueue.pop();
        }
        queueSpaceCv.notify_one();
        return !stop.load();
    }

    /// 提交一个历元：复用观测类型表、刷新图表序列并清掉重复残差。
    void Core::commitEpoch(Epoch &&item,
                           const std::function<void(int, Epoch &)> &onStored) {
        std::lock_guard lk(mutex);
        if (!epochs.empty() && epochs.back().rawObs.types && item.rawObs.types
            && *epochs.back().rawObs.types == *item.rawObs.types)
            item.rawObs.types = epochs.back().rawObs.types;
        const int idx = static_cast<int>(epochs.size());
        epochs.push_back(std::move(item));
        if (onStored) onStored(idx, epochs.back());
        try {
            std::lock_guard plk(view.plotMutex);
            view.plotData.insert(idx, epochs.back(), refECEF, mode);
        } catch (...) {
        }
        epochs.back().sppResult.postRes.clear();
        epochs.back().sppResult.postResPhase.clear();
        if (view.selectedEpoch == -1 || view.selectedEpoch == idx - 1) view.selectedEpoch = idx;
    }

    std::pair<int, int> Core::readEpochView() {
        std::lock_guard lk(mutex);
        const int epochCount = static_cast<int>(epochs.size());
        if (view.selectedEpoch >= epochCount) view.selectedEpoch = epochCount - 1;
        if (view.selectedEpoch < 0 && epochCount > 0) view.selectedEpoch = 0;
        return {epochCount, view.selectedEpoch};
    }
}
