#include "Filter.h"
#include "core/equ/Precision.h"

#include <algorithm>
#include <exception>

#include "Log.h"
#include "SPP.h"
#include "i18n/I18n.h"

std::size_t Filter::runForward(const Session &session) {
    clear();
    const bool solve = solvesForward();
    const bool trail = mode == SolverOptions::FilterMode::Both;
    std::size_t done = 0;
    for (std::size_t i = 0; session.feed.ensure(i); ++i) {
        ObsData obs = session.feed.obs(i);
        bool ok = false;
        // 纯后向档正序只扫不解，内嵌星历仍要在这趟注入：两趟共用同一份 Products，倒序趟直接看得见。
        if (session.prepare) session.prepare(i, *session.solver);
        if (solve) {
            try {
                ok = session.solver->processEpoch(obs);
            } catch (const std::exception &e) {
                LOG_ERROR << I18n::tr("log.filter.fwd_epoch_error", obs.epoch.m_sod, e.what());
            } catch (...) {
                LOG_ERROR << I18n::tr("log.filter.fwd_epoch_unknown", obs.epoch.m_sod);
            }
        }
        Out out;
        out.act = !solve ? Out::Act::Skip : (ok ? Out::Act::UseSolver : Out::Act::Fail);
        const bool goOn = session.onEpoch(i, *session.solver, out);
        if (trail) {
            Trail t;
            t.valid = goOn && out.solved && session.solver->result.xyzFloat.squaredNorm() > 1e12;
            if (t.valid) {
                t.pos = session.solver->result.xyzFloat;
                t.cov = session.solver->posCovariance();
                t.sigma0 = session.solver->sigma0;
            }
            mTrail.push_back(t);
        }
        ++done;
        if (!goOn) break;
    }
    return done;
}

std::size_t Filter::runBackward(const Session &session) const {
    std::size_t n = session.feed.count();
    while (session.feed.ensure(n)) {
        const std::size_t grown = session.feed.count();
        if (grown <= n) break;
        n = grown;
    }
    if (n == 0) return 0;
    const std::unique_ptr<SPP> bwd = session.make();
    if (!bwd) {
        LOG_ERROR << I18n::trc("log.filter.bwd_create_fail");
        return 0;
    }
    bwd->prod = session.solver->prod;
    bwd->mDirection = -1;

    std::size_t done = 0;
    for (std::size_t i = n; i-- > 0;) {
        ObsData obs = session.feed.obs(i);
        bool ok = false;
        try {
            ok = bwd->processEpoch(obs);
        } catch (const std::exception &e) {
            LOG_ERROR << I18n::tr("log.filter.bwd_epoch_error", i, e.what());
        } catch (...) {
        }

        const bool bwdValid = ok && bwd->result.xyzFloat.squaredNorm() > 1e12;
        const bool fwdValid = i < mTrail.size() && mTrail[i].valid;
        Out out;
        if (!fwdValid) {
            out.act = bwdValid ? Out::Act::UseSolver : Out::Act::Fail;
        } else if (bwdValid) {
            // 融合：两侧位置按 3x3 位置协方差加权，融合失败则保留正序解。
            const Eigen::Matrix3d PfInv = mTrail[i].cov.inverse();
            const Eigen::Matrix3d PbInv = bwd->posCovariance().inverse();
            if (PfInv.allFinite() && PbInv.allFinite()) {
                const Eigen::Matrix3d Pc = (PfInv + PbInv).inverse();
                if (Pc.allFinite()) {
                    out.act = Out::Act::Fuse;
                    out.xyz = Pc * (PfInv * mTrail[i].pos + PbInv * bwd->result.xyzFloat);
                    out.sigma = positionSigma(Pc, mTrail[i].sigma0);
                }
            }
        }
        if (!session.onEpoch(i, *bwd, out)) break;
        ++done;
    }
    return done;
}

void Filter::clear() { mTrail.clear(); }
