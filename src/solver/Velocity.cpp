#include "Velocity.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Cholesky>
#include <Eigen/LU>

#include "core/Const.h"

namespace {
    constexpr double kDopplerWeightScale = 1.0 / 40.0;
    constexpr int kMinSats = 4;
    constexpr double kMinXyzSq = 1.0e10;
}

namespace Velocity {
    DopplerResult byDoppler(const Eigen::Vector3d &recvXyz, const std::vector<DopplerSat> &sats) {
        DopplerResult out;
        if (!recvXyz.allFinite() || recvXyz.squaredNorm() < kMinXyzSq) return out;

        std::vector<Eigen::Vector4d> rows;
        std::vector<double> obs, wts;
        rows.reserve(sats.size());
        obs.reserve(sats.size());
        wts.reserve(sats.size());

        for (const DopplerSat &s: sats) {
            if (s.freqHz <= 0.0 || !std::isfinite(s.dopplerHz)) continue;
            if (!s.pos.allFinite() || !s.vel.allFinite()) continue;
            const double rho = (s.pos - recvXyz).norm();
            if (rho < 1.0) continue;
            const double w = s.posWeight * kDopplerWeightScale;
            if (!(w > 0.0)) continue;
            const Eigen::Vector3d u = (recvXyz - s.pos) / rho;
            const double lambda = C_MPS / s.freqHz;
            obs.push_back(-lambda * s.dopplerHz + s.vel.dot(u));
            Eigen::Vector4d row;
            row << u[0], u[1], u[2], 1.0;
            rows.push_back(row);
            wts.push_back(w);
        }

        out.used = static_cast<int>(rows.size());
        if (out.used < kMinSats) return out;

        Eigen::Matrix4d N = Eigen::Matrix4d::Zero();
        Eigen::Vector4d rhs = Eigen::Vector4d::Zero();
        for (std::size_t i = 0; i < rows.size(); ++i) {
            N += wts[i] * rows[i] * rows[i].transpose();
            rhs += wts[i] * rows[i] * obs[i];
        }
        const Eigen::Vector4d x = N.ldlt().solve(rhs);
        if (!x.allFinite()) return out;
        out.vel = x.head<3>();
        out.clockDrift = x[3];

        // σ0 由后验残差估：正好 4 颗时无冗余，退回 1（此时 sigma 只是形式上的几何精度）。
        double sigma0Sq = 1.0;
        if (out.used > kMinSats) {
            double vwv = 0.0;
            for (std::size_t i = 0; i < rows.size(); ++i) {
                const double e = obs[i] - rows[i].dot(x);
                vwv += wts[i] * e * e;
            }
            sigma0Sq = vwv / static_cast<double>(out.used - kMinSats);
            if (!std::isfinite(sigma0Sq) || sigma0Sq <= 0.0) sigma0Sq = 1.0;
        }
        const Eigen::Matrix4d Q = N.inverse();
        for (int k = 0; k < 3; ++k)
            out.sigma[k] = std::sqrt(std::max(Q(k, k), 0.0) * sigma0Sq);
        out.ok = true;
        return out;
    }

    void byDifference(const std::vector<double> &timeSec, const std::vector<Eigen::Vector3d> &xyz,
                      std::vector<Eigen::Vector3d> &vel) {
        const std::size_t n = std::min(timeSec.size(), xyz.size());
        vel.assign(n, Eigen::Vector3d::Zero());
        const auto valid = [&](const std::size_t i) {
            return i < n && xyz[i].allFinite() && xyz[i].squaredNorm() > kMinXyzSq;
        };
        for (std::size_t i = 0; i < n; ++i) {
            if (!valid(i)) continue;
            std::size_t a = i, b = i;
            if (i > 0 && valid(i - 1)) a = i - 1;
            if (i + 1 < n && valid(i + 1)) b = i + 1;
            if (a == b) continue;
            const double dt = timeSec[b] - timeSec[a];
            if (!(dt > 0.0)) continue;
            vel[i] = (xyz[b] - xyz[a]) / dt;
        }
    }
}
