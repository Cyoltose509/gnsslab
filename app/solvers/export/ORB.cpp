
#include <ostream>
#include <vector>

#include "core/Const.h"
#include "core/geom/CoordConvert.h"
#include "Core.h"
#include "solvers/common/Task.h"
#include "TimeStruct.h"
#include "util/CSV.h"

namespace Solvers::Export {
    namespace {
        constexpr double kTwoPi = 2.0 * PI;

        struct State {
            Stamp stamp;
            Eigen::Vector3d pos = Eigen::Vector3d::Zero();
            Eigen::Vector3d vel = Eigen::Vector3d::Zero();
            bool ok = false;
        };

        double deg(const double rad) {
            double d = rad * RAD_TO_DEG;
            d = std::fmod(d, 360.0);
            if (d < 0.0) d += 360.0;
            return d;
        }

        /// 5 点 Lagrange 求导。中心差分在 30 s 采样下截断误差 ~1 m/s，折算到半长轴是公里级；
        /// 多项式求导把这一项压到厘米每秒量级。点不够时自动退化成单侧差分。
        Eigen::Vector3d velocityAt(const std::vector<State> &st, const std::size_t k) {
            std::vector<std::size_t> win;
            for (std::size_t i = k; i-- > 0 && win.size() < 2;)
                if (st[i].ok) win.insert(win.begin(), i);
            win.push_back(k);
            for (std::size_t i = k + 1; i < st.size() && win.size() < 5; ++i)
                if (st[i].ok) win.push_back(i);
            if (win.size() < 2) return Eigen::Vector3d::Zero();

            const double tk = st[k].stamp.abs();
            Eigen::Vector3d v = Eigen::Vector3d::Zero();
            for (std::size_t a = 0; a < win.size(); ++a) {
                const double ti = st[win[a]].stamp.abs();
                double coeff = 0.0;
                for (std::size_t b = 0; b < win.size(); ++b) {
                    if (b == a) continue;
                    const double tb = st[win[b]].stamp.abs();
                    if (std::fabs(ti - tb) < 1.0e-9) {
                        coeff = 0.0;
                        break;
                    }
                    double term = 1.0 / (ti - tb);
                    for (std::size_t m = 0; m < win.size(); ++m) {
                        if (m == a || m == b) continue;
                        const double tm = st[win[m]].stamp.abs();
                        term *= (tk - tm) / (ti - tm);
                    }
                    coeff += term;
                }
                v += coeff * st[win[a]].pos;
            }
            return v;
        }
    }

    Elements elementsFromState(const Eigen::Vector3d &r, const Eigen::Vector3d &v) {
        Elements el{};
        const double rn = r.norm();
        const double v2 = v.squaredNorm();
        if (!(rn > 1.0e6) || !std::isfinite(v2)) return el;

        const double invA = 2.0 / rn - v2 / EARTH_GM;
        if (std::fabs(invA) < 1e-12) return el;
        el.a = 1.0 / invA;

        const Eigen::Vector3d h = r.cross(v);
        const double hn = h.norm();
        if (!(hn > 0.0)) return el;
        el.inc = std::acos(std::clamp(h.z() / hn, -1.0, 1.0));

        const Eigen::Vector3d nVec(-h.y(), h.x(), 0.0);
        const double nn = nVec.norm();
        const Eigen::Vector3d eVec = ((v2 - EARTH_GM / rn) * r - r.dot(v) * v) / EARTH_GM;
        el.e = eVec.norm();

        if (nn > 1e-9) {
            el.raan = std::atan2(nVec.y(), nVec.x());
            if (el.e > 1e-9)
                el.argp = std::acos(std::clamp(nVec.dot(eVec) / (nn * el.e), -1.0, 1.0));
            else
                el.argp = 0.0;
            if (eVec.z() < 0.0) el.argp = kTwoPi - el.argp;
        }

        // 真近点角 → 偏近点角 → 平近点角
        double nu = 0.0;
        if (el.e > 1e-9)
            nu = std::acos(std::clamp(eVec.dot(r) / (el.e * rn), -1.0, 1.0));
        if (r.dot(v) < 0.0) nu = kTwoPi - nu;
        const double ecc = std::clamp(el.e, 0.0, 0.999999);
        const double eAnom = 2.0 * std::atan2(std::sqrt(1.0 - ecc) * std::sin(nu / 2.0),
                                              std::sqrt(1.0 + ecc) * std::cos(nu / 2.0));
        el.mAnom = eAnom - ecc * std::sin(eAnom);

        if (el.a > 0.0) {
            // 表头是 km：内部按 m 算，写出前换算。
            el.perAlt = (el.a * (1.0 - el.e) - RADIUS_EARTH) * 1.0e-3;
            el.apoAlt = (el.a * (1.0 + el.e) - RADIUS_EARTH) * 1.0e-3;
            el.period = kTwoPi * std::sqrt(el.a * el.a * el.a / EARTH_GM);
        }
        return el;
    }

    void writeORB(std::ostream &out, const Task &task, [[maybe_unused]] const Spec &spec, const std::vector<std::size_t> &idx) {
        out << "% 轨道根数（瞬根数）：ECEF 位置按 GMST 转到惯性系后解算，"
                "含近/远地点高度与周期\n";
        out << "% program   : gnssLab\n";
        CSV::Writer w(out);
        w.row("Wk", "SOW", "a/m", "e", "i/deg", "RAAN/deg", "argp/deg", "M/deg", "hp/km", "ha/km", "T/s");
        if (idx.size() >= 2) {
            std::vector<State> st(idx.size());
            Row row;
            for (std::size_t k = 0; k < idx.size(); ++k) {
                task.exportRow(idx[k], row);
                st[k].stamp = row.stamp;
                const double x = row.get(Field::X);
                const double y = row.get(Field::Y);
                const double z = row.get(Field::Z);
                st[k].ok = row.solved && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
                if (st[k].ok) st[k].pos = Eigen::Vector3d(x, y, z);
            }

            // 位置求导出速度：5 点 Lagrange 插值多项式求导，首末历元自动退化为单侧差分。
            for (std::size_t k = 0; k < st.size(); ++k) {
                if (!st[k].ok) continue;
                st[k].vel = velocityAt(st, k);
            }

            for (const State &s: st) {
                if (!s.ok || s.vel.squaredNorm() <= 0.0) continue;
                const CommonTime ct = WeekSecond(s.stamp.week, s.stamp.tow).toCommonTime();
                const DCM eciToEcef = DCM::fromECIToECEF(ct);
                const Eigen::Vector3d rEci = eciToEcef.transpose() * s.pos;
                const Eigen::Vector3d omega(0.0, 0.0, OMEGA_EARTH);
                const Eigen::Vector3d vEci = eciToEcef.transpose() * s.vel + omega.cross(rEci);
                const Elements el = elementsFromState(rEci, vEci);
                // 脱离半径/速度门限的点解不出椭圆，直接不留这行（写 0 会被误读成真实根数）。
                if (!(el.a > 0.0)) continue;

                w.cell(s.stamp.week);
                w.cell(CSV::fx(s.stamp.tow, 3));
                w.cell(CSV::fx(el.a, 3));
                w.cell(CSV::fx(el.e, 8));
                w.cell(CSV::fx(deg(el.inc), 6));
                w.cell(CSV::fx(deg(el.raan), 6));
                w.cell(CSV::fx(deg(el.argp), 6));
                w.cell(CSV::fx(deg(el.mAnom), 6));
                w.cell(CSV::fx(el.perAlt, 3));
                w.cell(CSV::fx(el.apoAlt, 3));
                w.cell(CSV::fx(el.period, 3));
                w.endRow();
            }
        }
    }
}
