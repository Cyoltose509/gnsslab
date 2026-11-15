#include <cmath>
#include <iomanip>
#include <ostream>
#include <string>

#include "core/Const.h"
#include "core/geom/CoordConvert.h"
#include "Core.h"
#include "solvers/common/Task.h"

namespace Solvers::Export {
    namespace {
        /// RTKLIB 的质量指示：1 固定、2 浮点、5 单点、6 PPP、-1 无效。
        int qualityOf(const Row &row, const Mode mode) {
            if (!row.solved) return -1;
            if (row.stat == 1) return 1;
            if (mode == Mode::SPP || mode == Mode::INS) return 5;
            if (mode == Mode::RTK) return 2;
            return 6;
        }

        double finiteOr0(const double v) { return std::isfinite(v) ? v : 0.0; }
    }

    void writePOS(std::ostream &out, const Task &task, const Spec &spec, const std::vector<std::size_t> &idx) {
        const bool hasRef = task.refECEF.squaredNorm() > 1.0
                            && (task.truthReady() || task.mode == Mode::RTK);
        const PosForm form = spec.posForm == PosForm::ENU && !hasRef ? PosForm::BLH : spec.posForm;
        const bool vel = spec.posVelocity;

        if (spec.posHeader) {
            out << "% program   : gnssLab\n";
            out << "% inp file  : " << (task.fileName.empty() ? std::string("-") : task.fileName) << "\n";
            if (hasRef) {
                const Eigen::Vector3d blh = Coord::XYZToBLH(task.refECEF, Frame::WGS84);
                out << "% (lat/lon)  : " << std::fixed << std::setprecision(9)
                        << blh[0] * RAD_TO_DEG << ' ' << blh[1] * RAD_TO_DEG << '\n';
            }
            out << "% (Date) GPST ";
            switch (form) {
                case PosForm::ECEF: out << "x-ecef(m) y-ecef(m) z-ecef(m)";
                    break;
                case PosForm::ENU: out << "e-baseline(m) n-baseline(m) u-baseline(m)";
                    break;
                default: out << "latitude(deg) longitude(deg) height(m)";
                    break;
            }
            out << " Q ns sdn(m) sde(m) sdu(m) sdne(m) sdeu(m) sdun(m) age(s) ratio";
            if (vel) {
                switch (form) {
                    case PosForm::ECEF: out << " vx(m/s) vy(m/s) vz(m/s)";
                        break;
                    default: out << " vn(m/s) ve(m/s) vu(m/s)";
                        break;
                }
            }
            out << '\n';
        }

        Row row;
        for (const std::size_t i: idx) {
            task.exportRow(i, row);
            const CivilTime c = toCivil(row.stamp);
            out << civilText(c, '/', ' ', "");
            if (!row.solved) {
                out << "  0.0  0.0  0.0  -1  0"
                        << "  0.0  0.0  0.0  0.0  0.0  0.0  0.000  0.0";
                if (vel) out << "  0.0  0.0  0.0";
                out << '\n';
                continue;
            }
            const double se = row.get(Field::SigmaE);
            const double sn = row.get(Field::SigmaN);
            const double su = row.get(Field::SigmaU);
            const double sats = row.get(Field::Sats);
            const long long ns = std::isfinite(sats) ? static_cast<long long>(sats + 0.5) : 0;
            switch (form) {
                case PosForm::ECEF:
                    out << std::fixed << std::setprecision(4)
                            << ' ' << finiteOr0(row.get(Field::X))
                            << ' ' << finiteOr0(row.get(Field::Y))
                            << ' ' << finiteOr0(row.get(Field::Z));
                    break;
                case PosForm::ENU:
                    out << std::fixed << std::setprecision(4)
                            << ' ' << finiteOr0(row.get(Field::E))
                            << ' ' << finiteOr0(row.get(Field::N))
                            << ' ' << finiteOr0(row.get(Field::U));
                    break;
                default:
                    out << std::fixed << std::setprecision(9)
                            << ' ' << finiteOr0(row.get(Field::Lat))
                            << ' ' << finiteOr0(row.get(Field::Lon))
                            << std::setprecision(4)
                            << ' ' << finiteOr0(row.get(Field::H));
                    break;
            }
            out << ' ' << qualityOf(row, task.mode)
                    << ' ' << ns
                    << std::setprecision(4)
                    << ' ' << finiteOr0(sn)
                    << ' ' << finiteOr0(se)
                    << ' ' << finiteOr0(su)
                    << " 0.0000 0.0000 0.0000 0.000"
                    << ' ' << finiteOr0(row.get(Field::Ratio));
            if (vel) {
                if (form == PosForm::ECEF) {
                    out << std::setprecision(4)
                            << ' ' << finiteOr0(row.get(Field::Vx))
                            << ' ' << finiteOr0(row.get(Field::Vy))
                            << ' ' << finiteOr0(row.get(Field::Vz));
                } else {
                    // .pos 的速度段是「北 东 天」，行里存的是 NED，天向要取反。
                    out << std::setprecision(4)
                            << ' ' << finiteOr0(row.get(Field::Vn))
                            << ' ' << finiteOr0(row.get(Field::Ve))
                            << ' ' << finiteOr0(-row.get(Field::Vd));
                }
            }
            out << '\n';
        }
    }
}
