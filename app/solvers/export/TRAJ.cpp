
#include <iomanip>
#include <ostream>
#include <string>

#include "Core.h"
#include "solvers/common/Task.h"
#include "i18n/I18n.h"

namespace Solvers::Export {
    namespace {
        using F = Field;

        void col(std::ostream &out, const double v, const int width, const int prec) {
            if (!std::isfinite(v)) {
                out << std::setw(width) << "-";
                return;
            }
            out << std::setw(width) << std::fixed << std::setprecision(prec) << v;
        }
    }

    void writeTRAJ(std::ostream &out, const Task &task, [[maybe_unused]] const Spec &spec, const std::vector<std::size_t> &idx) {
        out << I18n::trc("export.traj.head") << '\n';
        out << I18n::trc("export.traj.units") << '\n';
        out << I18n::tr("export.traj.file", task.fileName.empty() ? std::string("-") : task.fileName) << '\n';
        out << std::setw(4) << "Wk" << std::setw(13) << "SOW"
                << std::setw(14) << "Lat" << std::setw(15) << "Lon" << std::setw(12) << "H"
                << std::setw(12) << "VN" << std::setw(12) << "VE" << std::setw(12) << "VD"
                << std::setw(11) << "Roll" << std::setw(11) << "Pitch" << std::setw(11) << "Heading"
                << std::setw(6) << "Zupt" << '\n';

        Row row;
        for (const std::size_t i: idx) {
            task.exportRow(i, row);
            if (!row.solved) continue;
            out << std::setw(4) << row.stamp.week;
            col(out, row.get(F::Tow), 13, 3);
            col(out, row.get(F::Lat), 14, 10);
            col(out, row.get(F::Lon), 15, 10);
            col(out, row.get(F::H), 12, 4);
            col(out, row.get(F::Vn), 12, 4);
            col(out, row.get(F::Ve), 12, 4);
            col(out, row.get(F::Vd), 12, 4);
            col(out, row.get(F::Roll), 11, 5);
            col(out, row.get(F::Pitch), 11, 5);
            col(out, row.get(F::Heading), 11, 5);
            const double z = row.get(F::ZuptFlag);
            out << std::setw(6) << (std::isfinite(z) && z > 0.5 ? "1" : "0") << '\n';
        }
    }
}
