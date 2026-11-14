#include <cmath>
#include <cstring>
#include <ostream>
#include <vector>

#include "core/Const.h"
#include "Core.h"
#include "solvers/common/Task.h"

namespace Solvers::Export {
    namespace {
        using F = Field;

        void put(std::ostream &out, const double v) {
            const double d = std::isfinite(v) ? v : 0.0;
            char buf[sizeof(double)];
            std::memcpy(buf, &d, sizeof(double));
            out.write(buf, sizeof(double));
        }
    }

    void writeSBET(std::ostream &out, const Task &task, [[maybe_unused]] const Spec &spec, const std::vector<std::size_t> &idx) {
        static_assert(sizeof(double) == 8, "SBET 要求 8 字节双精度");
        Row row;
        for (const std::size_t i: idx) {
            task.exportRow(i, row);
            if (!row.solved) continue;
            const double lat = row.get(F::Lat);
            const double lon = row.get(F::Lon);
            if (!std::isfinite(lat) || !std::isfinite(lon)) continue;
            put(out, row.get(F::Tow));
            put(out, lat * DEG_TO_RAD);
            put(out, lon * DEG_TO_RAD);
            put(out, row.get(F::H));
            put(out, row.get(F::Vn));
            put(out, row.get(F::Ve));
            put(out, row.get(F::Vd));
            put(out, row.get(F::Roll) * DEG_TO_RAD);
            put(out, row.get(F::Pitch) * DEG_TO_RAD);
            put(out, row.get(F::Heading) * DEG_TO_RAD);
            put(out, 0.0); // 游移角：本解算用 NED 平台，游移角恒 0
            put(out, row.get(F::Axb));
            put(out, row.get(F::Ayb));
            put(out, row.get(F::Azb));
            put(out, row.get(F::Wxb));
            put(out, row.get(F::Wyb));
            put(out, row.get(F::Wzb));
        }
    }
}
