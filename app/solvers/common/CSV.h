#pragma once

#include <ostream>

#include "CoordConvert.h"
#include "Core.h"
#include "util/CSV.h"

namespace Solvers {
    inline const char *csvStatStr(const int stat) {
        return stat == 1 ? "Fixed" : stat == 2 ? "Float" : "Invalid";
    }

    inline CSV::Writer &csvDopCells(CSV::Writer &w, const Result &res) {
        w.cell(CSV::f(res.pdop, 4)).cell(CSV::f(res.gdop, 4)).cell(CSV::f(res.hdop, 4))
                .cell(CSV::f(res.vdop, 4)).cell(CSV::f(res.tdop, 4));
        return w;
    }

    inline void csvInvalidRow(CSV::Writer &w, const Epoch &r, const int cols) {
        w.cell(r.week).cell(CSV::fx(r.sow, 3)).cell("Invalid");
        for (int i = 3; i < cols; ++i) w.cell("");
        w.endRow();
    }

    inline void csvWriteSolBlock(CSV::Writer &w, const Eigen::Vector3d &ecef,
                                 const Eigen::Vector3d &ref, const double sigma, const bool skip) {
        if (skip) {
            for (int i = 0; i < 8; ++i) w.cell("");
            return;
        }
        const ENU enu = Coord::XYZToENU(ecef, ref);
        const double rms = std::sqrt(enu.E() * enu.E() + enu.N() * enu.N() + enu.U() * enu.U());
        w.cell(CSV::f(ecef.x(), 11)).cell(CSV::f(ecef.y(), 11)).cell(CSV::f(ecef.z(), 11))
                .cell(CSV::f(enu.E(), 6)).cell(CSV::f(enu.N(), 6)).cell(CSV::f(enu.U(), 6))
                .cell(CSV::f(rms, 6)).cell(CSV::f(sigma, 4));
    }
}
