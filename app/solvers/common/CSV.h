#pragma once

#include <ostream>

#include "Config.h"
#include "CoordConvert.h"
#include "util/CSV.h"
#include "Epoch.h"

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

    inline void exportCSVSpp(std::ostream &out, const std::vector<Epoch> &epochs,
                             const Eigen::Vector3d &ref) {
        CSV::Writer w;
        w.row("Wk", "SOW", "ECEF-X/m", "ECEF-Y/m", "ECEF-Z/m",
              "REF-X/m", "REF-Y/m", "REF-Z/m", "EAST/m", "NORTH/m", "UP/m",
              "B/deg", "L/deg", "H/m", "VX/m", "VY/m", "VZ/m",
              "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "SigmaP", "SigmaV",
              "SatCount", "3dRMS/m").endRow();
        const int cols = w.colCount();
        for (const auto &r: epochs) {
            if (!r.solved) {
                csvInvalidRow(w, r, cols);
                continue;
            }
            const auto &res = r.sppResult;
            const ENU enu = Coord::XYZToENU(res.xyz, ref);
            const double rms3d = std::sqrt(enu.E() * enu.E() + enu.N() * enu.N() + enu.U() * enu.U());
            w.row(r.week, CSV::fx(r.sow, 3),
                  CSV::f(res.xyz[0], 11), CSV::f(res.xyz[1], 11), CSV::f(res.xyz[2], 11),
                  CSV::f(ref.x(), 11), CSV::f(ref.y(), 11), CSV::f(ref.z(), 11),
                  CSV::f(enu.E(), 6), CSV::f(enu.N(), 6), CSV::f(enu.U(), 6),
                  CSV::f(res.blh[0] * RAD_TO_DEG, 10), CSV::f(res.blh[1] * RAD_TO_DEG, 10), CSV::f(res.blh[2], 4),
                  CSV::f(res.vel[0], 6), CSV::f(res.vel[1], 6), CSV::f(res.vel[2], 6),
                  CSV::f(res.pdop, 4), CSV::f(res.gdop, 4), CSV::f(res.hdop, 4), CSV::f(res.vdop, 4), CSV::f(res.tdop, 4),
                  CSV::f(res.sigmaP, 4), CSV::f(res.sigmaV, 4),
                  r.numSatsResult,
                  CSV::f(rms3d, 6));
        }
        out << w.str();
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

    inline void exportCSVPpp(std::ostream &out, const std::vector<Epoch> &epochs,
                             const Eigen::Vector3d &ref) {
        CSV::Writer w;
        w.row("Wk", "SOW", "Status",
              "Float-X/m", "Float-Y/m", "Float-Z/m", "Float-E/m", "Float-N/m", "Float-U/m", "Float-3dRMS/m", "Float-SigmaP/m",
              "Fix-X/m", "Fix-Y/m", "Fix-Z/m", "Fix-E/m", "Fix-N/m", "Fix-U/m", "Fix-3dRMS/m", "Fix-SigmaP/m",
              "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "Ratio", "SatCount",
              "REF-X/m", "REF-Y/m", "REF-Z/m", "Vel-X/m", "Vel-Y/m", "Vel-Z/m").endRow();
        const int cols = w.colCount();
        for (const auto &r: epochs) {
            if (!r.solved) {
                csvInvalidRow(w, r, cols);
                continue;
            }
            const auto &res = r.sppResult;
            const bool fixed = res.stat == 1;
            const char *statStr = csvStatStr(res.stat);
            const Eigen::Vector3d fixE = fixed ? res.xyzFixed : Eigen::Vector3d::Zero();
            w.cell(r.week).cell(CSV::fx(r.sow, 3)).cell(statStr);
            csvWriteSolBlock(w, res.xyzFloat, ref, res.sigmaPFloat, false);
            csvWriteSolBlock(w, fixE, ref, res.sigmaPFixed, !fixed);
            csvDopCells(w, res)
                    .cell(res.ratio).cell(r.numSatsResult);
            w.cell(CSV::f(ref.x(), 11)).cell(CSV::f(ref.y(), 11)).cell(CSV::f(ref.z(), 11))
                    .cell(CSV::f(res.vel[0], 6)).cell(CSV::f(res.vel[1], 6)).cell(CSV::f(res.vel[2], 6));
            w.endRow();
        }
        out << w.str();
    }

    inline void exportCSVLeo(std::ostream &out, const std::vector<Epoch> &epochs,
                             const Eigen::Vector3d &ref, const bool hasRefOrbit) {
        CSV::Writer w;
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
                csvInvalidRow(w, r, cols);
                continue;
            }
            const auto &res = r.sppResult;
            const bool fixed = res.stat == 1;
            const char *statStr = csvStatStr(res.stat);
            const Eigen::Vector3d fixE = fixed ? res.xyzFixed : Eigen::Vector3d::Zero();
            const Eigen::Vector3d &refRow =
                    hasRefOrbit && r.refECEF.squaredNorm() > 1.0 ? r.refECEF : ref;
            w.cell(r.week).cell(CSV::fx(r.sow, 3)).cell(statStr);
            csvWriteSolBlock(w, res.xyzFloat, refRow, res.sigmaPFloat, false);
            csvWriteSolBlock(w, fixE, refRow, res.sigmaPFixed, !fixed);
            csvDopCells(w, res)
                    .cell(res.ratio).cell(r.numSatsResult);
            w.cell(CSV::f(refRow.x(), 11)).cell(CSV::f(refRow.y(), 11)).cell(CSV::f(refRow.z(), 11));
            if (hasRefOrbit)
                w.cell(CSV::f(r.rtnR, 6)).cell(CSV::f(r.rtnT, 6)).cell(CSV::f(r.rtnN, 6));
            w.cell(CSV::f(res.vel[0], 6)).cell(CSV::f(res.vel[1], 6)).cell(CSV::f(res.vel[2], 6));
            w.endRow();
        }
        out << w.str();
    }

    inline void exportCSVRtk(std::ostream &out, const std::vector<Epoch> &epochs,
                             const Eigen::Vector3d &ref) {
        CSV::Writer w;
        w.row("Wk", "SOW", "Status",
              "Float-X/m", "Float-Y/m", "Float-Z/m", "Float-E/m", "Float-N/m", "Float-U/m", "Float-3dRMS/m", "Float-SigmaP/m",
              "Fix-X/m", "Fix-Y/m", "Fix-Z/m", "Fix-E/m", "Fix-N/m", "Fix-U/m", "Fix-3dRMS/m", "Fix-SigmaP/m",
              "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "Ratio", "NSats",
              "REF-X/m", "REF-Y/m", "REF-Z/m").endRow();
        const int cols = w.colCount();
        for (const auto &r: epochs) {
            if (!r.solved) {
                csvInvalidRow(w, r, cols);
                continue;
            }
            const auto &res = r.sppResult;
            const bool fixed = res.stat == 1;
            const char *statStr = csvStatStr(res.stat);
            const Eigen::Vector3d base(ref.x(), ref.y(), ref.z());
            const Eigen::Vector3d floatE = r.rtkFloatXyz.squaredNorm() > 1.0
                                               ? r.rtkFloatXyz
                                               : base + r.rtkBaseFloat;
            const Eigen::Vector3d fixE = fixed ? res.xyz : Eigen::Vector3d::Zero();
            w.cell(r.week).cell(CSV::fx(r.sow, 3)).cell(statStr);
            csvWriteSolBlock(w, floatE, ref, res.sigmaPFloat, false);
            csvWriteSolBlock(w, fixE, ref, res.sigmaPFixed, !fixed);
            w.cell(CSV::f(res.pdop, 4)).cell(CSV::f(res.gdop, 4)).cell(CSV::f(res.hdop, 4)).cell(CSV::f(res.vdop, 4)).cell(
                        CSV::f(res.tdop, 4))
                    .cell(r.rtkRatio).cell(r.numSatsResult);
            w.cell(CSV::f(ref.x(), 11)).cell(CSV::f(ref.y(), 11)).cell(CSV::f(ref.z(), 11));
            w.endRow();
        }
        out << w.str();
    }

    inline void exportCSV(std::ostream &out, const std::vector<Epoch> &epochs,
                          const Mode mode, const Eigen::Vector3d &ref, const bool hasRefOrbit) {
        switch (mode) {
            case Mode::PPP: exportCSVPpp(out, epochs, ref);
                break;
            case Mode::LEO: exportCSVLeo(out, epochs, ref, hasRefOrbit);
                break;
            case Mode::RTK: exportCSVRtk(out, epochs, ref);
                break;
            default: exportCSVSpp(out, epochs, ref);
                break;
        }
    }
}
