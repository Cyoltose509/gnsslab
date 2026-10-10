#pragma once
// CSV 写出：逐字节对齐 app/widgets/solvers/GuiSolverPanel.cpp 的
// csvHeaderGeneric/csvRowGeneric、csvHeaderLeo/csvRowLeo、csvHeaderRtk/csvRowRtk。
// 改动 GUI 的 CSV 列时，这里必须同步。
#include <ostream>
#include <iomanip>
#include <cmath>
#include "SolverTask.h"     // GnssTask::EpochData / SolverMode
#include "CoordConvert.h"   // XYZtoENU
#include "CoordStruct.h"    // ENU / Vector3d / XYZ

namespace CliCsv {
    inline void csvRowTail(std::ostream &out, const GnssTask::EpochData &r) {
        const auto &res = r.sppResult;
        out << ',' << std::setprecision(10) << res.blh[0] * RAD_TO_DEG << ',' << res.blh[1] * RAD_TO_DEG << ','
            << std::setprecision(4) << res.blh[2] << ',' << std::setprecision(6) << res.vel[0] << ',' << res.vel[1] << ',' << res.vel[2]
            << ',' << std::setprecision(4) << res.pdop << ',' << res.gdop << ',' << res.hdop << ',' << res.vdop
            << ',' << res.tdop << ',' << res.sigmaP << ',' << res.sigmaV << ',' << r.numSatsResult;
    }

    inline void writeHeaderGeneric(std::ostream &out) {
        out << "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,REF-X/m,REF-Y/m,REF-Z/m,EAST/m,NORTH/m,UP/m,"
               "B/deg,L/deg,H/m,VX/m,VY/m,VZ/m,PDOP,GDOP,HDOP,VDOP,TDOP,SigmaP,SigmaV,SatCount,Stat,Ratio,"
               "FIXED-X/m,FIXED-Y/m,FIXED-Z/m\n";
    }
    inline void writeRowGeneric(std::ostream &out, const GnssTask::EpochData &r, const Vector3d &refECEF) {
        const auto &res = r.sppResult;
        out << r.week << ',' << r.sow << ',' << std::setprecision(11) << res.xyz[0] << ',' << res.xyz[1] << ',' << res.xyz[2];
        const ENU enu = XYZtoENU(res.xyz, refECEF);
        out << ',' << refECEF.x() << ',' << refECEF.y() << ',' << refECEF.z();
        out << ',' << enu.E() << ',' << enu.N() << ',' << enu.U();
        csvRowTail(out, r);
        out << ',' << static_cast<int>(res.stat) << ',' << res.ratio
            << ',' << std::setprecision(11) << res.xyzFixed[0] << ',' << res.xyzFixed[1] << ',' << res.xyzFixed[2];
    }

    inline void writeHeaderLeo(std::ostream &out, bool hasRefOrbit) {
        if (hasRefOrbit)
            out << "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,REF-X/m,REF-Y/m,REF-Z/m,R/m,T/m,N/m,"
                   "B/deg,L/deg,H/m,VX/m,VY/m,VZ/m,PDOP,GDOP,HDOP,VDOP,TDOP,SigmaP,SigmaV,SatCount\n";
        else
            out << "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,"
                   "B/deg,L/deg,H/m,VX/m,VY/m,VZ/m,PDOP,GDOP,HDOP,VDOP,TDOP,SigmaP,SigmaV,SatCount\n";
    }
    inline void writeRowLeo(std::ostream &out, const GnssTask::EpochData &r, bool hasRefOrbit) {
        const auto &res = r.sppResult;
        out << r.week << ',' << r.sow << ',' << std::setprecision(11) << res.xyz[0] << ',' << res.xyz[1] << ',' << res.xyz[2];
        if (hasRefOrbit) {
            out << ',' << r.refECEF[0] << ',' << r.refECEF[1] << ',' << r.refECEF[2];
            out << ',' << r.rtnR << ',' << r.rtnT << ',' << r.rtnN;
        }
        csvRowTail(out, r);
    }

    inline void writeHeaderRtk(std::ostream &out) {
        out << "Wk,SOW,STAT,Fix-X/m,Fix-Y/m,Fix-Z/m,Fix-E/m,Fix-N/m,Fix-U/m,"
               "Float-X/m,Float-Y/m,Float-Z/m,Float-E/m,Float-N/m,Float-U/m,Ratio,NSats\n";
    }
    inline void writeRowRtk(std::ostream &out, const GnssTask::EpochData &r, const Vector3d &refECEF) {
        const Vector3d ref(refECEF.x(), refECEF.y(), refECEF.z());
        const Vector3d blX = r.rtkBaseFloat;
        const ENU enuX = XYZtoENU(XYZ(ref.x() + blX.x(), ref.y() + blX.y(), ref.z() + blX.z()), refECEF);
        out << r.week << ',' << r.sow << ',' << std::setprecision(4) << r.rtkStat << ',';
        if (r.rtkStat == 1) {
            const Vector3d blF = r.sppResult.xyz - ref;
            const ENU enuF = XYZtoENU(XYZ(ref.x() + blF.x(), ref.y() + blF.y(), ref.z() + blF.z()), refECEF);
            out << blF.x() << ',' << blF.y() << ',' << blF.z() << ','
                << enuF.E() << ',' << enuF.N() << ',' << enuF.U() << ',';
        } else {
            out << ",,,,,,";
        }
        out << blX.x() << ',' << blX.y() << ',' << blX.z() << ','
            << enuX.E() << ',' << enuX.N() << ',' << enuX.U() << ','
            << r.rtkRatio << ',' << r.numSatsResult;
    }

    inline void writeHeader(std::ostream &out, GnssTask::SolverMode m, bool hasRefOrbit) {
        switch (m) {
            case GnssTask::SolverMode::Leo: writeHeaderLeo(out, hasRefOrbit); break;
            case GnssTask::SolverMode::Rtk: writeHeaderRtk(out); break;
            default: writeHeaderGeneric(out); break; // SPP / PPP
        }
    }
    inline void writeRow(std::ostream &out, const GnssTask::EpochData &r, GnssTask::SolverMode m,
                         const Vector3d &refECEF, bool hasRefOrbit) {
        switch (m) {
            case GnssTask::SolverMode::Leo: writeRowLeo(out, r, hasRefOrbit); break;
            case GnssTask::SolverMode::Rtk: writeRowRtk(out, r, refECEF); break;
            default: writeRowGeneric(out, r, refECEF); break;
        }
        out << '\n';
    }
}
