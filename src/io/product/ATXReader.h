#pragma once
#include "core/SatID.h"
#include "core/TimeStruct.h"
#include "core/obs/FreqCombo.h"
#include "core/geom/Geodesy.h"
#include <map>
#include <memory>
#include <string>
#include <vector>

/// 天线相位中心表（igs20.atx）：按卫星/接收机型号存 PCO 与天顶角-方位角格网 PCV。
class ATXReader {
public:
    /// 一块天线的 PCV 格网：天顶角等间距，方位角按列分块；只有天顶角那一列时 azi 为空。
    struct PCVGrid {
        double zen1 = 0.0, zen2 = 90.0, dzen = 5.0, dazi = 0.0;
        std::vector<double> noazi;
        std::vector<std::vector<double> > azi;

        [[nodiscard]] bool valid() const { return !noazi.empty(); }

        [[nodiscard]] double value(double zenDeg, double azDeg) const;
    };

    bool read(const std::string &path);

    static bool validate(const std::string &path, std::string &detail);

    [[nodiscard]] bool empty() const { return !tab || tab->empty(); }

    [[nodiscard]] bool coversReceiver(const std::string &antType) const;

    [[nodiscard]] std::vector<std::string> receiverTypes() const;

    [[nodiscard]] Eigen::Vector3d getSatPCO(const SatID &sat, const std::string &band, const CommonTime &t) const;

    [[nodiscard]] Eigen::Vector3d getRcvPCOENU(const std::string &rcvType, char sys,
                                               const FreqCombo &def, const CommonTime &t) const;

    void applySatPCO(const SatID &sat, Eigen::Vector3d &pTx, const CommonTime &epoch,
                     const FreqCombo &def, const DCM *satBody = nullptr) const;

    [[nodiscard]] double rcvPCVIF(const std::string &rcvType, char sys, const FreqCombo &def,
                                  double zenDeg, double azDeg, const CommonTime &t) const;

    [[nodiscard]] double satPCVIF(const SatID &sat, const FreqCombo &def, double nadirDeg, const CommonTime &t) const;

private:
    struct Entry {
        double fromMjd = -1.0e9;
        double untilMjd = 1.0e9;
        Eigen::Vector3d pco = Eigen::Vector3d::Zero();
        PCVGrid grid;
    };

    using BandEntries = std::map<std::string, std::vector<Entry> >;

    struct Tables {
        std::map<SatID, BandEntries> satEnt;
        std::map<std::string, BandEntries> rcvEnt;

        [[nodiscard]] bool empty() const { return satEnt.empty() && rcvEnt.empty(); }
    };

    std::shared_ptr<const Tables> tab;

    static const Entry *pick(const std::vector<Entry> &v, double mjd);

    static Eigen::Vector3d pcoAt(const BandEntries &tbl, const std::string &band, double mjd);

    static double pcvAt(const BandEntries &tbl, const std::string &band,
                        double zenDeg, double azDeg, double mjd);
};
