#pragma once
#include "GnssStruct.h"
#include "FreqCombo.h"
#include "Geodesy.h"
#include <map>
#include <memory>
#include <string>
#include <vector>


struct PCVGrid {
    double zen1 = 0.0, zen2 = 90.0, dzen = 5.0, dazi = 0.0;
    std::vector<double> noazi;
    std::vector<std::vector<double> > azi;

    [[nodiscard]] bool valid() const { return !noazi.empty(); }

    [[nodiscard]] double value(double zenDeg, double azDeg) const;
};

class ATXReader {
public:
    bool read(const std::string &path);

    // 校验 ANTEX 文件格式与完整性（下载 / 手工导入时用，不进逐历元路径）。
    // 返回 false 时 detail 写原因，返回 true 时写人读摘要。
    static bool validate(const std::string &path, std::string &detail);

    [[nodiscard]] bool empty() const { return !tab || tab->empty(); }


    [[nodiscard]] Eigen::Vector3d getSatPCO(const SatID &sat, const std::string &band, const CommonTime &t) const;


    [[nodiscard]] Eigen::Vector3d getRcvPCOENU(const std::string &rcvType, const CommonTime &t) const;

    /// 把天线相位中心改正加到卫星坐标上：本体姿态优先用 ORBEX 四元数（satBody），
    /// 没有 ATT 时退回由太阳方向推的名义偏航。
    void applySatPCO(const SatID &sat, Eigen::Vector3d &pTx, const CommonTime &epoch,
                     const FreqCombo &def, const Eigen::Matrix3d *satBody = nullptr) const;


    [[nodiscard]] double rcvPCVIF(const std::string &rcvType, char sys, const FreqCombo &def,
                    double zenDeg, double azDeg, const CommonTime &t) const;


    [[nodiscard]] double satPCVIF(const SatID &sat, const FreqCombo &def, double nadirDeg, const CommonTime &t) const;

private:
    // 一个「时段块」：IGS20 起同一副天线按 VALID FROM/UNTIL 分段给出标定，
    // 必须按观测历元挑块，不能见到同一天线就覆盖（否则静默用错时段，厘米级偏差）。
    struct Entry {
        double fromMjd = -1.0e9;
        double untilMjd = 1.0e9;
        Eigen::Vector3d pco = Eigen::Vector3d::Zero();
        PCVGrid grid;
    };

    // 频点 → 该频点下的各时段块；接收机 PCO 不分频点，键为 ""。
    using BandEntries = std::map<std::string, std::vector<Entry> >;

    // 整份 ANTEX 的只读表。ANTEX 是版本型产品（一副天线用几年），同一份文件在一次
    // 会话里只解析一次，之后各 solver 实例共享这一份（IoCache 按 path+mtime+size 认）。
    struct Tables {
        std::map<SatID, BandEntries> satEnt;
        std::map<std::string, BandEntries> rcvEnt;

        [[nodiscard]] bool empty() const { return satEnt.empty() && rcvEnt.empty(); }
    };

    std::shared_ptr<const Tables> tab;

    static const Entry *pick(const std::vector<Entry> &v, double mjd);

    // 某天线在某频点、某历元下的 PCV 值（该频点无标定时为 0）。
    static double pcvAt(const BandEntries &tbl, const std::string &band,
                        double zenDeg, double azDeg, double mjd);
};
