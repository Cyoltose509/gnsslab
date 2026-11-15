#pragma once

#include "product/sp3/Reader.h"
#include "product/CLKReader.h"
#include "product/ATXReader.h"
#include "product/OSBBias.h"
#include "product/ERPReader.h"
#include "product/TROReader.h"
#include "product/ATTReader.h"
#include "product/IONEXReader.h"
#include "core/geom/CoordStruct.h"
#include "core/geom/EphemerisTable.h"
#include "TropoGrid.h"

#include <Eigen/Eigen>
#include <string>

/// 接收机天线改正的两段（机体系 m）：质心→天线参考点 ARP、ARP→天线相位中心（PCO）。
/// 两段来源不同（产品头/任务表 与 天线标定），分开记才知道缺的是哪一段；进观测方程的是合成矢量。
struct AntennaOffset {
    Eigen::Vector3d comToArp = Eigen::Vector3d::Zero();
    Eigen::Vector3d pco = Eigen::Vector3d::Zero();
    bool hasArp = false;
    bool hasPco = false;

    [[nodiscard]] bool any() const { return hasArp || hasPco; }

    [[nodiscard]] Eigen::Vector3d total() const { return comToArp + pco; }
};

/// 解算的输入侧：星历、对流层格网、精密产品与接收机天线偏置，全部由外部读好后交给解算器。
/// 一次运行只有一份，正序与倒序解算器共用——倒序趟因此不必重读盘，也不会有谁被搬空。
struct Products {
    EphemerisTable eph;
    TropoGrid tropo;

    SP3::Reader sp3;
    CLKReader clk;
    ATXReader atx;
    OSBBias osb;
    ERPReader erp;
    TROReader tro;
    ATTReader att;
    IONEXReader ion;

    /// SP3 头里的参考框架，解继承产品的框架；缺省时按 IGS14
    Frame::Ref orbitFrame = Frame::Ref::IGS14;
    bool hasOrbitFrame = false;

    AntennaOffset recvAnt;
    std::string recvAntSrc;
};
