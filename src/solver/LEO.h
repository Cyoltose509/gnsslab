#pragma once

#include "solver/ppp/Kinematic.h"
#include "core/SatID.h"
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <Eigen/Eigen>


class LEO : public PPP::Kinematic {
public:
    LEO() {
        setTropo(false);
        mReanchorEveryEpoch = true;
        mMotionThresh = 100.0;

        ekf.clockInitSigma = 60.0;
        mConfig.firstEpochPosSigma = 1.0;
        mConfig.ambProcSigma = std::sqrt(1.0E-7); // 旧方差口径默认值 1e-7 的精确开方（σ 口径）
        mConfig.phaseSigma = 0.003;
        mConfig.minSatsForUpdate = 4;
        mApplySolidTide = false;
        mPinInitHeight = false;
        mUseCycleSlip = false;
    }

    ~LEO() override = default;


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

    /// 取值编排在 app 层，这里只管接过来用；缺的段落按 0 处理并落日志。
    void setAntennaOffset(const AntennaOffset &offset, const std::string &source);

    /// 接管另一份同配置解算器已读好的产品，外加天线改正（后向趟免二次读盘）。
    void adoptProducts(SPP &src) override;

    bool processEpoch(ObsData &obs) override;

    void applyOptions(const SolverOptions &o) override;

protected:
    Eigen::Vector3d predictPosition(double dt, const CommonTime &epoch) const override;

    void buildEquSys(ObsData &obsData, const Eigen::Vector3d &xyzEst, VariableDataMap &csData, bool firstIter) override;

    Eigen::Vector3d recvAntennaOffsetECEF(const Eigen::Vector3d &xyzEst, const CommonTime &epoch,
                                          const SatID &sat) const override;

    // 首历元引导
    void bootstrapFirstEpoch(ObsData &obsData) override;

private:
    AntennaOffset mAnt; // 质心→ARP 与 ARP→APC，机体系
    std::string mAntSrc; // 两段各自的出处

    mutable Eigen::Vector3d mLastGoodAnchor = Eigen::Vector3d::Zero(); // 质量门控重锚定：最近一个可信锚点
    mutable bool mSppGood = true; // 本历元 SPP 重锚定质量是否达标
    mutable bool mGeomWeak = false; // 本历元码观测几何不足以定轨
    mutable double mEpochPdop = 0.0; // 本历元 SPP 几何 PDOP
    bool mPrevEpochSolved = true; // 上一历元是否发布了位置解

    // 线性化点外推用的运动状态：全部由相邻历元解算结果差分得到，不含力模型
    Eigen::Vector3d mPrevVel = Eigen::Vector3d::Zero();
    Eigen::Vector3d mRecvAcc = Eigen::Vector3d::Zero();
    CommonTime mPrevVelTime{};
    bool mHasPrevVel = false;
};
