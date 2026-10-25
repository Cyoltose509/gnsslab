#pragma once

#include "PPPKinematic.h"
#include "Sp3OrbitReader.h"
#include <cstdlib>
#include <Eigen/Eigen>


class LEO : public PPPKinematic {
public:
    LEO() {
        setTropo(false);
        mReanchorEveryEpoch = true;
        mMotionThresh = 100.0;

        // 每历元重初始化的钟差先验方差：对齐 RTKLIB ppp.c VAR_CLK=SQR(60.)=3600(σ=60m)。
        // 曾用 1e8(σ=10km)：H 的零方向 δ=(cdt+1, codeBias−1, 所有 B−1) 只被 10km 先验钉住，
        // 模糊度边际方差永不收敛（σ_amb≈0.045·√clockInitVar：1e8→14.5m，3600→2.6m，100→0.45m），
        // AR 门(arSigGate=5)把全部星挡在门外 ⇒ LAMBDA 从不运行。Swarm-C 24h A/B：
        // 3600 vs 1e8 连续弧段 3D RMS 0.740 vs 0.738m（+0.3%，无回归），再紧(100/10)才伤解
        // （钟差↔径向位置退化：0.842/1.795m）。
        ekf.clockInitVar = 3600.0;
        // [实验旋钮] GNSSLAB_LEO_CLKVAR 覆盖钟差先验方差（A/B 用，定稿后删除）。
        if (const char *e = std::getenv("GNSSLAB_LEO_CLKVAR")) ekf.clockInitVar = std::atof(e);
        // 首历元位置先验方差 1.0 m²
        mConfig.firstEpochPosVar = 1.0;
        mConfig.ambNoise = 1.0E-7;
        // 相位 sigma：LEO 走**运动学**滤波（8km/s），需更强观测权值让滤波跟得上动态，
        // 故必须比静态(SPP::Config 的 0.01273)更紧。实测(Swarm-C, sigIFCode=0.1)：
        // 0.003→全程 1.615m/tail 2.181m（最优）；0.006→1.729m；0.01273→2.631m；0.02→3.346m。
        // 勿"统一"成静态最优值——运动学与静态的最优权值不同。
        mConfig.sigmaPhase = 0.003;
        mConfig.minSatsForUpdate = 4;
        // 星载接收机不随地壳形变，固体潮位移对 LEO 不适用（对地面站才需要）。
        mApplySolidTide = false;
        // 卫星天线 PCO **必须施加**：IGS/COD 的 SP3 轨道参考点是卫星质心(CoM)，而观测值参考
        // 天线相位中心(APC)，两者相差 ATX 里的卫星 PCO（GPS 可达 1~2 m）。这与接收机在地面
        // 还是星上无关。
        // 历史坑：2026-09-11 曾据「COD SP3 已在天线相位中心」把 LEO 的 satPCO 关掉并写进
        // 默认配置(leo.ini sat_pco=0)，该判断错误（地面 PPP 侧 2026-10-07 实测 sat_pco=0 会
        // 使偏差 21.3→28.9cm 变差）。2026-10-21 在 Swarm-C 24h 上重测：sat_pco 0→1 使
        // 3D RMS 1.321→1.078 m、>1m 占比 23.78%→2.10%、连续弧段 0.891→0.738 m。
        // 卫星 PCO 现为「ATX 已加载即自动施加」（无开关）：只要有天线文件就必然生效，勿需再设标志。
        // 首历元不用 σ=1cm 高程伪观测钉先验高度：LEO 的 RINEX 头位置是过期快照(8km/s 机动)，
        // 且无 ZTD 状态不存在「高度↔ZTD」退化。钉住会把矛盾泄进钟差(实测 LSQ 钟差 −5811km)。
        mPinInitHeight = false;
        // 周跳时**不**把模糊度硬钉回"码减相位"初值（forceValue）。LEO 以 8km/s 运动，
        // 模糊度必须能跟随动态演化；钉死会让它长期停在粗值上。
        // 实测(Swarm-C, sigIFCode=0.1)：开 mwSlip 全程 RMS 3.620m / tail 1.314m / U+0.754m；
        // 关 ⇒ 1.172m / 1.205m / U+0.633m（全程 3.1× 改善）。检测器本身仍跑（保留 newArc/Gap 判定），
        // 只是不再 forceValue。
        mUseCycleSlip = false;
    }

    ~LEO() override = default;


    bool loadAuxFile(const std::string &path);

    bool loadLeoReference(const std::string &path, const SatID &sat = SatID());

    bool processEpoch(ObsData &obs) override;

public:
    /// 参考轨道给的是质心 CoM，本类解的是天线相位中心 ARP。
    /// 评测方(harness_leo)需要同口径比较时用它把 CoM 真值转到 ARP；无辅助文件则返回原点。
    Eigen::Vector3d comToArpECEF(const CommonTime &epoch) const {
        return recvAntennaOffsetECEF(mRefPos, epoch);
    }

protected:
    Eigen::Vector3d predictPosition(double dt, const CommonTime &epoch) const override;

    // 星载 CoG→ARP 杠杆臂(ECEF)
    Eigen::Vector3d recvAntennaOffsetECEF(const Eigen::Vector3d &xyzEst, const CommonTime &epoch) const override;

    // 首历元引导
    void bootstrapFirstEpoch(ObsData &obsData) override;

private:
    Eigen::Vector3d mLeoAntOffset = Eigen::Vector3d::Zero(); // 机体杠杆臂
    bool mHasAux = false;
    Sp3OrbitReader mRefRdr; // 参考轨道
    SatID mRefSat; // 参考卫星
    bool mHasRef = false; // 是否加载了参考轨道

    mutable Eigen::Vector3d mLastGoodAnchor = Eigen::Vector3d::Zero(); // 质量门控重锚定：最近一个可信锚点
    mutable bool mSppGood = true; // 本历元 SPP 重锚定质量是否达标
};
