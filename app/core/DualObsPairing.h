#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "GnssStruct.h"
#include "EphemerisTable.h"

namespace SolveDriver {

// 双站观测流配对器：把"基准站"与"流动站"两路（文件 / OEM7 / 实时 socket 皆可）
// 按历元时间对齐成一对，供 RTK 双差解算消费。
//
// 设计要点：
//  * 只负责"产出配对好的 (base, rover, baseEph, roverEph)"，绝不接触 RTK 解算器——
//    解算器只处理已配对好的单历元（与 RTKLIB postpos/rtksvr 配对、rtkpos 只算单历元的分层一致）。
//  * 配对窗口 tol 由 IntervalEstimator 自适应给出（= 半个采样间隔），1Hz/5Hz/10Hz 通吃，
//    无需调用方告知码率；样本不足时 interval()=0 → tol=0 → 只推进不配对，绝不错配相邻历元。
//  * 丢弃语义：|Δ| > tol 时丢弃较旧的一路重读（双向推进），与 RTKLIB"对齐后再解算"一致。
//  * 两个 Puller 回调各自取本路"下一历元 + 星历快照"：文件/RINEX 用无参 parseRinexObs()，
//    OEM7/实时用 getNextEpoch() + ephSnapshot()。RINEX 的首跳加速（fastForwardToEpoch）由调用方
//    在构造 Puller 之前完成，本类不关心具体 IO 形态。
class DualObsPairing {
public:
    // 采样间隔估计器：观察某路历元间隔的众数（间隔抖动到 1ms 量级分散时退化为中位数），
    // 据此给出双站配对窗口 = 半个采样间隔。样本不足时 interval()=0 → pairWindow()=0，
    // 此时只推进不配对，绝不会把相邻历元错配。原独立类 EpochAligner 已并入本类。
    struct IntervalEstimator {
        static constexpr int kWindow = 32;           // 滑动窗口内的间隔样本数
        static constexpr int kMinSamples = 3;        // 少于此数视为未就绪
        static constexpr double kMinDelta = 1e-6;    // ≤此值视为重复历元，不计入
        static constexpr double kMaxSaneGap = 900.0; // ≥15min 视为中断/时间回绕，不计入

        void reset();
        /// 按时间顺序喂入该路读到的每个历元时间（秒）
        void observe(double epochSec);
        [[nodiscard]] double interval() const;
        [[nodiscard]] bool ready() const;
        static double pairWindow(const IntervalEstimator &a, const IntervalEstimator &b);

    private:
        bool mHave = false;
        double mLast = 0.0;
        std::vector<double> mDeltas;
    };

    using EphPtr = std::shared_ptr<EphemerisTable>;
    using Puller = std::function<bool(ObsData &out, EphPtr &eph)>;

    struct Pair {
        ObsData base, rover;
        EphPtr baseEph, roverEph;
    };

    enum class Action { Pair, AdvanceBase, AdvanceRover };

    DualObsPairing(Puller basePull, Puller roverPull);

    // 实时轮询模式（IO 为"轮询+超时+断连重连"，无法套用 next()）专用构造：
    // 不持 puller，仅用语义方法 observeBase/observeRover/pairWindow 完成配对判定。
    DualObsPairing() = default;

    // —— 实时轮询路径专用（不调用 next()）——
    // 对外只暴露 DualObsPairing：喂历元、取配对窗口，全程不出现 IntervalEstimator 名称。
    void observeBase(double epochSec);
    void observeRover(double epochSec);
    [[nodiscard]] double pairWindow() const;

    // 取下一对配对历元。返回 false 表示两路中任一路已耗尽（EOF / 断连）。
    bool next(Pair &out);

    // 配对判定（与 next() 内部完全一致）。供实时轮询路径复用：其 IO 是"轮询+超时+断连重连"，
    // 无法套用 next() 的"拉取即阻塞"模型，但仍应共用同一套判定逻辑，避免语义分叉。
    static Action classify(const CommonTime &baseEpoch, const CommonTime &roverEpoch, double tol);

    // 星历选择：基准站优先，回落流动站，最后回落基准站快照（保证非空指针）。
    // OEM7/实时两路各自携带内嵌星历；RINEX 路径由调用方直接传入全量 navTable（不经此处）。
    static EphemerisTable *selectEph(const EphPtr &baseEph, const EphPtr &roverEph);

private:
    bool ensurePrimed();
    bool stepBase();
    bool stepRover();

    Puller mBasePull, mRoverPull;
    IntervalEstimator mBaseIv, mRoverIv;
    ObsData mBase, mRover;
    EphPtr mBaseEph, mRoverEph;
    bool mHaveBase = false, mHaveRover = false;
};

} // namespace SolveDriver
