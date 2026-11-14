#pragma once

#include <atomic>
#include <functional>
#include <iosfwd>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "Core.h"
#include "solvers/export/Core.h"
#include "solvers/common/Realtime.h"
#include "util/CSV.h"
#include "util/Clock.h"

namespace Solvers {
    /// 解算任务的控制面
    struct Task : Core, std::enable_shared_from_this<Task> {
        /// 任务种类
        enum class Kind { Solver, Tool };

        virtual Kind kind() const { return Kind::Solver; }

        /// 配置页与结果页的 GUI 入口，实现见 solvers/view/TaskGui.cpp 与各模式 Panel.cpp
        virtual void renderConfigPanel() {}
        virtual void renderResult();

        /// 配置面板骨架：文件 / 实时 / 参数，参数页按调用方开关
        void renderConfigTabs(const std::string &title,
                              const std::function<void()> &fileTab,
                              const std::function<void()> &onStart,
                              const std::function<void()> &realtimeTab = {},
                              bool showParamTab = true);

        /// 标准文件解算启动：校验观测 → 写 PathMemory → 存参数 → 打日志 → startFile。
        void renderFileStart(const char *obsIniKey, bool hasProducts,
                             const std::function<void()> &onPersist = {},
                             const std::function<void()> &onLog = {});

        /// 「取消」按钮，由 renderConfigTabs 与 renderRealtimeConfig 共用。
        void renderCancelButton();

        /// 实时页：按 labels 逐个编辑 rtEndpoints，extraContent 追加自定义内容。
        void renderRealtimeConfig(const std::vector<std::string> &labels,
                                  const std::function<void()> &onStart,
                                  const std::function<void()> &extraContent = {});

        /// 面板配置序号：每个任务开一份独立的界面配置槽。
        static int nextCfgIndex();

        std::thread worker;

        int maxEpochs = 0;
        int realtimeSec = 0;

        int cfgIndex = nextCfgIndex();

        /// 联网获取参考真值的异步状态。后台线程只写这里的字段，truth 一律由界面线程改。
        /// 状态行的分色级别，与 refFetchStatus 成对写入。
        enum class RefStatusLevel { Pass, Warn, Fail };
        std::atomic<bool> refFetching{false};
        std::atomic<bool> refFetchCancel{false};   // 取消解算 / 关页 / 退出时置位：传输中止，结果不再套用
        std::atomic<int> refFetchPercent{-1};      // 当前文件的进度 0-100，-1 表示总量未知
        std::atomic<double> refFetchTotal{-1.0};   // 当前文件总字节数，-1 表示总量未知
        std::atomic<double> refFetchGot{0.0};      // 已收到字节数（含断点续传的部分）
        std::string refFetchCurrent;               // 当前正在下载的文件名
        std::string refFetchStatus;                // 状态文案：获取中 / 获取到… / 未知站点 / 错误
        RefStatusLevel refFetchLevel = RefStatusLevel::Fail; // Status 的分色，随 Status 一起写
        std::mutex refFetchMtx;                    // 保护 refFetchCurrent / Status / Level / Pending

        /// 后台取回、还没被界面取走的结果：界面层决定套不套用，取走后清空。
        struct RefFetchResult {
            bool ok = false;
            RefForm form = RefForm::None;      // Fixed=站点坐标，File=SP3 轨道路径
            Eigen::Vector3d xyz{0, 0, 0};
            std::string sp3Path;
            std::string obsPath;               // 发起时的观测文件：期间换过文件就不许套用
            Eigen::Vector3d prevXyz{0, 0, 0};  // 发起时的坐标：期间手改过就不覆盖
            std::string prevSp3Path;           // 同上，SP3 路径的版本
        };
        RefFetchResult refFetchPending;

        /// 获取线程：收尾由关页 / 退出 / 析构 join，界面上永远拿不到游离线程。
        std::thread refFetchThread;

        /// 复位任务位、结果容器与呈现状态，并调 onReset() 清模式特有状态。
        void resetForSolve();

        void startFile();

        void startRealtime();

        /// 按导出规格写出结果：格式、采样、元素三轴互不影响。
        void exportTo(std::ostream &out, const Export::Spec &spec) const;


        /// 导出侧能看到的历元数：GNSS 读 epochs，INS 读自己的导航序列。
        [[nodiscard]] virtual std::size_t exportCount() const;

        /// 第 i 个可导出历元的时刻。
        [[nodiscard]] virtual Export::Stamp exportStamp(std::size_t i) const;

        /// 导出文件默认名前缀（不含扩展名）：基类取观测文件名，RTK 覆写成「流动站-基准站」。
        [[nodiscard]] virtual std::string exportStem() const;

        /// 填满一行：没有的量保持 NaN，writer 会把该格留空。调用方负责持锁。
        virtual void exportRow(std::size_t i, Export::Row &row) const;

        /// 该历元导出用的参考基准（ECEF）：REF-XYZ / ENU / 3dRMS 都相对它。
        /// 返回零表示没有基准（无真值），这几列一律留空——别拿「首历元自己」伪装成参考坐标。
        [[nodiscard]] virtual Eigen::Vector3d exportRef(const Epoch &ep) const;

        /// 导出列集里要不要带「相对参考」的那几列（REF-XYZ / ENU / 3dRMS / RTN）。
        /// 无真值时列出来整列为空，不如不列（RTK 的基线列不受此判据，恒可写出）。
        [[nodiscard]] virtual bool exportRefReady() const;

        /// 该格式下本模式能写出的列（顺序即界面勾选顺序与默认表头顺序）；不支持就返回空。
        [[nodiscard]] virtual std::vector<Export::Col> exportFields(Export::Format fmt) const;

        /// 该格式下默认勾选的列：CSV 就是 `exportFields` 全集，其余格式不按列写，留空。
        [[nodiscard]] virtual std::vector<Export::Col> exportDefaults(Export::Format fmt) const;

        /// CSV 开头的元信息块（公共段；模式参数交 writeCsvParams）。
        void writeCSVMeta(CSV::Writer &w) const;

        /// 本模式的「参数配置」行：在公共参数之后续写，各模式在自己的 Task.cpp 里排。
        virtual void writeCsvParams(CSV::Meta &m) const {}

        /// 已产出的结果历元数；GNSS 读 epochs，INS 读自己的解算快照。
        [[nodiscard]] virtual std::size_t resultEpochCount() const;

        /// 等解算线程收尾（CLI 与 GUI 的 join 语义合一）。
        void waitDone();

        /// 停止条件：stop 位、实时时限、历元上限。实时会话与文件解算共用。
        virtual bool shouldStop();

        /// 请求停止并唤醒所有等待方；GUI 取消、CLI 停止与析构都走这里。
        void cancel();

        /// 结果历元数是否已达 maxEpochs；单站队列、RTK 配对队列、实时会话共用同一判据。
        bool epochLimitReached();

        virtual ~Task() {
            cancel();
            // 线程 lambda 自己持 shared_from_this()：最后一份引用可能就是它自己的，自 join 会抛
            if (worker.joinable()) {
                if (worker.get_id() == std::this_thread::get_id()) worker.detach();
                else worker.join();
            }
            if (refFetchThread.joinable()) {
                if (refFetchThread.get_id() == std::this_thread::get_id()) refFetchThread.detach();
                else refFetchThread.join();
            }
        }

    protected:
        /// 文件读取线程体；默认单站（读入 obsQueue + 质量输入），RTK 覆盖为双站配对管线。
        virtual void readBody();

        /// 解算线程体；单站模式走 SPP::Task 的骨架，RTK 自行覆盖，工具任务不走文件解算。
        virtual void solveBody();

        /// 实时会话工厂；不做实时的模式不覆盖。
        virtual std::unique_ptr<Realtime::Session> makeSession() { return nullptr; }

        /// 模式特有的复位（配对队列、求解器实例等）；resetForSolve 末尾调用。
        virtual void onReset() {}

        /// 该任务当前选定的广播星历：读观测前按它补 GLONASS 槽位，缺了会让频率落到 k=0。
        virtual std::string navPath() const;

        /// 读取失败：记错误、置读取结束并唤醒等待方；返回 -1 供调用方收尾。
        int failRead(const std::string &msg);

        /// 解算线程收尾：置位并唤醒等待方（停止、历元上限、异常三条路都要叫一次）。
        void finishSolving();

        /// 多普勒没凑够 4 颗的历元，用已收敛位置做 t±1 中心差分补速度。
        /// 必须先落完全部历元（要 t+1），只在收尾跑一次。
        void fillVelocityByDifference();

        /// 唤醒所有队列等待方；RTK 另有配对队列，覆盖后一并叫醒。
        virtual void wakeQueues();

        /// 数据源就绪：置位并叫醒等数据的解算线程。
        void markNavReady();

        /// 读取收尾：置 readDone 并唤醒两侧等待方。
        void finishRead();

    private:
        /// renderFileStart 的内部步骤：校验观测格式、落文件名、判断 RINEX。
        bool beginFileSolve(const std::string &obsPath);

        /// 观测文件 → obsQueue + 质量输入。格式分派在 ObsStream 里，这里不再分 RINEX/NovAtel。
        int readObsToQueue();

        /// 限速喂队列：Source 给一帧就压一帧，队列满就等解算侧腾位。
        int feedToQueue(Observer::Source &src);

        Clock::Point tStart = Clock::now();
    };
}
