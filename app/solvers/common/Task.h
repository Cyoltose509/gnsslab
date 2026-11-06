#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <iosfwd>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "Core.h"
#include "solvers/common/Realtime.h"

namespace Solvers {
    /// 解算任务的控制面
    struct Task : Core, std::enable_shared_from_this<Task> {
        /// 任务种类
        enum class Kind { Solver, Tool };

        virtual Kind kind() const { return Kind::Solver; }

        /// 配置页与结果页的 GUI 入口，实现见 solvers/view/TaskGui.cpp 与各模式 Panel.cpp
        virtual void renderConfigPanel() {}
        virtual void renderResult();

        /// 配置面板骨架：文件 / 实时 / 参数 三页
        void renderConfigTabs(const std::string &title,
                              const std::function<void()> &fileTab,
                              const std::function<void()> &onStart,
                              const std::function<void()> &realtimeTab = {});

        /// 标准文件解算启动：校验观测 → 写 PathMemory → 存参数 → 打日志 → startFile。
        void renderFileStart(const char *obsIniKey, bool hasProducts,
                             const std::function<void()> &onPersist = {},
                             const std::function<void()> &onLog = {});

        /// 「取消」按钮；INS 的配置页跳过 renderConfigTabs 自绘骨架，但仍复用这个按钮。
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

        /// 参考真值（ECEF/m）：SPP / PPP(静态) 配置页可勾选填写、剪贴板粘贴或联网获取，
        /// 用于精度评估（ENU 误差、CSV 真值列）。勾选时解算以它为参考点。
        bool useRefTruth = false;
        Eigen::Vector3d refXyz{0, 0, 0};

        /// 联网获取参考真值（WHU 数据中心 IGS SINEX）的异步状态。
        std::atomic<bool> refFetching{false};
        std::string refFetchStatus;   // 状态文案：获取中 / 获取到… / 未知站点 / 错误
        std::string refFetchFile;     // 用到的真值文件（如 IGS0OPSSNX_…_CRD.SNX）
        std::string refFetchStation;  // 提取到的 4 字符站点名
        std::mutex refFetchMtx;       // 保护 refXyz / refFetchStatus 的并发读写
        std::thread refFetchThread;

        /// 复位任务位、结果容器与呈现状态，并调 onReset() 清模式特有状态。
        void resetForSolve();

        void startFile();

        void startRealtime();

        /// 结果表格的表头与列顺序由各模式自己定；基类不导出（工具任务没有 CSV）。
        virtual void exportCSV(std::ostream &out) const;

        /// 等解算线程收尾（CLI 与 GUI 的 join 语义合一）。
        void waitDone();

        bool ok() const;

        /// 停止条件：stop 位、实时时限、历元上限。实时会话与文件解算共用。
        virtual bool shouldStop();

        /// 请求停止并唤醒所有等待方；GUI 取消、CLI 停止与析构都走这里。
        void cancel();

        /// 结果历元数是否已达 maxEpochs；单站队列、RTK 配对队列、实时会话共用同一判据。
        bool epochLimitReached();

        virtual ~Task() {
            cancel();
            if (worker.joinable()) worker.join();
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

        /// 读取失败：记错误、置读取结束并唤醒等待方；返回 -1 供调用方收尾。
        int failRead(const std::string &msg);

        /// 解算线程收尾：置位并唤醒等待方（停止、历元上限、异常三条路都要叫一次）。
        void finishSolving();

        /// 唤醒所有队列等待方；RTK 另有配对队列，覆盖后一并叫醒。
        virtual void wakeQueues();

    private:
        /// renderFileStart 的内部步骤：校验观测格式、落文件名、判断 RINEX。
        bool beginFileSolve(const std::string &obsPath);

        void fileRead();

        /// 观测文件 → obsQueue + 质量输入；按 isRINEX 分派到两条读法。
        int readObsToQueue();

        int readRINEXToQueue();

        int readNovAtelToQueue();

        /// 限速喂队列：Source 给一帧就压一帧，队列满就等解算侧腾位。
        int feedToQueue(Observer::Source &src);

        /// 数据源就绪：置位并叫醒等星历的解算线程。
        void markNavReady();

        /// 读取收尾：置 readDone 并唤醒两侧等待方。
        void finishRead();

        std::chrono::steady_clock::time_point tStart = std::chrono::steady_clock::now();
    };
}
