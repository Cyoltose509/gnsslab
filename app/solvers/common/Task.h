#pragma once

#include <atomic>
#include <functional>
#include <set>
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

struct Products;

namespace AppConfig {
    class ProductPaths;
}

namespace ProductReader {
    struct Loaded;
}

enum class ProductType;

namespace Solvers {
    /// 解算任务的控制面
    struct Task : Core, std::enable_shared_from_this<Task> {
        /// 任务种类
        enum class Kind { Solver, Tool };

        virtual Kind kind() const { return Kind::Solver; }


        virtual void renderConfigPanel() {
        }

        virtual void renderResult();


        void renderConfigTabs(const std::string &title,
                              const std::function<void()> &fileTab,
                              const std::function<void()> &onStart,
                              const std::function<void()> &realtimeTab = {},
                              bool showParamTab = true);


        void renderFileStart(const char *obsIniKey, bool hasProducts,
                             const std::function<void()> &onPersist = {},
                             const std::function<void()> &onLog = {});


        void renderCancelButton();


        void renderRealtimeConfig(const std::vector<std::string> &labels,
                                  const std::function<void()> &onStart,
                                  const std::function<void()> &extraContent = {});


        static int nextCfgIndex();

        std::thread worker;

        int maxEpochs = 0;
        int realtimeSec = 0;

        int cfgIndex = nextCfgIndex();


        enum class RefStatusLevel { Pass, Warn, Fail };

        std::atomic<bool> refFetching{false};
        std::atomic<bool> refFetchCancel{false};
        std::atomic<int> refFetchPercent{-1};
        std::atomic<double> refFetchTotal{-1.0};
        std::atomic<double> refFetchGot{0.0};
        std::string refFetchCurrent;
        std::string refFetchStatus;
        RefStatusLevel refFetchLevel = RefStatusLevel::Fail;
        std::mutex refFetchMtx;


        struct RefFetchResult {
            bool ok = false;
            RefForm form = RefForm::None;
            Eigen::Vector3d xyz{0, 0, 0};
            std::string sp3Path;
            std::string obsPath;
            Eigen::Vector3d prevXyz{0, 0, 0};
            std::string prevSp3Path;
        };

        RefFetchResult refFetchPending;


        std::thread refFetchThread;


        void resetForSolve();

        /// 产品缓存：同一批路径只在首次解算时读盘，改参数重算直接复用
        ProductReader::Loaded productsFor(const AppConfig::ProductPaths &d, const std::string &tag,
                                          const std::set<SatSystem> &systems,
                                          const std::vector<ProductType> &wanted);

        /// 没有轨道来源（精轨或广播星历）就没有解算输入：置错并返回 false
        bool requireOrbit(const std::string &sp3, const std::string &brdc);

        std::shared_ptr<::Products> prodCache;
        std::string prodCacheKey;
        bool prodCacheReadOk = true;

        void startFile();

        void startRealtime();


        void exportTo(std::ostream &out, const Export::Spec &spec) const;


        [[nodiscard]] virtual std::size_t exportCount() const;


        [[nodiscard]] virtual Export::Stamp exportStamp(std::size_t i) const;


        [[nodiscard]] virtual std::string exportStem() const;


        virtual void exportRow(std::size_t i, Export::Row &row) const;




        [[nodiscard]] std::vector<Export::Col> columnsForTruth(std::vector<Export::Col> cols) const;


        [[nodiscard]] virtual std::vector<Export::Col> exportFields(Export::Format fmt) const;


        [[nodiscard]] virtual std::vector<Export::Col> exportDefaults(Export::Format fmt) const;


        void writeCSVMeta(CSV::Writer &w) const;


        void writeCommonParams(CSV::Meta &m) const;


        virtual void writeCSVParams(CSV::Meta &m) const {
        }


        [[nodiscard]] virtual std::size_t resultEpochCount() const;


        void waitDone();


        virtual bool shouldStop();


        void cancel();


        bool epochLimitReached();

        virtual ~Task() {
            cancel();

            if (worker.joinable()) {
                if (worker.get_id() == std::this_thread::get_id()) worker.detach();
                else worker.join();
            }
            if (refFetchThread.joinable()) {
                if (refFetchThread.get_id() == std::this_thread::get_id()) refFetchThread.detach();
                else refFetchThread.join();
            }
        }

    public:
        virtual void readBody();


        virtual void solveBody();


        virtual std::unique_ptr<Realtime::Session> makeSession() { return nullptr; }


        virtual void onReset() {
        }


        virtual std::string navPath() const;


        int failRead(const std::string &msg);


        void finishSolving();


        virtual void wakeQueues();


        void markNavReady();


        void finishRead();

    private:
        bool beginFileSolve(const std::string &obsPath);


        int readObsToQueue();


        int feedToQueue(Observer::Source &src);

        Clock::Point tStart = Clock::now();
    };
}
