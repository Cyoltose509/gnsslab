#include "core/Application.h"
#include <windows.h>

#include "tools/GuiTimeConverter.h"
#include "tools/GuiCoordConverter.h"
#include "tools/GuiLsqSolver.h"
#include "tools/ins/GuiInsPanel.h"
#include "download/ui/Panel.h"
#include "download/remote/Earthdata.h"
#include "solvers/view/GuiSolverPanel.h"
#include "solvers/spp/Task.h"
#include "solvers/ppp_kinematic/Task.h"
#include "solvers/ppp_static/Task.h"
#include "solvers/ins/Task.h"
#include "solvers/leo/Task.h"
#include "solvers/rtk/Task.h"
#include "core/PathMemory.h"
#include "core/SiteTruth.h"
#include "io/ProductPaths.h"
#include "download/local/Store.h"
#include "download/remote/Catalog.h"
#include "download/remote/http/Core.h"
#include "io/ProductReader.h"
#include "ui/Markdown.h"
#include "EmbeddedReadme.h"
#include "version.h"
#include "Log.h"
#include "util/Clock.h"
#include "i18n/I18n.h"

#include "imgui.h"

namespace {
    using TaskList = std::vector<std::shared_ptr<Solvers::Task> >;

    /// 首次启动跟随系统界面语言：中文分简繁，日韩跟着走，其余一律先给英文，之后由用户自己选。
    const char *systemLanguageCode() {
        const LANGID id = GetUserDefaultUILanguage();
        if (PRIMARYLANGID(id) == LANG_CHINESE)
            return SUBLANGID(id) == SUBLANG_CHINESE_TRADITIONAL ? "ZH-TW" : "ZH-CN";
        if (PRIMARYLANGID(id) == LANG_JAPANESE) return "JA";
        if (PRIMARYLANGID(id) == LANG_KOREAN) return "KO";
        return "EN";
    }

    void stopAndJoinTasks(TaskList &tasks) {
        for (const auto &task: tasks) {
            task->cancel();   // 顺带置 refFetchCancel：获取线程也收手，不留退出期的游离线程
            if (task->view.qcWorker.joinable()) task->view.qcWorker.join();
        }
        for (const auto &task: tasks) {
            if (task->worker.joinable()) task->worker.join();
            if (task->refFetchThread.joinable()) task->refFetchThread.join();
        }
        tasks.clear();
    }

    /// 关掉的标签页要等解算和联网获取都收尾才真正释放：GET 还在跑时销毁，会用到已析构的 Http / 缓存。
    /// 只挑已经收尾的，还在跑的下帧再看；收尾与析构本身交给回收线程，界面线程不碰。
    bool isRetirable(const std::shared_ptr<Solvers::Task> &task) {
        const bool solving = task->worker.joinable() && !task->done.load() && !task->errorFlag();
        return !solving && !task->refFetching.load();
    }

    /// 站点真值的取法（GUI 走下载库缓存；CLI 不注册，只用本地已有文件）。
    void registerSiteTruth() {
        SiteTruth::fetcher() = [](const std::string &site, const int year, const int doy,
                                             double xyz[3], bool &fromCache, std::string &detail,
                                             const Download::Http::ProgressCb &progress) {
            return Download::Catalog::ensureSiteTruth(site, year, doy, xyz, fromCache, detail, progress);
        };
    }
}

const Application::ToolPanel Application::kToolPanels[3] = {
    {"tool.time.title", &Application::m_showTimeConverter, Tools::TimeConverter::Render},
    {"tool.coord.title", &Application::m_showCoordConverter, Tools::CoordConverter::Render},
    {"tool.lsq.title", &Application::m_showLsqSolver, Tools::LsqSolver::Render},
};

void Application::switchLanguage(const std::string_view code) {
    if (!I18n::setLanguage(code)) return;
    AppConfig::PathMemory::instance().set("gui.language", std::string(code));
    LOG_INFO << "[I18N] language -> " << I18n::language();
}

Application::Application() {
    registerSiteTruth();
}

void Application::Initialize() {
    Log::init("gnsslab.log");

    const auto &pm = AppConfig::PathMemory::instance();
    // 繁中/日/韩是本地机翻产物，没跑过 build/translate.py 时表里没有这几列；此时中文退回简中、其余退回英文。
    const std::string lang = pm.get("gui.language", systemLanguageCode());
    if (!I18n::setLanguage(lang)) I18n::setLanguage(lang.rfind("ZH", 0) == 0 ? "ZH-CN" : "EN");
    LOG_INFO << I18n::tr("log.app.start", PROJECT_VERSION, I18n::language());

    const std::string title = "GnssLab v" PROJECT_VERSION;

    const int winW = pm.getInt("win_w", 1280, 320, 7680);
    const int winH = pm.getInt("win_h", 720, 240, 4320);
    const bool winMax = pm.getInt("win_max", 0, 0, 1) != 0;
    m_ui.Initialize(title.c_str(), static_cast<float>(winW), static_cast<float>(winH), winMax);

    Download::Catalog::setUpdatesEnabled(pm.get("dl_autosync", pm.get("prod_autosync")) != "0");
    Download::Catalog::startUpdates();
    Download::Store::reclaimLegacyArchives();
    Download::Earthdata::load();
    startReaper();
}

void Application::Shutdown() {
    Download::Catalog::stopUpdates();

    {
        int w = 0, h = 0;
        m_ui.GetRestoredSize(w, h);
        const auto &pm = AppConfig::PathMemory::instance();
        if (w > 0 && h > 0) {
            pm.setInt("win_w", w);
            pm.setInt("win_h", h);
        }
        pm.setInt("win_max", m_ui.IsMaximized() ? 1 : 0);
    }

    AppConfig::PathMemory::instance().flush();
    I18n::reportMissing();
    LOG_INFO << I18n::tr("log.app.exit");
    stopAndJoinTasks(m_tasks);
    stopReaper();
    Download::Panel::state().shutdown();
    Download::Panel::shutdownEarthdata();
    m_ui.Shutdown();
}

void Application::Update() {
    // 关页时这里只做一次指针搬家：长任务的历元表、图表副本、QC 序列动辄几十万个堆块，
    // 在界面线程上析构会卡掉半秒以上，统一丢给回收线程。
    std::vector<std::shared_ptr<Solvers::Task> > ready;
    for (auto it = m_closingTasks.begin(); it != m_closingTasks.end();) {
        if (!isRetirable(*it)) {
            ++it;
            continue;
        }
        ready.push_back(std::move(*it));
        it = m_closingTasks.erase(it);
    }
    if (ready.empty()) return;
    {
        std::lock_guard lk(m_reapMutex);
        for (auto &task: ready) m_reapQueue.push_back(std::move(task));
    }
    m_reapCv.notify_one();
}

void Application::startReaper() {
    m_reaper = std::thread([this] {
        while (true) {
            std::vector<std::shared_ptr<Solvers::Task> > batch;
            {
                std::unique_lock lk(m_reapMutex);
                m_reapCv.wait(lk, [this] { return m_reapQuit || !m_reapQueue.empty(); });
                if (m_reapQueue.empty()) return;
                batch.swap(m_reapQueue);
            }
            const Clock::Stopwatch releaseCost;
            const std::size_t count = batch.size();
            for (const auto &task: batch) {
                if (task->view.qcWorker.joinable()) task->view.qcWorker.join();
                task->waitDone();
                if (task->refFetchThread.joinable()) task->refFetchThread.join();
            }
            batch.clear(); // 引用归零：历元、图表副本、QC 序列的堆内存在这一行放掉
            LOG_INFO << "[TASK] " << I18n::tr("app.log.reaped", count, releaseCost.ms());
        }
    });
}

void Application::stopReaper() {
    {
        std::lock_guard lk(m_reapMutex);
        // 退出时还在等解算收尾的一并交过去：join 由回收线程做，这里只等它清空。
        for (auto &task: m_closingTasks) {
            task->cancel();
            m_reapQueue.push_back(std::move(task));
        }
        m_closingTasks.clear();
        m_reapQuit = true;
    }
    m_reapCv.notify_all();
    if (m_reaper.joinable()) m_reaper.join();
}

void Application::RenderMenuBar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu(I18n::trc("gui.menu.task"))) {
            if (ImGui::MenuItem(I18n::trc("gui.menu.task.spp")))
                makeSppTask();
            if (ImGui::MenuItem(I18n::trc("gui.menu.task.ppp_static")))
                makePppStaticTask();
            if (ImGui::MenuItem(I18n::trc("gui.menu.task.ppp_kinematic")))
                makePppKinematicTask();
            if (ImGui::MenuItem(I18n::trc("gui.menu.task.rtk")))
                makeRtkTask();
            ImGui::Separator();
            if (ImGui::MenuItem(I18n::trc("gui.menu.task.leo")))
                makeLeoTask();
            ImGui::Separator();
            if (ImGui::MenuItem(I18n::trc("gui.menu.task.ins")))
                makeInsTask();
            ImGui::Separator();
            if (ImGui::MenuItem(I18n::trc("gui.menu.task.exit"), "Alt+F4"))
                m_quitRequested = true;
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu(I18n::trc("gui.menu.tools"))) {
            for (const auto &tool: kToolPanels)
                ImGui::MenuItem(I18n::trc(tool.menuKey), nullptr, &(this->*tool.open));
            if (ImGui::BeginMenu(I18n::trc("gui.menu.tools.ins"))) {
                if (ImGui::MenuItem(I18n::trc("gui.menu.tools.ins_calib"))) makeInsCalibTask();
                if (ImGui::MenuItem(I18n::trc("gui.menu.tools.ins_align"))) makeInsAlignTask();
                if (ImGui::MenuItem(I18n::trc("gui.menu.tools.ins_verify"))) makeInsVerifyTask();
                ImGui::EndMenu();
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu(I18n::trc("common.data"))) {
            ImGui::MenuItem(I18n::trc("gui.menu.data.download"), nullptr, &Download::Panel::state().open);
            ImGui::MenuItem(I18n::trc("gui.menu.data.earthdata"), nullptr, &m_showEarthdata);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu(I18n::trc("gui.menu.help"))) {
            if (ImGui::MenuItem(I18n::trc("gui.menu.help.about"), nullptr))
                m_showAbout = true;
            ImGui::EndMenu();
        }
        // 语言放在菜单栏末尾：切换是全局且立即生效的，藏进二级菜单里反而不容易找到。
        if (ImGui::BeginMenu(I18n::trc("gui.menu.language"))) {
            for (const auto &lang: I18n::languages()) {
                const bool active = lang.code == I18n::language();
                if (ImGui::MenuItem(lang.name.data(), nullptr, active) && !active)
                    switchLanguage(lang.code);
            }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    const char *aboutTitle = I18n::trc("gui.about.title");
    if (m_showAbout) {
        ImGui::SetNextWindowSize(ImVec2(760, 560), ImGuiCond_FirstUseEver);
        ImGui::OpenPopup(aboutTitle);
        m_showAbout = false;
    }

    if (ImGui::BeginPopupModal(aboutTitle, nullptr)) {
        ImGui::BeginChild("readme", ImVec2(0, -ImGui::GetFrameHeightWithSpacing()),
                          ImGuiChildFlags_Borders);
        const bool chinese = I18n::language().rfind("ZH", 0) == 0;
        UI::renderMarkdown(reinterpret_cast<const char *>(chinese ? UI::kEmbeddedReadmeZH : UI::kEmbeddedReadmeEN),
                           chinese ? UI::kEmbeddedReadmeZHLen : UI::kEmbeddedReadmeENLen);
        ImGui::EndChild();
        if (ImGui::Button(I18n::trc("gui.about.ok"), ImVec2(120, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

void Application::RenderTaskTabList() {
    auto &tasks = m_tasks;
    auto &closingTasks = m_closingTasks;
    int &taskToFocus = m_taskToFocus;

    for (int i = 0; i < static_cast<int>(tasks.size()); ++i) {
        auto &task = tasks[i];
        const bool isTool = task->kind() == Solvers::Task::Kind::Tool;
        const std::string tag = isTool ? std::string() : std::string(Solvers::modeLabel(task->mode));
        const std::string cat = isTool ? std::string() : "[" + tag + "] ";
        const bool isRealtime = task->isRealtime;
        const std::string defaultName = isTool ? I18n::tr("gui.tab.untitled") : tag + I18n::tr("common.solution");

        std::string label;
        if (task->state == Solvers::Task::State::Config)
            label = I18n::tr("gui.tab.config") + cat + (task->fileName.empty() ? defaultName : std::string(task->displayName()));
        else if (isRealtime)
            label = I18n::tr("gui.tab.realtime") + cat + task->displayName();
        else
            label = cat + task->displayName();

        if (task->state != Solvers::Task::State::Config && !isRealtime) {
            if (task->loading.load())
                label += " " + I18n::tr("gui.tab.loading");
            else if (task->errorFlag())
                label += " " + I18n::tr("gui.tab.error");
            else if (task->done.load())
                label += " " + I18n::tr("gui.tab.stopped");
        }

        // tab ID 必须绑任务自身（cfgIndex 进程内唯一），不能用列表下标：关掉前面的页后，
        // 后面的页会拿到别人用过的 ID，连它内部二级页的选中状态一起继承过来。
        label += "###task";
        label += std::to_string(task->cfgIndex);

        ImGuiTabItemFlags tabFlags = ImGuiTabItemFlags_None;
        if (i == taskToFocus) tabFlags |= ImGuiTabItemFlags_SetSelected;

        bool open = true;
        if (ImGui::BeginTabItem(label.c_str(), &open, tabFlags)) {
            if (taskToFocus == i) taskToFocus = -1;
            Solvers::SolverPanel::renderTask(task);
            ImGui::EndTabItem();
        }

        if (!open) {
            LOG_INFO << "[TASK] " << I18n::tr("app.log.close_tab", task->fileName);
            task->cancel();
            closingTasks.push_back(task);
            tasks.erase(tasks.begin() + i);

            if (taskToFocus > i) --taskToFocus;
            --i;
        }
    }
}

void Application::RenderTasks() {
    if (m_tasks.empty()) {
        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        const auto center = ImVec2(
            viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
            viewport->WorkPos.y + viewport->WorkSize.y * 0.5f
        );

        ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        constexpr ImGuiWindowFlags flags =
                ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

        if (ImGui::Begin("##empty_hint", nullptr, flags)) {
            ImGui::Text("GnssLab");
            ImGui::Spacing();
            ImGui::TextDisabled(I18n::trc("gui.empty.hint"));
        }
        ImGui::End();
        return;
    }


    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    constexpr ImGuiWindowFlags windowFlags =
            ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;


    if (ImGui::Begin("##task_host", nullptr, windowFlags)) {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 8.0f));

        if (ImGui::BeginTabBar("##tasks", ImGuiTabBarFlags_None)) {
            RenderTaskTabList();

            ImGui::EndTabBar();
        }

        ImGui::PopStyleVar();
    }
    ImGui::End();
}

void Application::attachTask(const std::shared_ptr<Solvers::Task> &task) {
    task->state = Solvers::Task::State::Config;
    task->loading = false;
    task->done = false;
    m_tasks.push_back(task);
    m_taskToFocus = static_cast<int>(m_tasks.size()) - 1;
}

void Application::makeSppTask() {
    const auto task = std::make_shared<Solvers::SPP::Task>();
    task->loadFromMemory();
    LOG_INFO << "[TASK] " << I18n::tr("app.log.new_task", I18n::tr("gui.menu.task.spp"), task->view.obsPathBuf);

    attachTask(task);
}

void Application::makePppStaticTask() {
    const auto task = std::make_shared<Solvers::PPPStatic::Task>();
    task->loadFromMemory();
    LOG_INFO << "[TASK] " << I18n::tr("app.log.new_task", I18n::tr("gui.menu.task.ppp_static"), task->view.obsPathBuf);

    attachTask(task);
}

void Application::makePppKinematicTask() {
    const auto task = std::make_shared<Solvers::PPPKinematic::Task>();
    task->loadFromMemory();
    LOG_INFO << "[TASK] " << I18n::tr("app.log.new_task", I18n::tr("gui.menu.task.ppp_kinematic"), task->view.obsPathBuf);

    attachTask(task);
}

void Application::makeLeoTask() {
    const auto task = std::make_shared<Solvers::LEO::Task>();
    task->loadFromMemory();
    LOG_INFO << "[TASK] " << I18n::tr("app.log.new_task", I18n::tr("gui.menu.task.leo"), task->view.obsPathBuf);

    attachTask(task);
}

void Application::makeRtkTask() {
    const auto task = std::make_shared<Solvers::RTK::Task>();
    task->loadFromMemory();
    LOG_INFO << "[TASK] " << I18n::tr("app.log.new_rtk", task->baseObsPathBuf, task->view.obsPathBuf);

    attachTask(task);
}

void Application::makeInsTask() {
    const auto task = std::make_shared<Solvers::INS::Task>();
    task->loadFromMemory();
    LOG_INFO << "[TASK] " << I18n::tr("app.log.new_ins", task->view.obsPathBuf);
    attachTask(task);
}

void Application::makeInsCalibTask() {
    const auto task = Tools::Ins::MakeSixPosCalibTask();
    LOG_INFO << "[TASK] " << I18n::tr("app.log.new_tool", I18n::tr("gui.menu.tools.ins_calib"));
    attachTask(task);
}

void Application::makeInsAlignTask() {
    const auto task = Tools::Ins::MakeStaticAlignTask();
    LOG_INFO << "[TASK] " << I18n::tr("app.log.new_tool", I18n::tr("gui.menu.tools.ins_align"));
    attachTask(task);
}

void Application::makeInsVerifyTask() {
    const auto task = Tools::Ins::MakeInsVerifyTask();
    LOG_INFO << "[TASK] " << I18n::tr("app.log.new_tool", I18n::tr("gui.menu.tools.ins_verify"));
    attachTask(task);
}

void Application::Render() {
    RenderMenuBar();


    for (const auto &tool: kToolPanels)
        if (this->*tool.open) tool.render(&(this->*tool.open));
    Download::Panel::state().render();
    if (m_showEarthdata) Download::Panel::renderEarthdata(&m_showEarthdata);
    RenderTasks();
}

void Application::Run() {
    Initialize();

    bool ready = true;
    while (!m_quitRequested && m_ui.BeginFrame(&ready)) {
        if (!ready) continue;
        Update();
        Render();
        AppConfig::PathMemory::instance().flush();
        m_ui.EndFrame();
    }

    Shutdown();
}
