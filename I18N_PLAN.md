# GnssLab 多语言（i18n）实施方案

状态：待启动（2026-11-03 定稿，代码未动）
范围：GUI 全部文案 + 解算日志 + CLI 输出，文本随 exe 发布，无外部语言文件依赖。

---

## 1. 现状盘点

| 位置 | 含中文字面量 | 备注 |
|---|---|---|
| `app/widgets` | 915 条 / 4818 字 | 页面标题、按钮、表头、提示、文件对话框过滤器 |
| `src` | 81 条 / 553 字 | 解算日志、异常消息 |
| `app/core` | 58 条 | 驱动层用户可见错误串 |
| `app/download` | 51 条 | 下载模块状态与错误 |
| `app/cli` | 47 条 | `std::cerr` / `std::cout`（不带 `LOG_*`） |
| `app/ui` | 2 条 | 框架层 |

日志调用点：`LOG_*` 共 96 处（src 12、widgets 56、download 19、core 9），CLI 直接打印 47 处，`throw` 54 处。

文案硬编码在 `ImGui::Text/BeginTabItem/Button` 与 `LOG_*` 流中；`SolverTask::qcTabLabel = "质量分析"` 之类还存在**把译文存进成员**的情况，热切换会残留，必须改成存 key。

---

## 2. 总体设计

```
src/core/I18n.h / I18n.cpp          接口 + 查表 + 术语展开 + 占位替换 + 缺键统计
src/core/lang/*.csv                 文案与术语（唯一真源）
tools/gen_lang.cpp                  构建期生成器（host 工具，无外部依赖）
<build>/gen/LangData.cpp            生成物，不进仓库
```

### 2.1 嵌入方式：构建期生成 `constexpr` 表

`gen_lang` 读 `src/core/lang/*.csv`，产出：

```cpp
namespace I18n::Detail {
    inline constexpr std::string_view kLangCodes[] = {"ZH-CN", "ZH-TW", "EN"};
    inline constexpr std::string_view kLangNames[] = {"简体中文", "繁體中文", "English"};
    struct Entry { std::string_view key; std::string_view text[kLangCount]; };
    inline constexpr Entry kEntries[] = { /* 按 key 升序 */ };
    inline constexpr Entry kTerms[]   = { /* 术语表，同样按 key 升序 */ };
}
```

- `string_view` 直指字面量：**启动零解析、查表零分配**。
- 排序由生成器完成，运行时 `std::lower_bound`，O(log n)。
- 缺键、重复键、占位符不一致在**构建期**报错，不让它变成运行时空白。

否决的方案：`.rc` + `RCDATA` 嵌原始 CSV。省一个工具，但要在 `src/core` 里新写 CSV reader（现有 `app/core/CSV` 只有 writer），解析错误只能运行时暴露；且 Win32 资源 API 侵入 core 层。

CMake 接线（`src/CMakeLists.txt`）：

```cmake
add_executable(gen_lang tools/gen_lang.cpp)
target_sources(gnss PRIVATE ${CMAKE_BINARY_DIR}/gen/LangData.cpp)
add_custom_command(
        OUTPUT  ${CMAKE_BINARY_DIR}/gen/LangData.cpp
        COMMAND gen_lang ${CMAKE_SOURCE_DIR}/src/core/lang ${CMAKE_BINARY_DIR}/gen/LangData.cpp
        DEPENDS gen_lang ${LANG_CSV_FILES}
        COMMENT "生成多语言表")
```

注意：`DEPENDS` 指向可执行目标需 CMake ≥ 3.20（当前 3.18），要么抬版本，要么改用 `add_custom_target` + `add_dependencies` 两段式。

### 2.2 CSV 规范

```csv
key,ZH-CN,ZH-TW,EN
spp.param.elev_cut,高度截止角,高度截止角,Elevation mask
spp.status.fixed,固定解,固定解,Fixed
spp.log.sat_count,{{卫星数}}：{0} 颗,{{衛星數}}：{0} 顆,{{Satellite count}}: {0}
```

- UTF-8 无 BOM（容忍 BOM），LF，RFC 4180 引号规则（允许字段内逗号/引号/换行）。
- 第一列 `key`，其余列头即语言代码；**加语言只加一列**，生成器按列头识别。
- 文件按模块拆分，key 前缀与文件对应：`common`（通用词/单位）、`gui`（框架/菜单）、`spp` / `rtk` / `ppp` / `leo` / `ins`、`download`、`log`（解算日志）、`terms`（术语表）。
- key 命名 `区域.对象.属性`，全小写、下划线分词：`spp.param.elev_cut`、`rtk.log.base_pos_missing`、`gui.file_filter.all`。
- 术语表 `terms.csv` 与文案同结构，只用于 `{{}}` 展开，不直接显示。

### 2.3 接口

```cpp
namespace I18n {
    struct Language { std::string_view code, name; };

    const std::vector<Language> &languages();
    bool setLanguage(std::string_view code);
    std::string_view language();

    std::string_view trv(std::string_view key);           // 无参：直接返回已展平的 view
    std::string tr(std::string_view key);                 // 展开 {{术语}}（同样走缓存）
    template <class... A>
    std::string tr(std::string_view key, A &&...args);    // {0}{1} 顺序替换

    bool exists(std::string_view key);
    std::size_t missingKeys();                            // 供退出时汇总
}
```

- 静态风格（与 `Log::init/write` 一致），不做实例传递。
- **术语在切换语言时一次性展平**：为每个语言构造不可变 `LangPack { table, flattened[] }`，`trv()` 直接返回其中一份 view；`setLanguage` 只做 `std::atomic<const LangPack*>` 换指针，读端无锁（解算线程写日志、GUI 线程画界面并发安全）。
- 缺键：返回 key 原文（界面立刻露馅），首次 `LOG_WARN` 一次，退出时汇总条数。
- 回退链：当前语言 → 默认语言（`ZH-CN`）→ key。

### 2.4 术语与占位

- `{{term}}` 递归展开，深度上限 10（与参考实现一致），循环引用截断。
- 占位用显式索引 `{0}` `{1}`，**不用 `%s`**：自己替换不走 printf，类型安全；显式索引让译文能调语序。
- 数字格式化在调用方完成（`CSV::fx(v, 3)` 之类），翻译层不掺格式逻辑。

---

## 3. 日志与 CLI

### 3.1 一律「键 + 参数」

```cpp
LOG_ERROR << I18n::tr("obs.open.failed", path);
LOG_WARN  << I18n::tr("spp.warn.unhealthy", sat, eph->health);
```

禁止 `LOG_X << "无法打开 " << path` 这类拼接——语序一变就没法翻。

### 3.2 稳定段不翻

`[SPP]`、`[RTK]`、`[PPPStatic]`、`[回退]` 这类模块前缀保持 ASCII 原样（`[回退]` 改为 `[FALLBACK]`），排障时 `grep "\[RTK\]"` 仍能捞全。

### 3.3 日志文件语言

文件跟当前 UI 语言；每次切换插一行 `[I18N] language -> EN`；会话首行记 `GnssLab 0.6.0 lang=ZH-CN`。用户发回的日志能看出语言与断点。

（备选：文件恒写 EN、界面按当前语言显示。需在内存里保留 key+参数渲染两次，当前无日志窗口，收益不大，暂不采用。）

### 3.4 异常文本不翻

`throw EndOfFile("nav data truncated")` 等 54 处保持英文（开发者诊断用）。用户可见路径是 catch 后转 `err` 字符串（如 `SolveDriver.h`），在那里用 key 渲染。

---

## 4. 运行时切换

- 文案**每帧绘制时取**，禁止存进成员。待改：`SolverTask::qcTabLabel`、`GuiFtpDownload` 的分类名数组等。
- 选择存 `PathMemory`（`gui.language`），启动读取；缺省跟随 `GetUserDefaultUILanguage()`。
- 字体：ImGui 1.92 支持动态字形，先实测去掉 `GetGlyphRangesChineseFull()` 后日/韩/俄首帧是否掉字；不行就 `msyh.ttc`（CJK）+ `segoeui.ttf`（拉丁/西里尔）合并字体栈。
- Win32 文件对话框：`GuiFileFilter` 存 UTF-8，调用点转 wide（`Utf8ToWide(I18n::trv(...))`）。
- 不翻的内容：CSV 导出表头（数据契约）、RINEX/星历字段名、站点名、卫星号。

---

## 5. 工具链

| 工具 | 时机 | 职责 |
|---|---|---|
| `tools/gen_lang` | 构建期（C++，无外部依赖） | CSV → `LangData.cpp`；同时校验：重复键、空 ZH-CN、各语言 `{n}` 索引集合不一致、`{{}}` 引用了未定义术语、列数不一致 → 退出码非 0 |
| `tools/extract_lang.py` | 开发期（可选，Python） | 扫源码中文字面量 → 生成占位 key + CSV 行 + 替换成 `trv("…")`，人工改成语义 key |
| `tools/lang_check` | 开发期 | 未使用键、缺语言列、术语一致性；与 `gen_lang` 的校验互补 |

测试与一次性脚本一律放 `test/`，产物进 `.cmake-build-debug`（见 `MEMORY.md`）。

---

## 6. 实施阶段

| 阶段 | 内容 | 验收 |
|---|---|---|
| P0 机制 | `I18n.{h,cpp}`、`tools/gen_lang`、CMake 接线、`common/gui/terms.csv` 各几条示例、语言下拉 + 持久化 + 字体 | 切 EN 后下拉与示例 key 生效；exe 拷到空目录仍正常（证明已内嵌）；非 0 退出码能在故意写坏 CSV 时触发 |
| P1 样板 | SPP 参数页 100% key 化；`SolverTask` 里存中文的字段改存 key | 运行中切语言，页面无残留中文、无布局崩 |
| P2 UI 批量 | `app/widgets` 915 条，按文件分批迁移 | 每批编译 + 手点；缺键统计为 0 |
| P3 日志/CLI | 96 处 `LOG_*` + 47 处 CLI 打印改键+参数；日志 header 与切换标记 | 切语言后新日志行为新语言、旧行不变、`[MODULE]` 前缀仍可 grep |
| P4 收尾 | `download` 51 条、异常转用户提示处 key 化、`lang_check` 全绿 | 三语言全量跑一轮解算，日志无 key 原文泄漏 |

---

## 7. 风险

1. **文本变长**：中→英普遍 1.5–2×，`SetNextItemWidth`、固定列宽、按钮宽度要复查；建议 P1 阶段就按 EN 布局定尺寸。
2. **热切换残留**：任何把译文缓存在成员/静态里的地方都会残留，只能靠逐个排查 + 样板页验收。
3. **字体覆盖**：日/韩/俄字形是否存在、动态字形是否首帧生效，必须实测。
4. **改文案要重编**：内嵌的代价，符合"单 exe 发布"的诉求；开发期可用 `GNSSLAB_LANG_DIR` 外部覆盖（发布版忽略）方便校对。
5. **术语一致性**：`terms.csv` 先行，否则 915 条各译各的。
6. **翻译质量**：ZH-TW 可用 OpenCC 从简体转初稿；EN 机翻打底后必须人工过一遍解算术语（IF 组合、固定解、周跳、对流层延迟…）。

---

## 8. 待定

1. 首发语言集：简体 + 繁体 + EN（建议）还是直接上日/韩/俄？
2. `GNSSLAB_LANG_DIR` 开发期覆盖要不要做？
3. 生成器抬 CMake 到 3.20，还是走 `add_custom_target` + `add_dependencies`？
4. 抽取脚本用 Python 还是并进 C++ 工具（后者正则处理会难写）。
