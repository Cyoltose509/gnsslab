# GnssLab

GNSS 数据处理实验室：Windows 桌面程序，包含 SPP / PPP / RTK 定位、LEO 定轨、INS 纯惯导解算，以及观测数据下载、结果图表与质量分析。

© 2026 Cyoltose

## 运行

图形界面运行 `bin/GnssLab.exe`；命令行解算 `bin/gnss_cli.exe <配置.ini>`，全部键与取值见 exe 同级的 `gnss.ini`。

自行编译：CLion 打开项目选 `Windows (Ninja, x64)` 预设构建，首次拉取先 `git submodule update --init --recursive`，需 Windows + VS2022 的 C++ 桌面开发组件。

## 界面

  * **任务栏**：新建 SPP / PPP（静态、动态）/ RTK / LEO / INS 解算任务。
  * **文件页**：选定观测文件、星历与产品；静态模式填恒值参考真值，动态模式挂 SP3 真值文件，INS 任务在此填写初始位置与初始速度。
  * **参数页**：采样间隔、参与星座、截止高度角、改正模型、对流层映射、滤波方向等，按模式自动切换。
  * **结果页**：给出坐标（浮点 / 固定）、ECEF 速度及其模长与 σ、参考与偏差、DOP 和解状态；卫星表逐历元显示伪距与相位残差；图表与卫星可见性随时间轴联动。INS 结果可切换自由解与零速修正解。
  * **工具**：时间/坐标转换、最小二乘、惯导标定与对准、INS 算法验证。
  * **数据**：日精密产品 / 观测文件 / 低轨卫星 / 参考真值（基站 SINEX、低轨精密轨道 SP3）/ 辅助数据 / 本地文件六个页签；Earthdata 账号在同一页填写。
  * **帮助 → 关于**：显示本文档。

## 导出

  * **格式**：SPP / PPP / RTK 为 CSV、POS（RTKLIB `rtkpost` 同款文本）、SP3-c、KML（轨迹线，Google Earth 与 QGIS 等通用）；LEO 另加轨道根数；INS 为 CSV、轨迹、KML、SBET（每记录 17 个双精度的二进制轨迹）。
  * **采样**为与解算一致 / 每 N 秒 / 每 N 历元；**元素**为 CSV 逐列勾选；**范围**可只导出一段历元。

## 数据来源与引用

观测数据、精密轨道与钟差、电离层与对流层产品来自 **IGS** 及其数据中心（**NASA CDDIS**、**WHU**）、**CODE**（AIUB）与 TU Wien；低轨星载观测与精密轨道来自 **NASA PO.DAAC**、**UCAR CDAAC**（COSMIC-2）、**ESA Swarm**、**GFZ ISDC** 与 **IDS**（Sentinel-6A/6B）。

用这些数据发表成果时请按官方要求引用署名：IGS 要求恰当引用并署名（[Disclaimer and Terms of Use](https://igs.org/about/disclaimer/)）；NASA 数据须在正文引用并在参考文献注明来源，CDDIS / PO.DAAC 数据集页面给出含 DOI 与访问日期的推荐格式（[Data Use Guidance](https://www.earthdata.nasa.gov/engage/open-data-services-software-policies/data-use-guidance)）；ESA 要求声明 ESA 为数据来源（[Data Policy](https://earth.esa.int/eogateway/documents/20142/1564626/ESA-Data-Policy-ESA-PB-EO-2010-54.pdf)）；UCAR CDAAC 与 GFZ ISDC 见各自站点条款。

产品本身的引用：CODE 系列为 Dach et al., *CODE final product series for the IGS*, DOI [10.48620/97773](https://doi.org/10.48620/97773)（CC0 1.0）；IDS 的 Sentinel-6A / Jason 轨道为 GSFC std2400，DOI [10.24400/312072/i01-2025.001](https://doi.org/10.24400/312072/i01-2025.001)。

上述机构名称仅用于说明数据来源，不代表对本程序的认可或背书；数据仅供科研与教学，不得用于生命安全关键系统，使用风险自担。

## 开源组件致谢

版权归各自作者所有，许可证全文见 `thirdparty/` 对应目录：

  * [Dear ImGui](https://github.com/ocornut/imgui) — 图形界面，MIT License
  * [ImPlot](https://github.com/epezent/implot) — 二维图表，MIT License
  * [ImPlot3D](https://github.com/brenocq/implot3d) — 三维图表，MIT License
  * [imgui_markdown](https://github.com/juliettef/imgui_markdown) — 关于页 Markdown 渲染，zlib License
  * [Eigen](https://eigen.tuxfamily.org) — 矩阵与最小二乘运算，MPL 2.0（部分文件为 BSD / LGPL）
  * [libcurl](https://curl.se) — FTP / HTTPS 数据传输，curl License（MIT 类）
  * [libarchive](https://libarchive.org) — 归档与压缩包解包（gzip / compress / tar / zip），BSD 2-Clause
  * [zlib](https://zlib.net) — gzip 与 zip 的 deflate 解压，libarchive 的依赖，zlib License
