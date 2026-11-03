# GnssLab

GnssLab 是一个基于 C++17 / CMake 的 GNSS（全球导航卫星系统）数据处理桌面程序，集成了文件读取、定位、处理，以及质量分析等功能；项目仍在持续开发中，功能会不断扩充。

© 2026 Cyoltose

## submodule构建

```bash
git submodule update --init --recursive
```

用 CLion 打开项目，选择 CMake 预设 `Windows (Ninja, x64) — 推荐`（或已有的 Release 配置），构建即可。

前置要求：Windows + Visual Studio 2022（勾选「使用 C++ 的桌面开发」工作负载）+ Git + CLion。
