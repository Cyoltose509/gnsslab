#pragma once

#include <functional>
#include <string>
#include <vector>

/// 解包：把下载回来的文件变成能直接读的明文。格式一律按文件头认，不看扩展名。
/// 外层压缩（gzip / compress）就地解开，归档（tar / zip）平铺展开到同名子目录、成员逐个落成明文，
/// 两种情况下压缩/归档包都只当中间产物、解出内容后删掉；Hatanaka 压缩的观测文件还原成标准 RINEX。
/// 格式解析交给 libarchive，本模块只管「落到哪个名字、原包留不留」。
namespace Download::Unpack {
    /// 解包进度：0~1，只供界面显示，不参与解包逻辑。
    using Progress = std::function<void(double frac)>;

    /// 一次 settle 的结果。
    struct Settled {
        std::string main;                  // 落点：解压出的文件，或归档展开后的同名目录
        std::vector<std::string> members;  // 归档展开出来的明文文件，非归档为空
    };

    /// 去掉外层压缩后缀与 SP3 的版本号，得到明文文件名。
    std::string plainName(const std::string &name);

    /// 落点名：在 plainName 基础上去掉 .tar / .zip，即归档展开目录的名字。
    std::string stemName(const std::string &name);

    /// RINEX2 短名的 Hatanaka 压缩扩展（.YYd / .YYD），只看名字不改后缀。
    bool isShortCompactName(const std::string &name);

    /// Hatanaka 压缩名 → 标准 RINEX 名（abcd1230.25d → abcd1230.25o，.crx → .rnx）。
    std::string rinexName(const std::string &name);

    /// 读成明文字符串：外层压缩在内存里解开，不落盘也不动原文件；非压缩文件原样读。
    std::string readPlain(const std::string &path);

    /// 就地解开外层压缩，成功后删掉压缩包；不是压缩文件或解不开时原样返回。
    std::string inflate(const std::string &path);

    /// 解开（可能被截断的）压缩流，把能解出的部分写成明文，返回明文路径。
    /// 与 inflate 的差别：不要求流尾完整。配合 Http::downloadPartial 只取压缩流前段时用。
    /// keepArchive 为真时保留压缩片段，供续传接下一段后重新解开。
    std::string inflateHead(const std::string &path, bool keepArchive = false);

    /// 是否是多文件的归档（tar / zip），按文件头判。
    bool isArchive(const std::string &path);

    /// 把 tar / zip 平铺展开到 outDir（丢掉目录层级），返回落盘路径。
    /// onProgress 按归档已读字节报 0~1。
    std::vector<std::string> expandTo(const std::string &archivePath, const std::string &outDir,
                                      const Progress &onProgress = {});

    /// 下载完调这一个：解外层压缩 → 归档展开（成员逐个再解） → Hatanaka 还原。
    /// 归档展开到主文件同级的同名子目录，展开成功后原包删掉，main 返回该目录。
    /// onProgress 按「外层解压 → 归档展开 → 成员逐个还原」三段加权报 0~1。
    Settled settle(const std::string &path, const Progress &onProgress = {});
}
