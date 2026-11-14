#include "download/local/Unpack.h"

#include "Log.h"
#include "StringUtils.h"
#include "io/CRXDecoder.h"

#include <archive.h>
#include <archive_entry.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace Download::Unpack {
    namespace {
        enum class Kind { None, Gzip, Compress, Tar, Zip };

        Kind sniff(const std::string &path) {
            std::ifstream in(path, std::ios::binary);
            if (!in) return Kind::None;
            unsigned char head[512] = {};
            in.read(reinterpret_cast<char *>(head), sizeof head);
            const auto n = static_cast<std::size_t>(in.gcount());

            if (n >= 2 && head[0] == 0x1F && head[1] == 0x8B) return Kind::Gzip;
            if (n >= 2 && head[0] == 0x1F && head[1] == 0x9D) return Kind::Compress;
            if (n >= 4 && head[0] == 'P' && head[1] == 'K' && head[2] == 0x03 && head[3] == 0x04) return Kind::Zip;
            if (n >= 4 && head[0] == 'P' && head[1] == 'K' && head[2] == 0x05 && head[3] == 0x06) return Kind::Zip;
            if (n >= 262 && std::string(reinterpret_cast<char *>(head + 257), 5) == "ustar") return Kind::Tar;
            return Kind::None;
        }

        std::string flatName(const std::string &raw) {
            std::string s = raw;
            if (const std::size_t at = s.find_last_of("/\\"); at != std::string::npos) s = s.substr(at + 1);
            if (s.empty() || s == "." || s == "..") return {};
            return s;
        }

        archive *openRead(const std::string &path, const bool archiveFormats) {
            archive *a = archive_read_new();
            if (!a) return nullptr;
            archive_read_support_filter_gzip(a);
            archive_read_support_filter_compress(a);
            if (archiveFormats) {
                archive_read_support_format_tar(a);
                archive_read_support_format_zip(a);
            } else {
                archive_read_support_format_raw(a);
            }
            if (archive_read_open_filename(a, path.c_str(), 65536) != ARCHIVE_OK) {
                LOG_WARN << "[解包] 打不开 " << path << "：" << archive_error_string(a);
                archive_read_free(a);
                return nullptr;
            }
            return a;
        }

        void report(const Progress *cb, const double frac) {
            if (cb && *cb) (*cb)(std::clamp(frac, 0.0, 1.0));
        }

        class ByteProgress {
        public:
            ByteProgress(Progress cb, archive *a, const std::string &path)
                : mA(a), mCb(std::move(cb)) {
                if (!mCb) return;
                std::error_code ec;
                mTotal = static_cast<double>(fs::file_size(path, ec));
                if (mTotal <= 0.0) mCb = {};
            }

            void tick() {
                if (!mCb) return;
                const auto done = static_cast<double>(archive_filter_bytes(mA, 0));
                if (done - mLast < 1048576.0 && done < mTotal) return;
                mLast = done;
                mCb(std::clamp(done / mTotal, 0.0, 1.0));
            }

            void finish() const {
                if (mCb) mCb(1.0);
            }

        private:
            archive *mA = nullptr;
            Progress mCb;
            double mTotal = 0.0;
            double mLast = 0.0;
        };

        /// 把当前条目写进 fo，返回是否完整写完。
        bool drainEntry(archive *a, std::ostream &fo, ByteProgress *bp = nullptr) {
            std::array<char, 65536> buf{};
            for (;;) {
                const la_ssize_t n = archive_read_data(a, buf.data(), buf.size());
                if (n == 0) return true;
                if (n < 0) {
                    LOG_WARN << "[解包] 读出失败：" << archive_error_string(a);
                    return false;
                }
                fo.write(buf.data(), n);
                if (!fo) return false;
                if (bp) bp->tick();
            }
        }

        /// 解开单流压缩，成功后删原包；不是压缩文件或解不开时原样返回。quiet 供归档成员批量解压时收日志。
        std::string inflateFile(const std::string &path, const bool quiet, const Progress &onProgress = {}) {
            if (const Kind k = sniff(path); k != Kind::Gzip && k != Kind::Compress) return path;

            std::string out = plainName(path);
            if (out == path) out = path + ".unpacked";

            archive *a = openRead(path, false);
            if (!a) return path;

            archive_entry *entry = nullptr;
            if (archive_read_next_header(a, &entry) != ARCHIVE_OK) {
                LOG_WARN << "[解包] " << path << " 不是完整的压缩流：" << archive_error_string(a);
                archive_read_free(a);
                return path;
            }

            std::ofstream fo(out, std::ios::binary | std::ios::trunc);
            if (!fo) {
                archive_read_free(a);
                return path;
            }
            ByteProgress bp(onProgress, a, path);
            const bool ok = drainEntry(a, fo, &bp);
            fo.close();
            archive_read_free(a);

            std::error_code ec;
            if (!ok || fo.fail()) {
                fs::remove(out, ec);
                LOG_WARN << "[解包] 解压失败，保留压缩包：" << path;
                return path;
            }
            bp.finish();
            fs::remove(path, ec);
            if (!quiet) LOG_INFO << "[解包] " << path << " → " << out;
            return out;
        }

        /// 解包的递归体内核
        Settled settleImpl(const std::string &path, const bool quiet, const Progress *cb, const double inflateHi) {
            Settled s;
            const Progress inflate = cb && *cb
                ? Progress([cb, inflateHi](const double f) { report(cb, f * inflateHi); })
                : Progress{};
            s.main = inflateFile(path, quiet, inflate);

            if (const Kind k = sniff(s.main); k == Kind::Tar || k == Kind::Zip) {
                const std::string stem = stemName(fs::path(s.main).filename().string());
                const std::string dir = (fs::path(s.main).parent_path() / stem).generic_string() + "/";
                std::error_code ec;
                fs::create_directories(dir, ec);
                if (ec) {
                    LOG_WARN << "[解包] 建展开目录失败 " << dir << "：" << ec.message();
                    return s;
                }
                const Progress expand = cb && *cb
                    ? Progress([cb](const double f) { report(cb, 0.10 + 0.25 * f); })
                    : Progress{};
                s.members = expandTo(s.main, dir, expand);
                if (s.members.empty()) return s; // 空包/坏包：原包留着，便于重下

                // 成员多半还套着一层 gzip（COSMIC-2 的 podCrx 就是一包 *_crx.gz），逐个落成明文。
                const auto n = static_cast<double>(s.members.size());
                for (std::size_t i = 0; i < s.members.size(); ++i) {
                    s.members[i] = settleImpl(s.members[i], true, nullptr, 0.0).main;
                    report(cb, 0.35 + 0.65 * static_cast<double>(i + 1) / n);
                }

                std::error_code rmEc;
                fs::remove(s.main, rmEc);
                if (rmEc) LOG_WARN << "[解包] 归档删除失败，保留 " << s.main << "：" << rmEc.message();
                s.main = dir.substr(0, dir.size() - 1);
                if (!quiet)
                    LOG_INFO << "[解包] " << path << " → 目录 " << s.main << "（" << s.members.size() << " 个文件）";
                return s;
            }

            if (!CRXDecoder::isCompactRINEX(s.main)) return s;

            const std::string rnx = rinexName(s.main);
            CRXDecoder decoder;

            if (std::string err; !decoder.decode(s.main, rnx, err)) {
                LOG_WARN << "[解包] Hatanaka 还原失败，保留原文件：" << err;
                return s;
            }
            std::error_code ec;
            fs::remove(s.main, ec);
            if (!quiet) LOG_INFO << "[解包] 已还原 RINEX 观测文件：" << rnx;
            s.main = rnx;
            return s;
        }
    }

    std::string plainName(const std::string &name) {
        std::string out = Str::stripSuffixIC(Str::stripSuffixIC(name, ".gz"), ".Z");
        const std::size_t at = Str::toLower(out).rfind(".sp3.");
        if (at == std::string::npos) return out;
        if (const std::string ver = out.substr(at + 5);
            ver.size() == 3 && std::isdigit(static_cast<unsigned char>(ver[0])) != 0 &&
            std::isdigit(static_cast<unsigned char>(ver[1])) != 0 &&
            std::isdigit(static_cast<unsigned char>(ver[2])) != 0)
            out = out.substr(0, at + 4);
        return out;
    }

    std::string stemName(const std::string &name) {
        std::string out = plainName(name);
        out = Str::stripSuffixIC(out, ".unpacked");
        for (const char *suffix: {".tgz", ".tar", ".zip"}) out = Str::stripSuffixIC(out, suffix);
        return out;
    }

    bool isShortCompactName(const std::string &name) {
        return name.size() >= 4 && name[name.size() - 4] == '.' &&
               std::isdigit(static_cast<unsigned char>(name[name.size() - 3])) != 0 &&
               std::isdigit(static_cast<unsigned char>(name[name.size() - 2])) != 0 &&
               (name.back() == 'd' || name.back() == 'D');
    }

    std::string rinexName(const std::string &name) {
        if (Str::endsWithIC(name, ".crx")) {
            const bool upper = name.substr(name.size() - 3) == "CRX";
            return name.substr(0, name.size() - 3) + (upper ? "RNX" : "rnx");
        }
        if (Str::endsWithIC(name, "_crx")) return name.substr(0, name.size() - 4) + ".rnx";
        if (isShortCompactName(name)) {
            std::string out = name;
            out.back() = name.back() == 'd' ? 'o' : 'O';
            return out;
        }
        return name + ".rnx";
    }

    std::string readPlain(const std::string &path) {
        if (const Kind k = sniff(path); k != Kind::Gzip && k != Kind::Compress) {
            std::ifstream in(path, std::ios::binary);
            if (!in) return {};
            std::ostringstream ss;
            ss << in.rdbuf();
            return ss.str();
        }

        archive *a = openRead(path, false);
        if (!a) return {};
        archive_entry *entry = nullptr;
        if (archive_read_next_header(a, &entry) != ARCHIVE_OK) {
            LOG_WARN << "[解包] " << path << " 不是完整的压缩流：" << archive_error_string(a);
            archive_read_free(a);
            return {};
        }
        std::ostringstream ss;
        const bool ok = drainEntry(a, ss);
        archive_read_free(a);
        if (!ok) return {};
        return ss.str();
    }

    std::string inflate(const std::string &path) {
        return inflateFile(path, false);
    }

    std::string inflateHead(const std::string &path, const bool keepArchive) {
        if (const Kind k = sniff(path); k != Kind::Gzip && k != Kind::Compress) return path;

        std::string out = plainName(path);
        if (out == path) out = path + ".unpacked";

        archive *a = openRead(path, false);
        if (!a) return path;

        archive_entry *entry = nullptr;
        if (archive_read_next_header(a, &entry) != ARCHIVE_OK) {
            archive_read_free(a);
            std::error_code ec;
            fs::remove(out, ec);
            return path;
        }

        std::ofstream fo(out, std::ios::binary | std::ios::trunc);
        if (!fo) {
            archive_read_free(a);
            return path;
        }
        // 截断的流 archive_read_data 会返回负值，此前已解出的字节仍然有效，直接收下。
        std::array<char, 65536> buf{};
        for (;;) {
            const la_ssize_t n = archive_read_data(a, buf.data(), buf.size());
            if (n <= 0) break;
            fo.write(buf.data(), n);
            if (!fo) break;
        }
        fo.close();
        archive_read_free(a);

        std::error_code ec;
        const auto size = fs::file_size(out, ec);
        if (fo.fail() || ec || size == 0) {
            fs::remove(out, ec);
            return path;
        }
        if (!keepArchive) fs::remove(path, ec);
        LOG_INFO << "[解包] 压缩流片段 " << path << " → " << out << "（" << size << " 字节）";
        return out;
    }

    bool isArchive(const std::string &path) {
        const Kind k = sniff(path);
        return k == Kind::Tar || k == Kind::Zip;
    }

    std::vector<std::string> expandTo(const std::string &archivePath, const std::string &outDir,
                                      const Progress &onProgress) {
        std::vector<std::string> out;
        archive *a = openRead(archivePath, true);
        if (!a) return out;
        ByteProgress bp(onProgress, a, archivePath);

        archive_entry *entry = nullptr;
        while (archive_read_next_header(a, &entry) == ARCHIVE_OK) {
            if (archive_entry_filetype(entry) != AE_IFREG) {
                archive_read_data_skip(a);
                continue;
            }
            const std::string flat = flatName(archive_entry_pathname(entry));
            if (flat.empty()) {
                archive_read_data_skip(a);
                continue;
            }

            const std::string dest = outDir + flat;
            std::ofstream fo(dest, std::ios::binary | std::ios::trunc);
            if (!fo) {
                archive_read_data_skip(a);
                continue;
            }
            const bool ok = drainEntry(a, fo, &bp);
            fo.close();
            if (ok && fo.good()) {
                out.push_back(dest);
            } else {
                std::error_code ec;
                fs::remove(dest, ec);
            }
        }
        bp.finish();
        archive_read_free(a);

        LOG_INFO << "[解包] 展开 " << archivePath << " → " << out.size() << " 个文件";
        return out;
    }

    Settled settle(const std::string &path, const Progress &onProgress) {
        if (!onProgress) return settleImpl(path, false, nullptr, 0.0);
        const Kind k = sniff(path);
        const std::string bare = plainName(path);
        const bool tarInside = Str::endsWithIC(bare, ".tar") || Str::endsWithIC(bare, ".zip") ||
                               Str::endsWithIC(bare, ".tgz");
        const double inflateHi = (k == Kind::Gzip || k == Kind::Compress) && !tarInside ? 0.85 : 0.10;
        report(&onProgress, 0.0);
        Settled s = settleImpl(path, false, &onProgress, inflateHi);
        report(&onProgress, 1.0);
        return s;
    }
}
