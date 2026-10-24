#include "ImuCalibDetect.h"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <vector>

namespace ImuCalibDetect {
    const char *posLabel(const ImuCalibrator::Pos p) {
        static const char *s[6] = {"X 上", "X 下", "Y 上", "Y 下", "Z 上", "Z 下"};
        return s[static_cast<int>(p)];
    }

    const char *rotLabel(const ImuCalibrator::Rot r) {
        static const char *s[6] = {"X 正转", "X 逆转", "Y 正转", "Y 逆转", "Z 正转", "Z 逆转"};
        return s[static_cast<int>(r)];
    }

    std::string fileContentHash(const std::string &path) {
        // FNV-1a 64
        auto fnv = [](const char *b, const size_t n, uint64_t h) {
            for (size_t i = 0; i < n; ++i) {
                h ^= static_cast<uint8_t>(b[i]);
                h *= 1099511628211ULL;
            }
            return h;
        };
        std::ifstream f(path, std::ios::binary);
        if (!f) return std::string("ERR:") + path;
        uint64_t h = 1469598103934665603ULL;
        f.seekg(0, std::ios::end);
        const std::streamoff sz = f.tellg();
        f.seekg(0);
        h = fnv(reinterpret_cast<const char *>(&sz), sizeof(sz), h);
        // 首尾各读 256KB（1<<18）；缓冲区与读取量一致，且用堆分配避免栈溢出（worker 线程默认栈仅 1MB）。
        constexpr std::streamoff kBlock = 1 << 18;
        std::vector<char> buf(kBlock);
        if (const std::streamoff head = std::min<std::streamoff>(sz, kBlock); head > 0) {
            f.read(buf.data(), head);
            h = fnv(buf.data(), static_cast<size_t>(f.gcount()), h);
        }
        if (sz > 1 << 19) {
            f.seekg(sz - kBlock);
            f.read(buf.data(), kBlock);
            h = fnv(buf.data(), static_cast<size_t>(f.gcount()), h);
        }
        char out[20];
        std::snprintf(out, sizeof(out), "%016llx", static_cast<unsigned long long>(h));
        return out;
    }

    namespace {
        // 返回「轴(0/1/2) + 符号」合成的 0..5 键：正轴→2*ax，负轴→2*ax+1
        int axisKey(const double a, const double b, const double c) {
            int ax = 0;
            double m = std::fabs(a);
            if (std::fabs(b) > m) {
                m = std::fabs(b);
                ax = 1;
            }
            if (std::fabs(c) > m) {
                m = std::fabs(c);
                ax = 2;
            }
            const double v = ax == 0 ? a : ax == 1 ? b : c;
            return v >= 0.0 ? 2 * ax : 2 * ax + 1;
        }
    } // namespace

    std::vector<CalibSegment> detect(const ImuLog &log, const DetectParams &p) {
        std::vector<CalibSegment> segs;
        if (log.empty()) return segs;
        const size_t N = log.size();

        // 1) 每样本陀螺主轴模长 + 加速度主轴模长（加速度量级≈g，作为稳定的旋转阈值参考）
        std::vector<double> m(N), am(N);
        for (size_t i = 0; i < N; ++i) {
            const auto &s = log[i];
            m[i] = std::max({
                std::fabs(static_cast<double>(s.gx)),
                std::fabs(static_cast<double>(s.gy)),
                std::fabs(static_cast<double>(s.gz))
            });
            am[i] = std::max({
                std::fabs(static_cast<double>(s.ax)),
                std::fabs(static_cast<double>(s.ay)),
                std::fabs(static_cast<double>(s.az))
            });
        }
        // 百分位（排序副本）
        auto pctOf = [](const std::vector<double> &v, const double q) {
            std::vector<double> c = v;
            std::sort(c.begin(), c.end());
            const size_t idx = std::min<size_t>(c.size() - 1, static_cast<size_t>(q * (c.size() - 1)));
            return c[idx];
        };
        const double aMed = pctOf(am, 0.50);

        // 2) 旋转阈值
        const double thr = std::max(p.rotThrCounts, aMed * p.rotFrac);

        // 3) 每样本「段键」
        std::vector<int> key(N);
        for (size_t i = 0; i < N; ++i) {
            const auto &s = log[i];
            if (m[i] > thr)
                key[i] = 6 + axisKey(s.gx, s.gy, s.gz); // 6..11=旋转
            else
                key[i] = axisKey(s.ax, s.ay, s.az); // 0..5 =静置
        }

        // 4) 初分段（连续同键）
        struct Run {
            size_t i0, i1;
            int key;
        };
        std::vector<Run> runs;
        size_t i = 0;
        while (i < N) {
            const int k = key[i];
            size_t j = i;
            while (j < N && key[j] == k) ++j;
            runs.push_back({i, j, k});
            i = j;
        }

        // 4.5) 估计采样间隔
        auto estimateDt = [](const ImuLog &_log) -> double {
            if (_log.size() < 8) return 0.0;
            std::vector<double> d;
            d.reserve(_log.size() - 1);
            for (size_t j = 1; j < _log.size(); ++j) {
                if (const double v = _log[j].tow - _log[j - 1].tow; v > 0.0) d.push_back(v);
            }
            if (d.size() < _log.size() / 2) return 0.0;
            const size_t mid = d.size() / 2;
            std::nth_element(d.begin(), d.begin() + static_cast<long>(mid), d.end());
            return d[mid];
        };
        const double dtEst = estimateDt(log);
        const size_t minLen = dtEst > 0.0
                                  ? std::max<size_t>(1, static_cast<size_t>(std::llround(p.minSegSeconds / dtEst)))
                                  : static_cast<size_t>(p.minSegSamples);

        // 5) 合并过短段
        bool changed = true;
        while (changed) {
            changed = false;
            for (size_t k = 0; k < runs.size(); ++k) {
                if (runs[k].i1 - runs[k].i0 >= minLen) continue;
                const bool hasL = k > 0;
                const bool hasR = k + 1 < runs.size();
                if (!hasL && !hasR) break;
                const size_t lenL = hasL ? runs[k - 1].i1 - runs[k - 1].i0 : 0;
                const size_t lenR = hasR ? runs[k + 1].i1 - runs[k + 1].i0 : 0;
                const bool toLeft = !hasR || (hasL && lenL >= lenR);
                const size_t seg = toLeft ? k - 1 : k + 1; // 较长的邻段
                runs[seg] = {std::min(runs[k].i0, runs[seg].i0), std::max(runs[k].i1, runs[seg].i1), runs[seg].key};
                runs.erase(runs.begin() + static_cast<long>(k));
                changed = true;
                break;
            }
        }

        // 6) 逐段用段落均值重分类
        for (const auto &[i0, i1, _key]: runs) {
            const size_t n = i1 - i0;
            if (n == 0) continue;
            const double inv = 1.0 / static_cast<double>(n);
            double ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
            for (size_t k = i0; k < i1; ++k) {
                const auto &s = log[k];
                ax += s.ax;
                ay += s.ay;
                az += s.az;
                gx += s.gx;
                gy += s.gy;
                gz += s.gz;
            }
            CalibSegment seg;
            seg.i0 = i0;
            seg.i1 = i1;
            if (_key >= 6) {
                // 旋转
                seg.isStatic = false;
                seg.rot = static_cast<ImuCalibrator::Rot>(axisKey(gx * inv, gy * inv, gz * inv));
            } else {
                // 静置
                seg.isStatic = true;
                seg.pos = static_cast<ImuCalibrator::Pos>(axisKey(ax * inv, ay * inv, az * inv));
            }
            segs.push_back(seg);
        }
        return segs;
    }
}
