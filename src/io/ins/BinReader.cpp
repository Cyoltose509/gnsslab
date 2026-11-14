#include "io/ins/BinReader.h"

#include <algorithm>
#include <cstdint>

namespace {
    constexpr std::size_t kBlockRecords = 4096;
}

bool BinRecordReader::open(const std::string &path, const std::size_t stride) {
    if (m_in.is_open()) m_in.close();
    m_in.clear();
    m_buf.clear();
    m_stride = stride;
    m_pos = m_filled = m_loaded = m_consumed = m_total = 0;
    m_error.clear();
    m_failed = false;
    if (stride == 0) {
        m_error = "记录长度必须大于 0 个 double";
        return false;
    }

    m_in.open(path, std::ios::in | std::ios::binary);
    if (!m_in) {
        m_error = "无法打开文件：" + path;
        return false;
    }
    m_in.seekg(0, std::ios::end);
    const std::streampos end = m_in.tellg();
    if (end < 0) {
        m_error = "无法读取文件长度：" + path;
        return false;
    }
    m_in.seekg(0, std::ios::beg);

    const auto recordBytes = static_cast<std::uintmax_t>(stride) * sizeof(double);
    const auto bytes = static_cast<std::uintmax_t>(end);
    m_total = bytes / recordBytes;
    if (const auto rest = bytes % recordBytes; rest != 0)
        m_error = "文件末尾有 " + std::to_string(rest) + " 字节凑不成一整条记录，已丢弃";
    if (m_total == 0) return true;

    m_buf.resize(kBlockRecords * stride);
    return fill();
}

bool BinRecordReader::fill() {
    m_pos = m_filled = 0;
    if (m_failed || m_loaded >= m_total) return false;
    const std::size_t want = std::min(kBlockRecords, m_total - m_loaded);
    const auto recordBytes = m_stride * sizeof(double);
    m_in.read(reinterpret_cast<char *>(m_buf.data()), static_cast<std::streamsize>(want * recordBytes));
    const auto got = static_cast<std::size_t>(m_in.gcount()) / recordBytes;
    m_filled = got;
    m_loaded += got;
    if (got < want) {
        m_failed = true;
        m_error = "读取中断，实际只读到 " + std::to_string(m_loaded) + " / " +
                  std::to_string(m_total) + " 条记录";
    }
    return m_filled > 0;
}

bool BinRecordReader::next(const double *&rec) {
    if (m_pos >= m_filled && !fill()) {
        rec = nullptr;
        return false;
    }
    rec = m_buf.data() + m_pos * m_stride;
    ++m_pos;
    ++m_consumed;
    return true;
}
