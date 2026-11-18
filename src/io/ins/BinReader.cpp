#include "io/ins/BinReader.h"

#include <algorithm>
#include <cstdint>

#include "Exception.h"

namespace {
    constexpr std::size_t kBlockRecords = 4096;
}

void BinRecordReader::open(const std::string &path, const std::size_t stride) {
    if (m_in.is_open()) m_in.close();
    m_in.clear();
    m_buf.clear();
    m_stride = stride;
    m_pos = m_filled = m_loaded = m_consumed = m_total = 0;
    if (stride == 0) throw InvalidRequest("BinRecordReader: record stride must be non-zero");

    m_in.open(path, std::ios::in | std::ios::binary);
    if (!m_in) throw FileMissingException("BinRecordReader: cannot open " + path);
    m_in.seekg(0, std::ios::end);
    const std::streampos end = m_in.tellg();
    if (end < 0) throw FFStreamError("BinRecordReader: cannot determine size of " + path);
    m_in.seekg(0, std::ios::beg);

    const auto recordBytes = static_cast<std::uintmax_t>(stride) * sizeof(double);
    const auto bytes = static_cast<std::uintmax_t>(end);
    m_total = bytes / recordBytes;
    if (m_total == 0) return;

    m_buf.resize(kBlockRecords * stride);
    fill();
}

bool BinRecordReader::fill() {
    m_pos = m_filled = 0;
    if (m_loaded >= m_total) return false;
    const std::size_t want = std::min(kBlockRecords, m_total - m_loaded);
    const auto recordBytes = m_stride * sizeof(double);
    m_in.read(reinterpret_cast<char *>(m_buf.data()), static_cast<std::streamsize>(want * recordBytes));
    const auto got = static_cast<std::size_t>(m_in.gcount()) / recordBytes;
    m_filled = got;
    m_loaded += got;
    if (got < want)
        throw FFStreamError("BinRecordReader: read interrupted after " + std::to_string(m_loaded) + " of " +
                            std::to_string(m_total) + " records");
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
