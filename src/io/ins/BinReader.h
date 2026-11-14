#pragma once

#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

class BinRecordReader {
public:
    bool open(const std::string &path, std::size_t stride);

    bool next(const double *&rec);

    [[nodiscard]] std::size_t count() const { return m_total; }

    [[nodiscard]] std::size_t consumed() const { return m_consumed; }

    [[nodiscard]] bool empty() const { return m_total == 0; }

    [[nodiscard]] std::size_t stride() const { return m_stride; }

    [[nodiscard]] bool bad() const { return m_failed; }

    [[nodiscard]] const std::string &error() const { return m_error; }

private:
    bool fill();

    std::ifstream m_in;
    std::vector<double> m_buf;
    std::size_t m_stride = 0;
    std::size_t m_pos = 0, m_filled = 0, m_loaded = 0, m_consumed = 0, m_total = 0;
    std::string m_error;
    bool m_failed = false;
};
