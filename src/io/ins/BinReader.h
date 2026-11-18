#pragma once

#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

class BinRecordReader {
public:
    void open(const std::string &path, std::size_t stride); // 打不开或长度非法抛异常

    bool next(const double *&rec); // 返回 false 只表示读完了；中途读错抛异常

    [[nodiscard]] std::size_t count() const { return m_total; }

    [[nodiscard]] std::size_t consumed() const { return m_consumed; }

    [[nodiscard]] bool empty() const { return m_total == 0; }

    [[nodiscard]] std::size_t stride() const { return m_stride; }

private:
    bool fill();

    std::ifstream m_in;
    std::vector<double> m_buf;
    std::size_t m_stride = 0;
    std::size_t m_pos = 0, m_filled = 0, m_loaded = 0, m_consumed = 0, m_total = 0;
};
