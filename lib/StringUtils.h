
#pragma once

#include <iostream>
#include <string>
#include <stdexcept> // For std::invalid_argument and std::out_of_range
#include <limits>    // For std::numeric_limits<double>::max()

using namespace std;

inline string strip(std::string s) {
    // 如果字符串为空，则无需处理
    if (s.empty()) return s;

    // 移除头部的空白字符
    s.erase(0, s.find_first_not_of(" \t\n\r\f\v"));

    // 移除尾部的空白字符
    s.erase(s.find_last_not_of(" \t\n\r\f\v") + 1);

    return s;
}

inline std::string& stripTrailing(std::string& line) {
    if (const size_t end = line.find_last_not_of(" \t\n\r\v\f"); end != std::string::npos) {
        line.resize(end + 1); // 修改字符串的长度，去掉后面的空白字符
    } else {
        line.clear(); // 如果全是空白字符，清空字符串
    }
    return line; // 返回修改后的字符串的引用
}

/// 安全的 substr：起始越界返回空串，长度超界自动截断
inline std::string safeSubstr(const std::string &str, size_t pos, size_t len) {
    if (pos >= str.size()) return {};
    return str.substr(pos, (std::min)(len, str.size() - pos));
}

inline double safeStod(const std::string &str, const double defaultValue = 0.0) {
    if (str.empty() || str.find_first_not_of(' ') == std::string::npos) {
        // 字符串为空或只包含空白字符
        return defaultValue;
    }

    try {
        return std::stod(str);
    } catch ([[maybe_unused]] const std::invalid_argument &e) {
        // 字符串不是有效的浮点数表示
        return defaultValue;
    } catch ([[maybe_unused]] const std::out_of_range &e) {
        // 转换后的数值超出了 double 类型可表示的范围
        return (std::numeric_limits<double>::max)();
    }
}

inline int safeStoi(const std::string& str) {
    if (str.empty()) {
        return 0; // 如果字符串为空，直接返回 0
    }

    size_t start = 0;
    while (start < str.size() && std::isspace(static_cast<unsigned char>(str[start]))) {
        start++; // 跳过开头的空白字符
    }

    if (start == str.size()) {
        return 0; // 如果全是空白字符，返回 0
    }

    size_t end = str.size() - 1;
    while (end > start && std::isspace(static_cast<unsigned char>(str[end]))) {
        end--; // 跳过结尾的空白字符
    }

    const std::string trimmedStr = str.substr(start, end - start + 1);

    try {
        return std::stoi(trimmedStr);
    } catch ([[maybe_unused]] const std::invalid_argument& e) {
        return 0; // 如果转换失败，返回 0
    } catch ([[maybe_unused]] const std::out_of_range& e) {
        return 0; // 如果整数超出范围，返回 0
    }
}


