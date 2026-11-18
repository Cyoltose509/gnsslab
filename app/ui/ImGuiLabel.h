#pragma once

#include <cstddef>
#include <string>
#include <string_view>

#include "i18n/I18n.h"

namespace UI {
    /// 译文 key 拼上 ImGui 的 "##id" / "###id" 后缀；dyn 接动态 ID 片段，返回值只在本次完整表达式内有效。
    /// 轮转 8 个槽位：同一条表达式里放多个控件也不会互相覆盖（超过 8 个按最旧覆盖）。
    inline const char *idLabel(const std::string_view key, const std::string_view suffix,
                               const std::string_view dyn = {}) {
        constexpr std::size_t kSlots = 8;
        static thread_local std::string buf[kSlots];
        static thread_local std::size_t next = 0;
        std::string &out = buf[next++ % kSlots];
        out.assign(I18n::trc(key));
        out.append(suffix);
        out.append(dyn);
        return out.c_str();
    }
}
