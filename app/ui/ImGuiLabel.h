#pragma once

#include <string_view>

namespace UI {
    /// 译文 key 拼上 ImGui 的 "##id" / "###id" 后缀；dyn 接动态 ID 片段，返回值只在本次完整表达式内有效。
    const char *idLabel(std::string_view key, std::string_view suffix, std::string_view dyn = {});
}
