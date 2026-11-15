#include "ui/ImGuiLabel.h"

#include <string>

#include "i18n/I18n.h"

namespace UI {
    const char *idLabel(const std::string_view key, const std::string_view suffix,
                        const std::string_view dyn) {
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
