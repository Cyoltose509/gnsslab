#pragma once

#include <cstddef>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

/// 界面文案与术语表。表在构建期由 src/i18n/gen_lang 从 src/lang 与 app/lang 的 CSV 编成
/// LangData.cpp，运行期只做二分查表；切换语言时把该语言的整份表（含 {{术语}} 展开）展平一次，
/// 换指针即生效。
namespace I18n {
    /// 生成器按实际列数填充，多出来的槽位留空；加语言不必改这里，只是别超过这个数。
    inline constexpr std::size_t kMaxLanguages = 8;

    struct Language {
        std::string_view code;
        std::string_view name;
    };

    namespace Detail {
        /// 一行文案：key + 各语言文本（下标同 kLangCodes）。
        struct Entry {
            std::string_view key;
            std::string_view text[kMaxLanguages];
        };

        extern const std::string_view kLangCodes[];
        extern const std::string_view kLangNames[];
        extern const std::size_t kLangCount;
        extern const Entry kEntries[];   // 文案表，按 key 升序
        extern const std::size_t kEntryCount;
        extern const Entry kTerms[];     // 术语表，按 key 升序；{{}} 展开与 tr() 回退共用
        extern const std::size_t kTermCount;

        template<class T>
        std::string argText(T &&v) {
            using U = std::decay_t<T>;
            if constexpr (std::is_same_v<U, std::string>) {
                return std::forward<T>(v);
            } else if constexpr (std::is_same_v<U, std::string_view>) {
                return std::string(v);
            } else if constexpr (std::is_same_v<U, const char *> || std::is_same_v<U, char *>) {
                return v ? std::string(v) : std::string();
            } else if constexpr (std::is_same_v<U, char>) {
                return std::string(1, v);
            } else if constexpr (std::is_integral_v<U>) {
                return std::to_string(v);
            } else {
                std::ostringstream os;
                os << v;
                return os.str();
            }
        }

        /// 把 {0}{1} 按显式索引替换；索引越界或缺参时原样留着，便于一眼看出漏传。
        std::string fill(std::string_view text, const std::string *args, std::size_t count);
    }

    const std::vector<Language> &languages();

    /// 切到指定语言代码；不认识的代码返回 false，保持当前语言。
    bool setLanguage(std::string_view code);

    std::string_view language();

    /// 文案本体（术语已展平）。键不存在时返回键原文。
    std::string_view trv(std::string_view key);

    /// 同上，但给需要 NUL 结尾的 C 接口用（ImGui、Win32）。指向表内常驻 std::string。
    const char *trc(std::string_view key);

    std::string tr(std::string_view key);

    /// 带参数的文案：tr("spp.log.start", 15.0, 4) → "截止高度角 15°, 卫星 4"。
    /// 浮点走流默认精度（与手写 << 一致），别用 to_string 的 6 位小数。
    template<class... A>
    std::string tr(std::string_view key, A &&...args) {
        if constexpr (sizeof...(A) == 0) {
            return std::string(trv(key));
        } else {
            const std::string slot[] = {Detail::argText(std::forward<A>(args))...};
            return Detail::fill(trv(key), slot, sizeof...(A));
        }
    }

    bool exists(std::string_view key);

    /// 累计缺键次数（每次取用都算）与退出时的一次汇总。
    std::size_t missingKeys();

    void reportMissing();
}
