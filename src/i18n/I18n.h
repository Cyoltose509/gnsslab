#pragma once

#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>


namespace I18n {
    inline constexpr std::size_t kMaxLanguages = 8;

    struct Language {
        std::string_view code;
        std::string_view name;
    };

    namespace Detail {
        struct Entry {
            std::string_view key;
            std::string_view text[kMaxLanguages];
        };

        extern const std::string_view kLangCodes[];
        extern const std::string_view kLangNames[];
        extern const std::size_t kLangCount;
        extern const Entry kEntries[];
        extern const std::size_t kEntryCount;
        extern const Entry kTerms[];
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


        std::string fill(std::string_view text, const std::string *args, std::size_t count);
    }

    const std::vector<Language> &languages();


    bool setLanguage(std::string_view code);

    std::string_view language();


    std::string_view trv(std::string_view key);


    const char *trc(std::string_view key);

    std::string tr(std::string_view key);


    template<class... A>
    std::string tr(const std::string_view key, A &&... args) {
        if constexpr (sizeof...(A) == 0) {
            return std::string(trv(key));
        } else {
            const std::string slot[] = {Detail::argText(std::forward<A>(args))...};
            return Detail::fill(trv(key), slot, sizeof...(A));
        }
    }

    bool exists(std::string_view key);


    std::size_t missingKeys();

    void reportMissing();
}
