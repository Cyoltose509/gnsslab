#include "I18n.h"

#include <algorithm>
#include <atomic>
#include <memory>
#include <mutex>
#include <set>

#include "Log.h"

namespace {
    namespace D = I18n::Detail;

    struct Pack {
        std::string code;
        std::vector<std::string> terms;
        std::vector<std::string> text;
    };

    constexpr int kMaxExpandDepth = 10;

    std::mutex gMutex;
    std::vector<std::unique_ptr<Pack> > gPacks;
    std::atomic<const Pack *> gCurrent{nullptr};

    std::atomic<std::size_t> gMissingCount{0};
    std::mutex gMissingMutex;
    std::set<std::string> gMissingLogged;

    int indexOf(const D::Entry *tab, const std::size_t count, const std::string_view key) {
        const D::Entry *end = tab + count;
        const D::Entry *hit = std::lower_bound(tab, end, key,
                                               [](const D::Entry &e, const std::string_view k) {
                                                   return e.key < k;
                                               });
        return hit != end && hit->key == key ? static_cast<int>(hit - tab) : -1;
    }

    int langIndexOf(const std::string_view code) {
        for (std::size_t i = 0; i < D::kLangCount; ++i)
            if (D::kLangCodes[i] == code) return static_cast<int>(i);
        return -1;
    }

    /// 只展开一层：把 {{Key}} 换成术语表里当前的文本，认不出的原样保留。
    std::string expandOnce(const std::string_view text, const std::vector<std::string> &terms) {
        std::string out;
        out.reserve(text.size());
        for (std::size_t i = 0; i < text.size();) {
            if (text[i] == '{' && i + 1 < text.size() && text[i + 1] == '{') {
                if (const std::size_t close = text.find("}}", i + 2); close != std::string_view::npos) {
                    if (const int idx = indexOf(D::kTerms, D::kTermCount, text.substr(i + 2, close - i - 2)); idx >= 0) {
                        out += terms[idx];
                        i = close + 2;
                        continue;
                    }
                }
            }
            out.push_back(text[i]);
            ++i;
        }
        return out;
    }

    void buildPack(Pack &pack, const std::size_t lang) {
        pack.code = std::string(D::kLangCodes[lang]);
        pack.terms.resize(D::kTermCount);
        for (std::size_t i = 0; i < D::kTermCount; ++i)
            pack.terms[i] = std::string(D::kTerms[i].text[lang]);
        // 术语之间可以互相引用，多摊几轮直到不动为止；成环时下一轮文本不变，自然收敛。
        for (int pass = 0; pass < kMaxExpandDepth; ++pass) {
            bool changed = false;
            for (std::size_t i = 0; i < D::kTermCount; ++i) {
                if (std::string next = expandOnce(pack.terms[i], pack.terms); next != pack.terms[i]) {
                    pack.terms[i] = std::move(next);
                    changed = true;
                }
            }
            if (!changed) break;
        }
        pack.text.resize(D::kEntryCount);
        for (std::size_t i = 0; i < D::kEntryCount; ++i)
            pack.text[i] = expandOnce(D::kEntries[i].text[lang], pack.terms);
    }

    /// 当前语言的包；还没设过语言时落到第一列（生成器保证 ZH-CN 在第一列）。
    const Pack *current() {
        if (const Pack *p = gCurrent.load(std::memory_order_acquire)) return p;
        if (D::kLangCount > 0) I18n::setLanguage(D::kLangCodes[0]);
        return gCurrent.load(std::memory_order_acquire);
    }

    void noteMissing(const std::string_view key) {
        gMissingCount.fetch_add(1, std::memory_order_relaxed);
        {
            std::lock_guard lock(gMissingMutex);
            if (!gMissingLogged.insert(std::string(key)).second) return;
        }
        LOG_WARN << "[I18N] missing key: " << key;
    }
}

namespace I18n {
    const std::vector<Language> &languages() {
        static const std::vector<Language> table = [] {
            std::vector<Language> out;
            out.reserve(Detail::kLangCount);
            for (std::size_t i = 0; i < Detail::kLangCount; ++i)
                out.push_back({Detail::kLangCodes[i], Detail::kLangNames[i]});
            return out;
        }();
        return table;
    }

    bool setLanguage(const std::string_view code) {
        const int lang = langIndexOf(code);
        if (lang < 0) return false;
        if (const Pack *cur = gCurrent.load(std::memory_order_acquire); cur && cur->code == code) return true;

        std::lock_guard lock(gMutex);
        for (const auto &pack: gPacks)
            if (pack->code == code) {
                gCurrent.store(pack.get(), std::memory_order_release);
                return true;
            }
        auto pack = std::make_unique<Pack>();
        buildPack(*pack, static_cast<std::size_t>(lang));
        gPacks.push_back(std::move(pack));
        gCurrent.store(gPacks.back().get(), std::memory_order_release);
        return true;
    }

    std::string_view language() {
        const Pack *p = current();
        return p ? std::string_view(p->code) : std::string_view();
    }

    std::string_view trv(const std::string_view key) {
        const Pack *p = current();
        if (!p) return key;
        if (const int i = indexOf(Detail::kEntries, Detail::kEntryCount, key); i >= 0) return p->text[i];
        if (const int t = indexOf(Detail::kTerms, Detail::kTermCount, key); t >= 0) return p->terms[t];
        noteMissing(key);
        return key;
    }

    const char *trc(const std::string_view key) { return trv(key).data(); }

    std::string tr(const std::string_view key) { return std::string(trv(key)); }

    bool exists(const std::string_view key) {
        return indexOf(Detail::kEntries, Detail::kEntryCount, key) >= 0 ||
               indexOf(Detail::kTerms, Detail::kTermCount, key) >= 0;
    }

    std::size_t missingKeys() { return gMissingCount.load(std::memory_order_relaxed); }

    void reportMissing() {
        if (const std::size_t n = missingKeys(); n > 0)
            LOG_WARN << "[I18N] missing keys requested " << n << " time(s); see the WARN lines above";
    }

    namespace Detail {
        std::string fill(const std::string_view text, const std::string *args, const std::size_t count) {
            std::string out;
            out.reserve(text.size() + 16);
            for (std::size_t i = 0; i < text.size();) {
                if (text[i] == '{' && i + 1 < text.size() && text[i + 1] >= '0' && text[i + 1] <= '9') {
                    std::size_t j = i + 1;
                    std::size_t idx = 0;
                    while (j < text.size() && text[j] >= '0' && text[j] <= '9') {
                        idx = idx * 10 + static_cast<std::size_t>(text[j] - '0');
                        ++j;
                    }
                    if (j < text.size() && text[j] == '}' && idx < count) {
                        out += args[idx];
                        i = j + 1;
                        continue;
                    }
                }
                out.push_back(text[i]);
                ++i;
            }
            return out;
        }
    }
}
