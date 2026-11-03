#pragma once

#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>

/// 产品表的进程内共享缓存。
namespace IoCache {
    struct Key {
        std::string path;
        long long mtime = 0;
        long long size = 0;

        bool operator<(const Key &o) const {
            if (path != o.path) return path < o.path;
            if (mtime != o.mtime) return mtime < o.mtime;
            return size < o.size;
        }
    };


    inline Key keyOf(const std::string &path) {
        Key k;
        std::error_code ec;
        const std::filesystem::path p = std::filesystem::absolute(path, ec);
        k.path = ec ? path : p.lexically_normal().string();
        if (std::filesystem::is_regular_file(p, ec)) {
            k.size = static_cast<long long>(std::filesystem::file_size(p, ec));
            if (ec) k.size = 0;
            const auto ft = std::filesystem::last_write_time(p, ec);
            if (!ec) k.mtime = ft.time_since_epoch().count();
        }
        return k;
    }

    template<class T>
    std::map<Key, std::shared_ptr<const T> > &entriesOf() {
        static std::map<Key, std::shared_ptr<const T> > m;
        return m;
    }

    template<class>
    std::mutex &mutexOf() {
        static std::mutex m;
        return m;
    }


    template<class T>
    std::shared_ptr<const T> find(const Key &k) {
        std::lock_guard<std::mutex> lock(mutexOf<T>());
        const auto &m = entriesOf<T>();
        const auto it = m.find(k);
        return it == m.end() ? nullptr : it->second;
    }



    template<class T>
    void keep(const Key &k, std::shared_ptr<const T> v) {
        if (!v) return;
        std::lock_guard<std::mutex> lock(mutexOf<T>());
        entriesOf<T>().emplace(k, std::move(v));
    }


    template<class T>
    void drop(const Key &k) {
        std::lock_guard<std::mutex> lock(mutexOf<T>());
        entriesOf<T>().erase(k);
    }

    template<class T>
    int count() {
        std::lock_guard<std::mutex> lock(mutexOf<T>());
        return static_cast<int>(entriesOf<T>().size());
    }
}
