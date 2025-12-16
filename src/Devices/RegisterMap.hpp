/**
 * @file RegisterMap.hpp
 * @brief Thread-safe global register map that can store heterogeneous data.
 */

#ifndef REGISTERMAP_HPP
#define REGISTERMAP_HPP

#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>

class RegisterMap {
public:
    using Value = std::variant<int64_t, double, bool, std::string>;

    enum { 
        HostIP  // Host IP address key
    };

    // Singleton access
    static RegisterMap* getInstance();


    // Store or overwrite a value
    void set(const std::string& key, Value value) {
        std::lock_guard<std::mutex> lock(mutex_);
        map_[key] = std::move(value);
    }


    // Retrieve a typed value; returns std::nullopt if missing or wrong type
    template <typename T>
    std::optional<T> get(const std::string& key) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = map_.find(key);
        if (it == map_.end()) {
            return std::nullopt;
        }
        if (auto val = std::get_if<T>(&it->second)) {
            return *val;
        }
        return std::nullopt;
    }

    // Remove a key if present
    void erase(const std::string& key) {
        std::lock_guard<std::mutex> lock(mutex_);
        map_.erase(key);
    }

private:
    RegisterMap()  = default;
    ~RegisterMap() = default;

    std::unordered_map<std::string, Value> map_;
    std::mutex mutex_;
};

#endif