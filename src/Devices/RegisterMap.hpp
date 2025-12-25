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
    enum class RegisterKeys{ 
        HostIP, // Host IP address key
        MaxKeys
    };

    struct RegisterKeyHash {
        std::size_t operator()(const RegisterKeys& key) const noexcept {
            return static_cast<std::size_t>(key);
        }
    };

    using Value = std::variant<double, bool, std::string>;

    // Singleton access
    static RegisterMap* getInstance();


    // Store or overwrite a value
    void set(RegisterKeys key, Value value) {
        if (key == RegisterKeys::MaxKeys) {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        map_[key] = std::move(value);
    }


    // Retrieve a typed value; returns std::nullopt if missing or wrong type
    template <typename T>
    std::optional<T> get(RegisterKeys key) {
        if (key == RegisterKeys::MaxKeys) {
            return std::nullopt;
        }

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
    void erase(RegisterKeys key) {
        std::lock_guard<std::mutex> lock(mutex_);
        map_.erase(key);
    }

private:
    RegisterMap()  = default;
    ~RegisterMap() = default;

    std::unordered_map<RegisterKeys, Value, RegisterKeyHash> map_;
    std::mutex mutex_;
};

#endif