#include "VideoStereoCalibration.hpp"
#include "utils/logger.hpp"

#include <filesystem>
#include <fstream>
#include <optional>
#include <nlohmann/json.hpp>


namespace Vision {
VideoStereoCalib::VideoStereoCalib() {
    if (loadCalibrationProfile(calibStoragePath) != 0) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Stereo calibration profile not loaded.\n");
    }
}


int VideoStereoCalib::loadCalibrationProfile(const char* path) {
    if (!path || path[0] == '\0') {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Invalid calibration profile path\n");
        return -1;
    }

    const std::filesystem::path profilePath(path);
    const std::filesystem::path parentDir = profilePath.has_parent_path() ? profilePath.parent_path() : std::filesystem::path{};

    if (!parentDir.empty() && !std::filesystem::exists(parentDir)) {
        std::error_code ec;
        std::filesystem::create_directories(parentDir, ec);
        if (ec) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                         "Failed to create calibration profile directory: %s\n",
                                         parentDir.string().c_str());
            return -1;
        }
    }

    auto readArrayDoubles = [](const nlohmann::json& j, size_t expected) -> std::optional<std::vector<double>> {
        if (!j.is_array() || j.size() != expected) return std::nullopt;
        std::vector<double> out;
        out.reserve(expected);
        for (size_t i = 0; i < expected; ++i) {
            if (!j[i].is_number()) return std::nullopt;
            out.push_back(j[i].get<double>());
        }
        return out;
    };

    auto readMatrixDoubles = [&](const nlohmann::json& j, size_t rows, size_t cols) -> std::optional<std::vector<double>> {
        if (!j.is_array() || j.size() != rows) return std::nullopt;
        std::vector<double> out;
        out.reserve(rows * cols);
        for (size_t r = 0; r < rows; ++r) {
            if (!j[r].is_array() || j[r].size() != cols) return std::nullopt;
            for (size_t c = 0; c < cols; ++c) {
                if (!j[r][c].is_number()) return std::nullopt;
                out.push_back(j[r][c].get<double>());
            }
        }
        return out;
    };

    auto applyVec = [](auto& dstArray, const std::vector<double>& src) {
        if (src.size() != dstArray.size()) return;
        for (size_t i = 0; i < dstArray.size(); ++i) {
            dstArray[i] = src[i];
        }
    };

    nlohmann::json config;
    if (!std::filesystem::exists(profilePath)) {
        std::ofstream out(profilePath, std::ios::out | std::ios::trunc);
        if (!out.is_open()) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                         "Failed to create calibration profile: %s\n",
                                         profilePath.string().c_str());
            return -1;
        }

        nlohmann::json defaultProfile = nlohmann::json::parse(std::string(DefaultProfileJson));
        config["active-profile-name"] = "default";
        config["profiles"] = nlohmann::json::object();
        config["profiles"]["default"] = defaultProfile;

        out << config.dump(2);
        out.flush();
        Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO,
                                     "Created default calibration profile: %s\n",
                                     profilePath.string().c_str());
    }

    std::ifstream profileFile(profilePath);
    if (!profileFile.is_open()) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                     "Failed to open calibration profile: %s\n",
                                     profilePath.string().c_str());
        return -1;
    }

    try {
        profileFile >> config;
    } catch (const std::exception& e) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                     "Failed to parse calibration profile container %s: %s\n",
                                     profilePath.string().c_str(),
                                     e.what());
        return -1;
    }

    if (!config.is_object()) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                     "Calibration profile container is not a JSON object: %s\n",
                                     profilePath.string().c_str());
        return -1;
    }

    if (!config.contains("active-profile-name") || !config["active-profile-name"].is_string()) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                     "Calibration profile missing 'active-profile-name': %s\n",
                                     profilePath.string().c_str());
        return -1;
    }
    const std::string activeName = config["active-profile-name"].get<std::string>();

    if (!config.contains("profiles") || !config["profiles"].is_object() || !config["profiles"].contains(activeName)) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                     "Calibration profile missing active profile '%s': %s\n",
                                     activeName.c_str(),
                                     profilePath.string().c_str());
        return -1;
    }

    const nlohmann::json& profile = config["profiles"][activeName];
    if (!profile.is_object()) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                     "Active profile '%s' is not a JSON object: %s\n",
                                     activeName.c_str(),
                                     profilePath.string().c_str());
        return -1;
    }

    // Defaults already set in profileSettings_. Only overwrite fields present and well-formed.
    if (profile.contains("version") && profile["version"].is_number_unsigned()) {
        profileSettings_.version = profile["version"].get<uint32_t>();
    }
    if (profile.contains("model") && profile["model"].is_string()) {
        profileSettings_.model = profile["model"].get<std::string>();
    }

    if (profile.contains("image_size") && profile["image_size"].is_object()) {
        const auto& s = profile["image_size"];
        if (s.contains("width") && s["width"].is_number_unsigned()) profileSettings_.imageWidth = s["width"].get<uint32_t>();
        if (s.contains("height") && s["height"].is_number_unsigned()) profileSettings_.imageHeight = s["height"].get<uint32_t>();
    }

    if (profile.contains("left") && profile["left"].is_object()) {
        const auto& left = profile["left"];
        if (left.contains("K")) {
            if (auto v = readMatrixDoubles(left["K"], 3, 3)) applyVec(profileSettings_.K_left, *v);
        }
        if (left.contains("D")) {
            if (auto v = readArrayDoubles(left["D"], CalibrationProfileSettings::DSize)) applyVec(profileSettings_.D_left, *v);
        }
    }

    if (profile.contains("right") && profile["right"].is_object()) {
        const auto& right = profile["right"];
        if (right.contains("K")) {
            if (auto v = readMatrixDoubles(right["K"], 3, 3)) applyVec(profileSettings_.K_right, *v);
        }
        if (right.contains("D")) {
            if (auto v = readArrayDoubles(right["D"], CalibrationProfileSettings::DSize)) applyVec(profileSettings_.D_right, *v);
        }
    }

    if (profile.contains("stereo") && profile["stereo"].is_object()) {
        const auto& stereo = profile["stereo"];
        if (stereo.contains("R")) {
            if (auto v = readMatrixDoubles(stereo["R"], 3, 3)) applyVec(profileSettings_.R, *v);
        }
        if (stereo.contains("T")) {
            if (auto v = readArrayDoubles(stereo["T"], 3)) applyVec(profileSettings_.T, *v);
        }
    }

    if (profile.contains("rectify") && profile["rectify"].is_object()) {
        const auto& rect = profile["rectify"];
        if (rect.contains("R1")) {
            if (auto v = readMatrixDoubles(rect["R1"], 3, 3)) applyVec(profileSettings_.R1, *v);
        }
        if (rect.contains("R2")) {
            if (auto v = readMatrixDoubles(rect["R2"], 3, 3)) applyVec(profileSettings_.R2, *v);
        }
        if (rect.contains("P1")) {
            if (auto v = readMatrixDoubles(rect["P1"], 3, 4)) applyVec(profileSettings_.P1, *v);
        }
        if (rect.contains("P2")) {
            if (auto v = readMatrixDoubles(rect["P2"], 3, 4)) applyVec(profileSettings_.P2, *v);
        }
        if (rect.contains("Q")) {
            if (auto v = readMatrixDoubles(rect["Q"], 4, 4)) applyVec(profileSettings_.Q, *v);
        }
    }

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO,
                                 "Loaded stereo calibration profile '%s' (%ux%u)\n",
                                 activeName.c_str(),
                                 profileSettings_.imageWidth,
                                 profileSettings_.imageHeight);

    return 0;
}

int VideoStereoCalib::storeCalibrationProfile(const char* profileName) {
    if (!profileName || profileName[0] == '\0') {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Invalid profile name\n");
        return -1;
    }

    auto arr3 = [](const std::array<double, 3>& a) {
        return nlohmann::json::array({a[0], a[1], a[2]});
    };
    auto mat3x3 = [](const std::array<double, 9>& m) {
        return nlohmann::json::array(
            {nlohmann::json::array({m[0], m[1], m[2]}),
             nlohmann::json::array({m[3], m[4], m[5]}),
             nlohmann::json::array({m[6], m[7], m[8]})});
    };
    auto mat3x4 = [](const std::array<double, 12>& m) {
        return nlohmann::json::array(
            {nlohmann::json::array({m[0], m[1], m[2], m[3]}),
             nlohmann::json::array({m[4], m[5], m[6], m[7]}),
             nlohmann::json::array({m[8], m[9], m[10], m[11]})});
    };
    auto mat4x4 = [](const std::array<double, 16>& m) {
        return nlohmann::json::array(
            {nlohmann::json::array({m[0], m[1], m[2], m[3]}),
             nlohmann::json::array({m[4], m[5], m[6], m[7]}),
             nlohmann::json::array({m[8], m[9], m[10], m[11]}),
             nlohmann::json::array({m[12], m[13], m[14], m[15]})});
    };

    nlohmann::json profile;
    profile["version"] = profileSettings_.version;
    profile["model"] = profileSettings_.model;
    profile["image_size"] = {{"width", profileSettings_.imageWidth}, {"height", profileSettings_.imageHeight}};

    profile["left"] = nlohmann::json::object();
    profile["left"]["K"] = mat3x3(profileSettings_.K_left);
    profile["left"]["D"] = nlohmann::json::array();
    for (double v : profileSettings_.D_left) profile["left"]["D"].push_back(v);

    profile["right"] = nlohmann::json::object();
    profile["right"]["K"] = mat3x3(profileSettings_.K_right);
    profile["right"]["D"] = nlohmann::json::array();
    for (double v : profileSettings_.D_right) profile["right"]["D"].push_back(v);

    profile["stereo"] = nlohmann::json::object();
    profile["stereo"]["R"] = mat3x3(profileSettings_.R);
    profile["stereo"]["T"] = arr3(profileSettings_.T);

    profile["rectify"] = nlohmann::json::object();
    profile["rectify"]["R1"] = mat3x3(profileSettings_.R1);
    profile["rectify"]["R2"] = mat3x3(profileSettings_.R2);
    profile["rectify"]["P1"] = mat3x4(profileSettings_.P1);
    profile["rectify"]["P2"] = mat3x4(profileSettings_.P2);
    profile["rectify"]["Q"] = mat4x4(profileSettings_.Q);

    const std::filesystem::path profilePath(calibStoragePath);
    const std::filesystem::path parentDir = profilePath.has_parent_path() ? profilePath.parent_path() : std::filesystem::path{};
    if (!parentDir.empty() && !std::filesystem::exists(parentDir)) {
        std::error_code ec;
        std::filesystem::create_directories(parentDir, ec);
        if (ec) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                         "Failed to create calibration profile directory: %s\n",
                                         parentDir.string().c_str());
            return -1;
        }
    }

    nlohmann::json config;
    if (std::filesystem::exists(profilePath)) {
        std::ifstream in(profilePath);
        if (in.is_open()) {
            try {
                in >> config;
            } catch (...) {
                config = nlohmann::json::object();
            }
        }
    }

    if (!config.is_object()) {
        config = nlohmann::json::object();
    }
    if (!config.contains("profiles") || !config["profiles"].is_object()) {
        config["profiles"] = nlohmann::json::object();
    }

    config["profiles"][profileName] = profile;
    config["active-profile-name"] = std::string(profileName);

    std::ofstream out(profilePath, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                     "Failed to write calibration profile: %s\n",
                                     profilePath.string().c_str());
        return -1;
    }
    out << config.dump(2);
    out.flush();

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Stored calibration profile: %s\n", profileName);
    return 0;
}
}
