#include "VideoStereoCalibration.hpp"
#include "utils/logger.hpp"

#include <filesystem>
#include <fstream>
#include <algorithm>
#include <optional>
#include <nlohmann/json.hpp>


namespace Vision {
namespace {

static cv::Mat toGray(const cv::Mat& bgrOrBgraOrGray) {
    cv::Mat gray;
    if (bgrOrBgraOrGray.empty()) return gray;
    if (bgrOrBgraOrGray.channels() == 1) {
        gray = bgrOrBgraOrGray;
    } else if (bgrOrBgraOrGray.channels() == 4) {
        cv::cvtColor(bgrOrBgraOrGray, gray, cv::COLOR_BGRA2GRAY);
    } else {
        cv::cvtColor(bgrOrBgraOrGray, gray, cv::COLOR_BGR2GRAY);
    }
    return gray;
}

template <size_t N>
static void writeRowMajor(const cv::Mat& m, std::array<double, N>& out) {
    if (m.empty()) return;
    const int rows = m.rows;
    const int cols = m.cols;
    if (rows * cols != static_cast<int>(N)) return;

    cv::Mat as64;
    if (m.type() == CV_64F) {
        as64 = m;
    } else {
        m.convertTo(as64, CV_64F);
    }

    size_t k = 0;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            out[k++] = as64.at<double>(r, c);
        }
    }
}

static std::vector<cv::Point3f> makeChessboardObjectPoints(cv::Size boardSize, double squareSize) {
    std::vector<cv::Point3f> obj;
    obj.reserve(static_cast<size_t>(boardSize.area()));
    for (int y = 0; y < boardSize.height; ++y) {
        for (int x = 0; x < boardSize.width; ++x) {
            obj.emplace_back(static_cast<float>(x * squareSize),
                             static_cast<float>(y * squareSize),
                             0.0f);
        }
    }
    return obj;
}

static void replaceAll(std::string& s, const std::string& from, const std::string& to) {
    if (from.empty()) return;
    std::string::size_type pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) {
        s.replace(pos, from.size(), to);
        pos += to.size();
    }
}

static std::optional<nlohmann::json> parseJsonLenient(std::string text) {
    // 1) Try strict JSON parse first.
    try {
        return nlohmann::json::parse(text);
    } catch (...) {
    }

    // 2) Attempt to normalize common "python dict" formatting:
    //    - single quotes -> double quotes
    //    - True/False -> true/false
    //    - None -> null
    replaceAll(text, "True", "true");
    replaceAll(text, "False", "false");
    replaceAll(text, "None", "null");
    std::replace(text.begin(), text.end(), '\'', '"');

    try {
        return nlohmann::json::parse(text);
    } catch (...) {
        return std::nullopt;
    }
}
} // namespace

VideoStereoCalib::VideoStereoCalib() {
    if (loadCalibrationProfile(calibStoragePath) != 0) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Stereo calibration profile not loaded.\n");
    }
}

bool VideoStereoCalib::buildRectifyMaps_() {
    if (profileSettings_.imageWidth == 0 || profileSettings_.imageHeight == 0) {
        return false;
    }

    m_RectifySize = cv::Size(static_cast<int>(profileSettings_.imageWidth),
                             static_cast<int>(profileSettings_.imageHeight));

    // Construct cv::Mat headers over the stored arrays.
    cv::Mat K1(3, 3, CV_64F, profileSettings_.K_left.data());
    cv::Mat D1(1, static_cast<int>(CalibrationProfileSettings::DSize), CV_64F, profileSettings_.D_left.data());
    cv::Mat K2(3, 3, CV_64F, profileSettings_.K_right.data());
    cv::Mat D2(1, static_cast<int>(CalibrationProfileSettings::DSize), CV_64F, profileSettings_.D_right.data());

    cv::Mat R1(3, 3, CV_64F, profileSettings_.R1.data());
    cv::Mat R2(3, 3, CV_64F, profileSettings_.R2.data());

    cv::Mat P1full(3, 4, CV_64F, profileSettings_.P1.data());
    cv::Mat P2full(3, 4, CV_64F, profileSettings_.P2.data());
    // initUndistortRectifyMap expects a 3x3 "new camera matrix"; use the left 3x3 of P1/P2.
    cv::Mat P1 = P1full(cv::Rect(0, 0, 3, 3));
    cv::Mat P2 = P2full(cv::Rect(0, 0, 3, 3));

    try {
        cv::initUndistortRectifyMap(K1, D1, R1, P1, m_RectifySize, CV_16SC2, map1L, map2L);
        cv::initUndistortRectifyMap(K2, D2, R2, P2, m_RectifySize, CV_16SC2, map1R, map2R);
    } catch (const cv::Exception& e) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to build rectify maps: %s\n", e.what());
        map1L.release();
        map2L.release();
        map1R.release();
        map2R.release();
        m_RectifySize = {};
        return false;
    }

    return !map1L.empty() && !map2L.empty() && !map1R.empty() && !map2R.empty();
}

int VideoStereoCalib::configureFromJson(const std::string& jsonStr) {
    const auto parsed = parseJsonLenient(jsonStr);
    if (!parsed || !parsed->is_object()) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Calibration config is not valid JSON\n");
        return -1;
    }
    const nlohmann::json& root = *parsed;

    // output_view.show_overlays
    if (root.contains("output_view") && root["output_view"].is_object()) {
        const auto& view = root["output_view"];
        if (view.contains("show_overlays") && view["show_overlays"].is_boolean()) {
            calibReq_.showOverlays = view["show_overlays"].get<bool>();
        }
    }

    // capture.required_samples
    if (root.contains("capture") && root["capture"].is_object()) {
        const auto& cap = root["capture"];
        if (cap.contains("required_samples") && cap["required_samples"].is_number_integer()) {
            const int v = cap["required_samples"].get<int>();
            if (v > 0) calibReq_.requiredSamples = static_cast<std::size_t>(v);
        } else if (cap.contains("required_samples") && cap["required_samples"].is_number_unsigned()) {
            calibReq_.requiredSamples = static_cast<std::size_t>(cap["required_samples"].get<unsigned>());
        }
    }

    // target.pattern.cols/rows, target.square_size.value/units
    if (root.contains("target") && root["target"].is_object()) {
        const auto& tgt = root["target"];

        if (tgt.contains("pattern") && tgt["pattern"].is_object()) {
            const auto& pat = tgt["pattern"];
            int cols = calibReq_.boardSize.width;
            int rows = calibReq_.boardSize.height;
            if (pat.contains("cols") && pat["cols"].is_number_integer()) cols = pat["cols"].get<int>();
            if (pat.contains("rows") && pat["rows"].is_number_integer()) rows = pat["rows"].get<int>();
            if (cols > 0 && rows > 0) {
                calibReq_.boardSize = cv::Size(cols, rows);
            }
        }

        if (tgt.contains("square_size") && tgt["square_size"].is_object()) {
            const auto& ss = tgt["square_size"];
            double value = 0.0;
            std::string units = "mm";
            if (ss.contains("value") && ss["value"].is_number()) value = ss["value"].get<double>();
            if (ss.contains("units") && ss["units"].is_string()) units = ss["units"].get<std::string>();

            if (value > 0.0) {
                if (units == "mm" || units == "millimeter" || units == "millimeters") {
                    calibReq_.squareSizeMeters = value / 1000.0;
                } else if (units == "m" || units == "meter" || units == "meters") {
                    calibReq_.squareSizeMeters = value;
                } else if (units == "cm" || units == "centimeter" || units == "centimeters") {
                    calibReq_.squareSizeMeters = value / 100.0;
                } else {
                    Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN,
                                                 "Unknown square_size.units '%s', assuming mm\n",
                                                 units.c_str());
                    calibReq_.squareSizeMeters = value / 1000.0;
                }
            }
        }
    }

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO,
                                 "Calibration config: board=%dx%d square=%.6fm samples=%zu overlays=%d\n",
                                 calibReq_.boardSize.width,
                                 calibReq_.boardSize.height,
                                 calibReq_.squareSizeMeters,
                                 calibReq_.requiredSamples,
                                 calibReq_.showOverlays ? 1 : 0);
    return 0;
}

int VideoStereoCalib::DoCalibration(cv::Mat& leftBgr, cv::Mat& rightBgr) {
    return DoCalibration(leftBgr,
                         rightBgr,
                         calibReq_.boardSize,
                         calibReq_.squareSizeMeters,
                         calibReq_.requiredSamples,
                         calibReq_.showOverlays);
}

void VideoStereoCalib::resetCalibrationSession() {
    m_ImagePointsL.clear();
    m_ImagePointsR.clear();
    m_ObjectPoints.clear();
    m_ImageSize = {};
    m_Calibrated = false;
    m_LastRmsError = 0.0;
}

std::size_t VideoStereoCalib::numCollectedSamples() const {
    return m_ImagePointsL.size();
}

bool VideoStereoCalib::isCalibrated() const {
    return m_Calibrated;
}

double VideoStereoCalib::lastRmsError() const {
    return m_LastRmsError;
}

std::optional<double> VideoStereoCalib::depthMetersFromDisparity(double disparityPx) const {
    if (!(disparityPx > 0.0)) return std::nullopt;
    // In OpenCV stereoRectify, P2(0,3) = Tx where Tx = -f * B (in pixel units).
    // Depth Z = (f*B)/d = (-Tx)/d. Units follow the units used during calibration (we use meters).
    const double Tx = profileSettings_.P2[3];
    if (Tx == 0.0) return std::nullopt;
    const double z = (-Tx) / disparityPx;
    if (!(z > 0.0) || !std::isfinite(z)) return std::nullopt;
    return z;
}

double VideoStereoCalib::focalLengthPx() const {
    // P1 is stored row-major 3x4, so P1(0,0) is index 0.
    return profileSettings_.P1[0];
}

cv::Size VideoStereoCalib::imageSize() const {
    return cv::Size(static_cast<int>(profileSettings_.imageWidth),
                    static_cast<int>(profileSettings_.imageHeight));
}

bool VideoStereoCalib::hasRectification() const {
    return m_HaveRectifyMaps;
}

cv::Matx33d VideoStereoCalib::leftK() const {
    const double* k = profileSettings_.K_left.data();
    return cv::Matx33d(k[0], k[1], k[2],
                       k[3], k[4], k[5],
                       k[6], k[7], k[8]);
}

cv::Matx33d VideoStereoCalib::rightK() const {
    const double* k = profileSettings_.K_right.data();
    return cv::Matx33d(k[0], k[1], k[2],
                       k[3], k[4], k[5],
                       k[6], k[7], k[8]);
}

cv::Matx33d VideoStereoCalib::stereoR() const {
    const double* r = profileSettings_.R.data();
    return cv::Matx33d(r[0], r[1], r[2],
                       r[3], r[4], r[5],
                       r[6], r[7], r[8]);
}

cv::Vec3d VideoStereoCalib::stereoT() const {
    const double* t = profileSettings_.T.data();
    return cv::Vec3d(t[0], t[1], t[2]);
}

double VideoStereoCalib::baselineMeters() const {
    return std::abs(profileSettings_.T[0]);
}

cv::Matx33d VideoStereoCalib::rectifyR1() const {
    const double* r = profileSettings_.R1.data();
    return cv::Matx33d(r[0], r[1], r[2],
                       r[3], r[4], r[5],
                       r[6], r[7], r[8]);
}

cv::Matx33d VideoStereoCalib::rectifyR2() const {
    const double* r = profileSettings_.R2.data();
    return cv::Matx33d(r[0], r[1], r[2],
                       r[3], r[4], r[5],
                       r[6], r[7], r[8]);
}

cv::Matx34d VideoStereoCalib::projectionP1() const {
    const double* p = profileSettings_.P1.data();
    return cv::Matx34d(p[0], p[1], p[2], p[3],
                       p[4], p[5], p[6], p[7],
                       p[8], p[9], p[10], p[11]);
}

cv::Matx34d VideoStereoCalib::projectionP2() const {
    const double* p = profileSettings_.P2.data();
    return cv::Matx34d(p[0], p[1], p[2], p[3],
                       p[4], p[5], p[6], p[7],
                       p[8], p[9], p[10], p[11]);
}

cv::Matx44d VideoStereoCalib::reprojectionQ() const {
    const auto& q = profileSettings_.Q;
    return cv::Matx44d(q[0],  q[1],  q[2],  q[3],
                       q[4],  q[5],  q[6],  q[7],
                       q[8],  q[9],  q[10], q[11],
                       q[12], q[13], q[14], q[15]);
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
        out << defaultProfile.dump(2);
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

    // Current format: the file itself is a single profile object.
    // Backward compatibility: if the file has a "profiles" object, pick a reasonable entry and migrate.
    nlohmann::json profile;
    if (config.is_object() && config.contains("profiles") && config["profiles"].is_object()) {
        const auto& profilesObj = config["profiles"];
        if (profilesObj.contains("default")) {
            profile = profilesObj["default"];
        } else if (!profilesObj.empty()) {
            // Pick the first profile entry.
            profile = profilesObj.begin().value();
        } else {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                         "Calibration profile file has empty 'profiles' object: %s\n",
                                         profilePath.string().c_str());
            return -1;
        }

        // Migrate on disk to single-profile format (best-effort).
        try {
            std::ofstream out(profilePath, std::ios::out | std::ios::trunc);
            if (out.is_open()) {
                out << profile.dump(2);
                out.flush();
            }
        } catch (...) {
        }
    } else {
        profile = config;
    }

    if (!profile.is_object()) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                     "Calibration profile is not a JSON object: %s\n",
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

    m_HaveRectifyMaps = buildRectifyMaps_();

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO,
                                 "Loaded stereo calibration profile (%ux%u)\n",
                                 profileSettings_.imageWidth,
                                 profileSettings_.imageHeight);

    return 0;
}

int VideoStereoCalib::storeCalibrationProfile() {
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

    std::ofstream out(profilePath, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                     "Failed to write calibration profile: %s\n",
                                     profilePath.string().c_str());
        return -1;
    }
    out << profile.dump(2);
    out.flush();

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Stored calibration profile\n");
    return 0;
}

int VideoStereoCalib::DetectChessBoard(cv::Mat& leftBgr, cv::Mat& rightBgr, cv::Size boardSize) {
    if (leftBgr.empty() || rightBgr.empty()) {
        return -1;
    }
    
    cv::Mat leftGray = toGray(leftBgr);
    cv::Mat rightGray = toGray(rightBgr);
    std::vector<cv::Point2f> cornersL, cornersR;

    bool foundL = cv::findChessboardCorners(leftGray, boardSize, cornersL);
    bool foundR = cv::findChessboardCorners(rightGray, boardSize, cornersR);

    // Draw the chessboard corners
    cv::drawChessboardCorners(leftBgr, boardSize, cornersL, foundL);
    cv::drawChessboardCorners(rightBgr, boardSize, cornersR, foundR);

    const std::string status = (foundL && foundR) ? "CALIB: OK" : "CALIB: NO DETECTION";
    cv::putText(leftBgr, status, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0,255,0}, 2);
    cv::putText(rightBgr, status, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0,255,0}, 2);

    return 0;
}

nlohmann::json& VideoStereoCalib::getCurrentCalibrationStats() {
    return m_CalibrationStatsBank;
}

void VideoStereoCalib::rectify(const cv::Mat& leftIn, const cv::Mat& rightIn,
                 cv::Mat& leftRect, cv::Mat& rightRect) const {
    if (!m_HaveRectifyMaps || map1L.empty() || map2L.empty() || map1R.empty() || map2R.empty()) {
        leftRect = leftIn;
        rightRect = rightIn;
        return;
    }
    if (leftIn.empty() || rightIn.empty()) {
        leftRect = leftIn;
        rightRect = rightIn;
        return;
    }
    if (leftIn.size() != m_RectifySize || rightIn.size() != m_RectifySize) {
        leftRect = leftIn;
        rightRect = rightIn;
        return;
    }

    cv::remap(leftIn, leftRect, map1L, map2L, cv::INTER_LINEAR);
    cv::remap(rightIn, rightRect, map1R, map2R, cv::INTER_LINEAR);
} 

int VideoStereoCalib::DoCalibration(cv::Mat& leftBgr,
                                    cv::Mat& rightBgr,
                                    cv::Size boardSize,
                                    double squareSize,
                                    std::size_t targetSamples,
                                    bool drawOverlay) {
    if (leftBgr.empty() || rightBgr.empty()) {
        return -1;
    }
    if (leftBgr.size() != rightBgr.size()) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Stereo frames have different sizes (L=%dx%d R=%dx%d)\n",
                                     leftBgr.cols, leftBgr.rows, rightBgr.cols, rightBgr.rows);
        return -1;
    }
    if (boardSize.width <= 0 || boardSize.height <= 0) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Invalid chessboard size (%d x %d)\n",
                                     boardSize.width, boardSize.height);
        return -1;
    }
    if (!(squareSize > 0.0)) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Invalid square size: %f\n", squareSize);
        return -1;
    }
    if (targetSamples < 5) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "targetSamples too small: %zu\n", targetSamples);
        return -1;
    }

    if (m_Calibrated) {
        return 0;
    }

    cv::Mat leftGray = toGray(leftBgr);
    cv::Mat rightGray = toGray(rightBgr);

    std::vector<cv::Point2f> cornersL, cornersR;
    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    bool foundL = cv::findChessboardCorners(leftGray, boardSize, cornersL, flags);
    bool foundR = cv::findChessboardCorners(rightGray, boardSize, cornersR, flags);

    if (foundL) {
        cv::cornerSubPix(leftGray, cornersL, {11, 11}, {-1, -1},
                         {cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01});
    }
    if (foundR) {
        cv::cornerSubPix(rightGray, cornersR, {11, 11}, {-1, -1},
                         {cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01});
    }

    if (drawOverlay) {
        cv::drawChessboardCorners(leftBgr, boardSize, cornersL, foundL);
        cv::drawChessboardCorners(rightBgr, boardSize, cornersR, foundR);

        const std::string status = (foundL && foundR) ? "CALIB: OK" : "CALIB: NO DETECTION";
        const std::string count = "Samples: " + std::to_string(m_ImagePointsL.size()) + "/" + std::to_string(targetSamples);
        cv::putText(leftBgr, status, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
        cv::putText(leftBgr, count, {20, 80}, cv::FONT_HERSHEY_SIMPLEX, 0.9, {0, 255, 0}, 2);
        cv::putText(rightBgr, status, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
        cv::putText(rightBgr, count, {20, 80}, cv::FONT_HERSHEY_SIMPLEX, 0.9, {0, 255, 0}, 2);

        // Set calibration stats
        m_CalibrationStatsBank["count"                 ] = count;
        m_CalibrationStatsBank["status"                ] = status;
    }

    if (!(foundL && foundR)) {
        return 1; // processed frame, no sample captured
    }

    // Capture sample.
    if (m_ImagePointsL.empty()) {
        m_ImageSize = leftBgr.size();
    }
    if (leftBgr.size() != m_ImageSize) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Frame size changed during calibration; resetting session\n");
        resetCalibrationSession();
        m_ImageSize = leftBgr.size();
    }

    m_ImagePointsL.push_back(cornersL);
    m_ImagePointsR.push_back(cornersR);
    m_ObjectPoints.push_back(makeChessboardObjectPoints(boardSize, squareSize));

    if (m_ImagePointsL.size() < targetSamples) {
        return 1; // still collecting
    }

    // Compute calibration.
    cv::Mat K1 = cv::initCameraMatrix2D(m_ObjectPoints, m_ImagePointsL, m_ImageSize, 0);
    cv::Mat K2 = cv::initCameraMatrix2D(m_ObjectPoints, m_ImagePointsR, m_ImageSize, 0);
    cv::Mat D1 = cv::Mat::zeros(5, 1, CV_64F);
    cv::Mat D2 = cv::Mat::zeros(5, 1, CV_64F);
    cv::Mat R, T, E, F;

    const int calibFlags = cv::CALIB_USE_INTRINSIC_GUESS;
    const cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6);
    m_LastRmsError = cv::stereoCalibrate(m_ObjectPoints,
                                        m_ImagePointsL,
                                        m_ImagePointsR,
                                        K1, D1, K2, D2,
                                        m_ImageSize,
                                        R, T, E, F,
                                        calibFlags,
                                        criteria);

    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K1, D1, K2, D2, m_ImageSize, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, m_ImageSize);

    profileSettings_.imageWidth = static_cast<uint32_t>(m_ImageSize.width);
    profileSettings_.imageHeight = static_cast<uint32_t>(m_ImageSize.height);

    writeRowMajor(K1, profileSettings_.K_left);
    writeRowMajor(D1.reshape(1, 1), profileSettings_.D_left);
    writeRowMajor(K2, profileSettings_.K_right);
    writeRowMajor(D2.reshape(1, 1), profileSettings_.D_right);
    writeRowMajor(R, profileSettings_.R);
    writeRowMajor(T.reshape(1, 1), profileSettings_.T);
    writeRowMajor(R1, profileSettings_.R1);
    writeRowMajor(R2, profileSettings_.R2);
    writeRowMajor(P1, profileSettings_.P1);
    writeRowMajor(P2, profileSettings_.P2);
    writeRowMajor(Q, profileSettings_.Q);

    m_Calibrated = true;
    m_HaveRectifyMaps = buildRectifyMaps_();
    
    m_CalibrationStatsBank["board_width"           ] = boardSize.width;
    m_CalibrationStatsBank["board_height"          ] = boardSize.height;
    m_CalibrationStatsBank["square_size_meters"    ] = squareSize;
    m_CalibrationStatsBank["found_left"            ] = foundL;
    m_CalibrationStatsBank["found_right"           ] = foundR;
    m_CalibrationStatsBank["num_collected_samples" ] = m_ImagePointsL.size();
    m_CalibrationStatsBank["rms_error"             ] = m_LastRmsError;
    m_CalibrationStatsBank["image_width"           ] = m_ImageSize.width;
    m_CalibrationStatsBank["image_height"          ] = m_ImageSize.height;
    m_CalibrationStatsBank["calibrated"            ] = m_Calibrated;

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO,
                                 "Stereo calibration complete: samples=%zu rms=%.6f\n",
                                 m_ObjectPoints.size(),
                                 m_LastRmsError);
    return 0;
}

}
