#include "core.hpp"
#include "log_storage.hpp"
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unistd.h>
namespace fs = std::filesystem;
static std::string cell(const Json &v) {
    if (v.is_null())
        return "";
    if (v.is_boolean())
        return v.get<bool>() ? "1" : "0";
    if (!v.is_string())
        return v.dump();
    std::string s = v.get<std::string>(), out = "\"";
    for (char c : s) {
        out += c;
        if (c == '"')
            out += '"';
    }
    return out + '"';
}
FlightLog::FlightLog(std::string path) : directory(std::move(path)) {
    info = {{"recording", false},  {"filename", nullptr}, {"points", 0},
            {"dropped_events", 0}, {"error", nullptr},    {"latest_complete", nullptr}};
    fs::create_directories(directory);
    log_storage::recover(directory, ".partial.csv");
    std::string latest;
    for (auto &p : fs::directory_iterator(directory)) {
        auto n = p.path().filename().string();
        if (n.rfind("flight_", 0) == 0 && p.path().extension() == ".csv" &&
            n.find(".partial.") == std::string::npos &&
            n.find(".interrupted.") == std::string::npos && n > latest)
            latest = n;
    }
    if (!latest.empty())
        info["latest_complete"] = latest;
    writer = std::thread(&FlightLog::run, this);
}
FlightLog::~FlightLog() {
    {
        std::lock_guard<std::mutex> l(mutex);
        stopping = true;
    }
    wake.notify_all();
    if (writer.joinable())
        writer.join();
}
Json FlightLog::status() {
    std::lock_guard<std::mutex> l(mutex);
    return info;
}
void FlightLog::submit(const std::string &kind, const Json &s, const Setpoint &target) {
    std::lock_guard<std::mutex> l(mutex);
    if (queue.size() >= 4096) {
        info["dropped_events"] = info["dropped_events"].get<int>() + 1;
        info["error"] = "CSV queue overflow; segment break recorded";
        return;
    }
    queue.push_back({{"kind", kind},
                     {"state", s},
                     {"target", target.json()},
                     {"now", now_s()},
                     {"utc", log_storage::utc()},
                     {"drops", info["dropped_events"]}});
    wake.notify_one();
}
void FlightLog::run() {
    name_thread("flight-csv");
    const std::vector<std::string> fields = {"timestamp",
                                             "target_x",
                                             "target_y",
                                             "target_z",
                                             "actual_x",
                                             "actual_y",
                                             "actual_z",
                                             "vx",
                                             "vy",
                                             "vz",
                                             "point_index",
                                             "utc_iso",
                                             "elapsed_s",
                                             "sample_ms",
                                             "segment",
                                             "break_reason",
                                             "x_m",
                                             "y_m",
                                             "z_m",
                                             "vx_m_s",
                                             "vy_m_s",
                                             "vz_m_s",
                                             "roll_deg",
                                             "pitch_deg",
                                             "yaw_deg",
                                             "mode",
                                             "landed_state",
                                             "reset_counter",
                                             "estimator_flags",
                                             "position_valid",
                                             "range_m",
                                             "flow_quality",
                                             "attitude_age_s",
                                             "odometry_age_s",
                                             "estimator_age_s",
                                             "range_age_s",
                                             "flow_age_s",
                                             "started_mid_session",
                                             "dropped_events",
                                             "monotonic_s", "boot_id", "runtime_session"};
    std::ofstream file;
    fs::path partial;
    bool armed_known = false, was_armed = false, mid = false;
    double begin = 0, heartbeat = 0, last_position = 0, last_flush = 0;
    int64_t last_sample = -1, last_reset = -1, last_drops = 0;
    size_t count = 0, segment = 0;
    auto finish = [&](bool complete) {
        if (!file.is_open())
            return;
        log_storage::checkpoint(file, partial);
        file.close();
        const auto final = log_storage::finish(partial, complete ? "." : ".interrupted.");
        const auto name = final.filename().string();
        std::lock_guard<std::mutex> l(mutex);
        info["recording"] = false;
        info["filename"] = name;
        if (complete)
            info["latest_complete"] = name;
    };
    auto row = [&](const Json &r) {
        for (size_t i = 0; i < fields.size(); i++) {
            if (i)
                file << ',';
            file << cell(r.value(fields[i], Json(nullptr)));
        }
        file << '\n';
        if (!file)
            throw std::runtime_error("CSV write failed");
    };
    while (true) {
        Json e;
        bool exit = false;
        {
            std::unique_lock<std::mutex> l(mutex);
            wake.wait_for(l, std::chrono::milliseconds(200),
                          [&] { return stopping || !queue.empty(); });
            if (!queue.empty()) {
                e = std::move(queue.front());
                queue.pop_front();
            } else
                exit = stopping;
        }
        try {
            if (exit) {
                finish(false);
                break;
            }
            double now = e.is_null() ? now_s() : e["now"].get<double>();
            if (file.is_open() && now_s() - last_flush >= 1) {
                log_storage::checkpoint(file, partial);
                last_flush = now_s();
            }
            if (heartbeat > 0 && now - heartbeat >= 3) {
                finish(false);
                armed_known = false;
                heartbeat = 0;
            }
            if (e.is_null())
                continue;
            const auto &s = e["state"];
            if (e["kind"] == "HEARTBEAT") {
                heartbeat = now;
                bool armed = s.value("armed", false);
                if (armed && !file.is_open()) {
                    partial = log_storage::reserve(directory, "flight_", ".partial.csv");
                    file.open(partial, std::ios::out | std::ios::trunc);
                    if (!file)
                        throw std::runtime_error("Cannot create flight CSV");
                    for (size_t i = 0; i < fields.size(); i++) {
                        if (i)
                            file << ',';
                        file << fields[i];
                    }
                    file << '\n';
                    log_storage::checkpoint(file, partial);
                    begin = now;
                    last_flush = now_s();
                    last_sample = last_reset = -1;
                    last_position = 0;
                    count = segment = 0;
                    mid = !armed_known;
                    last_drops = e["drops"];
                    std::lock_guard<std::mutex> l(mutex);
                    info.update({{"recording", true},
                                 {"filename", partial.filename().string()},
                                 {"points", 0}});
                } else if (!armed)
                    finish(true);
                was_armed = armed;
                armed_known = true;
                continue;
            }
            if (!file.is_open() || !was_armed)
                continue;
            int64_t sample = s.value("position_sample_ms", 0u);
            if (sample == last_sample)
                continue;
            int64_t epoch = s.value("odometry_reset_counter", -1), drops = e["drops"];
            std::string reason;
            if (last_sample < 0)
                reason = "start";
            else {
                if (sample < last_sample)
                    reason += "clock_back|";
                if (epoch != -1 && last_reset != -1 && epoch != last_reset)
                    reason += "estimator_reset|";
                if (now - last_position >= 1)
                    reason += "position_gap|";
                if (drops != last_drops)
                    reason += "queue_drop|";
                if (!reason.empty())
                    segment++;
            }
            Json r = {{"timestamp", s.value("position_sample_ms", 0u) / 1000.},
                      {"point_index", count},
                      {"utc_iso", e["utc"]},
                      {"elapsed_s", now - begin},
                      {"sample_ms", sample},
                      {"segment", segment},
                      {"break_reason", reason},
                      {"mode", s.value("mode", Json(nullptr))},
                      {"landed_state", s.value("landed_state", Json(nullptr))},
                      {"reset_counter", epoch},
                      {"estimator_flags", s.value("estimator_flags", 0)},
                      {"started_mid_session", mid},
                      {"dropped_events", drops}};
            r["monotonic_s"] = e["now"];
            r["boot_id"] = log_storage::boot_id();
            r["runtime_session"] = log_storage::session();
            const auto &p = s["local_position"];
            const char *names[] = {"x", "y", "z", "vx", "vy", "vz"};
            const char *old[] = {"x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s"};
            for (int i = 0; i < 6; i++) {
                r[old[i]] = p.value(names[i], Json(nullptr));
                if (i < 3) {
                    r[std::string("actual_") + names[i]] = r[old[i]];
                    // A reset may arrive just before the control thread clears its old goal.
                    r[std::string("target_") + names[i]] =
                        reason.find("estimator_reset") == std::string::npos &&
                                reason.find("clock_back") == std::string::npos
                            ? e["target"]["position"][i]
                            : Json(nullptr);
                } else
                    r[names[i]] = r[old[i]];
            }
            int flags = s.value("estimator_flags", 0);
            r["position_valid"] = (flags & 46) == 46 && !(flags & 128) && fresh(s, "estimator", 6);
            for (const char *k : {"range_m", "flow_quality", "attitude_age_s", "odometry_age_s",
                                  "estimator_age_s", "range_age_s", "flow_age_s"})
                r[k] = s.value(k, Json(nullptr));
            for (int i = 0; i < 3; i++)
                r[std::array<std::string, 3>{"roll_deg", "pitch_deg", "yaw_deg"}[i]] =
                    fresh(s, "attitude", .5) ? s["attitude_deg"][i] : Json(nullptr);
            row(r);
            count++;
            last_sample = sample;
            last_reset = epoch;
            last_position = now;
            last_drops = drops;
            std::lock_guard<std::mutex> l(mutex);
            info["points"] = count;
        } catch (const std::exception &error) {
            if (file.is_open())
                file.close();
            std::lock_guard<std::mutex> l(mutex);
            info["recording"] = false;
            info["error"] = error.what();
        }
    }
}
