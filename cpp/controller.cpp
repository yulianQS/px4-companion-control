#include "core.hpp"
#include <algorithm>
#include "control/mission.hpp"
#include "vision_track.hpp"
#include <fstream>

// Controller lifecycle, shared checks/events, telemetry and scheduling.
Controller::Controller(PX4 &p, FlightLog &f) : px4(p), log(f) {
    std::ifstream runtime(project_root() / "config/runtime.json");
    if (runtime)
        vision_tracking_config = Json::parse(runtime).value("vision_tracking", Json::object());
    static_cast<void>(VisionTrack(vision_tracking_config));
    px4.set_sample_handler([this](const std::string &kind, const Json &s) {
        Setpoint out;
        {
            std::lock_guard<std::mutex> l(mutex);
            out = streaming ? target : Setpoint{};
        }
        log.submit(kind, s, out);
    });
    vision_reader = std::thread(&Controller::read_vision, this);
    ticker = std::thread(&Controller::tick, this);
    mission = std::make_unique<Mission>(*this);
}

Controller::~Controller() {
    mission.reset();
    stopping = true;
    if (ticker.joinable())
        ticker.join();
    if (vision_reader.joinable()) vision_reader.join();
    px4.set_sample_handler(nullptr);
}

void Controller::ready(const Json &s, bool need_position) {
    if (!s.value("connected", false) || !s.value("bidirectional", false))
        throw std::runtime_error("PX4 heartbeat/bidirectional link unavailable");
    int flags = s.value("estimator_flags", 0);
    if (!fresh(s, "position", 1) || !fresh(s, "estimator", 6) || (flags & 6) != 6 || (flags & 128))
        throw std::runtime_error("EKF velocity estimate unavailable");
    if (need_position) {
        if ((flags & 46) != 46 || !fresh(s, "attitude", .5) || !fresh(s, "odometry", 1))
            throw std::runtime_error("Fresh EKF position, heading and reset-counter required");
        Vec p = position(s), v = velocity(s);
        for (double n : p)
            if (!std::isfinite(n))
                throw std::runtime_error("Position not finite");
        for (double n : v)
            if (!std::isfinite(n))
                throw std::runtime_error("Velocity not finite");
        if (!std::isfinite(yaw(s)))
            throw std::runtime_error("Heading not finite");
    }
}

void Controller::capture(const Json &s) {
    holding = false;
    target = Setpoint{};
    try {
        ready(s);
        target.p = position(s);
        target.heading = yaw(s);
        holding = true;
        reset = s.at("odometry_reset_counter");
        sample_ms = s.value("position_sample_ms", 0u);
    } catch (...) {
    }
}

void Controller::event(const std::string &id, const std::string &stage,
                       const std::string &message) {
    if (id.empty())
        return;
    Json e = {{"id", id}, {"type", "event"}, {"stage", stage}, {"message", message}};
    events.push_back(e);
    if (events.size() > 64)
        events.erase(events.begin());
    last_action = e;
    last_action["ok"] = stage != "failed";
    last_action["confirmed"] = stage == "completed";
}

void Controller::finish(const std::string &state, const std::string &message, const Json &s,
                        bool capture_position) {
    task["state"] = state;
    task["message"] = message;
    task["target_reached"] = state == "succeeded";
    event(task_command_id, state == "succeeded" ? "completed" : "failed", message);
    if (capture_position)
        capture(s);
    else {
        holding = false;
        target = Setpoint{};
    }
    waypoints.clear();
}

void Controller::tick() {
    name_thread("fc-tick-20Hz");
    auto next = std::chrono::steady_clock::now();
    while (!stopping) {
        next += std::chrono::milliseconds(50);
        auto s = px4.status();
        double now = now_s();
        try {
            std::lock_guard<std::mutex> l(mutex);
            update(s, now);
            if (streaming) {
                px4.send(target);
                sent_at = now_s();
                if (!stream_started)
                    stream_started = sent_at;
            }
        } catch (const std::exception &e) {
            std::lock_guard<std::mutex> l(mutex);
            stream_error = e.what();
            streaming = sticks = velocity_active = false;
            finish("failed", e.what(), s, false);
        }
        std::this_thread::sleep_until(next);
        if (std::chrono::steady_clock::now() > next + std::chrono::milliseconds(50))
            next = std::chrono::steady_clock::now();
    }
}

Json Controller::status() {
    auto mission_status = mission ? mission->status(false) : Json::object();
    const auto tracked = vision_snapshot();
    Json tracking = Json::object();
    for (const auto *key : {"source", "camera_ready", "target_found", "target_observed", "target_retained",
                           "center", "width", "height", "tag_id", "pose_valid", "distance_m",
                           "tracking_epoch", "last_observed_monotonic", "confirmation_seconds", "tracking_reason"})
        if (tracked.contains(key)) tracking[key] = tracked[key];
    auto s = px4.status();
    std::lock_guard<std::mutex> l(mutex);
    double now = now_s();
    Json motion = task;
    motion["feedback_age_s"] = s.value("position_age_s", Json(nullptr));
    motion["setpoint"] = target.json();
    motion["holding_position"] = holding;
    const bool taking_off = task.value("state", std::string()) == "running" &&
        task.value("params", Json::object()).value("kind", std::string()) == "auto_takeoff";
    const bool visual = task.value("state", std::string()) == "running" && task.value("params", Json::object()).value("kind", std::string()) == "vision_follow";
    motion["controller"] = visual ? "vision_yaw_vertical" : taking_off ? "px4_relative_takeoff" : "px4_position";
    std::string source = visual ? "vision" : sticks                                            ? "joystick"
                         : task.value("state", std::string()) == "landing" ? "px4_land"
                         : taking_off                                      ? "px4_relative_takeoff"
                         : velocity_active                                 ? "velocity"
                         : task.value("state", std::string()) == "running" ? "task"
                         : holding                                         ? "hold"
                         : streaming                                       ? "zero"
                                                                           : "monitor";
    Json control = {{"takeoff_ground_z_m", std::isfinite(takeoff_ground_z) ? Json(takeoff_ground_z) : Json(nullptr)},
                    {"height_above_takeoff_ground_m", std::isfinite(takeoff_ground_z) && fresh(s, "position", 1) ? Json(takeoff_ground_z - position(s)[2]) : Json(nullptr)},
                    {"lease_active", now < lease_until},
                    {"lease_remaining_s", std::max(0., lease_until - now)},
                    {"zero_stream", streaming},
                    {"zero_error", stream_error.empty() ? Json(nullptr) : Json(stream_error)},
                    {"zero_age_s", sent_at > 0 ? Json(now - sent_at) : Json(nullptr)},
                    {"bench_remaining_s", nullptr},
                    {"last_action", last_action},
                    {"source", source},
                    {"setpoint", streaming ? target.json() : Json(nullptr)},
                    {"controller", taking_off ? "px4_relative_takeoff"
                                   : holding || task.value("state", std::string()) == "running"
                                       ? "px4_position"
                                       : "velocity"}};
    return {{"source", "px4"},
            {"actuation_enabled", true},
            {"protocol_version", 1},
            {"data",
             {{"px4", s},
              {"control", control},
              {"motion", motion},
              {"joystick", joystick_status(now)},
              {"vision", Json::object()},
              {"vision_tracking", std::move(tracking)},
              {"mission", std::move(mission_status)},
              {"trajectory", log.status()},
              {"command_events", events},
              {"backend", "cpp"}}}};
}

// Slow housekeeping runs on the command worker, never on the 20Hz control thread.

void Controller::maintain() {
    auto s = px4.status();
    std::string command;
    double now = now_s();
    {
        std::lock_guard<std::mutex> l(mutex);
        if (task.value("state", std::string()) == "landing" &&
            task.value("land_confirmed", false) && s["armed"] == true &&
            s.value("landed_state", 0) == 1 && fresh(s, "landed", 3)) {
            if (!settled_at)
                settled_at = now;
            if (now - settled_at >= 1)
                command = "DISARM";
        } else if (streaming && now >= lease_until && s["armed"] == false &&
                   s.value("connected", false))
            command = "RELEASE";
    }
    if (command.empty())
        return;
    try {
        auto result = perform(command, Json::object(), "housekeeping");
        std::lock_guard<std::mutex> l(mutex);
        last_action = result;
        last_action["action"] = command;
    } catch (const std::exception &e) {
        std::lock_guard<std::mutex> l(mutex);
        last_action = {{"ok", false}, {"message", e.what()}, {"action", command}};
    }
}
