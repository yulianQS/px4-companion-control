#include "core.hpp"
#include <algorithm>

// Joystick sessions, neutral start, axis validation and stale-frame rejection.
Json Controller::perform_joystick(const std::string &command, const Json &p, const std::string &id) {
    auto s = px4.status();
    std::lock_guard<std::mutex> l(mutex);
    const bool taking_off = task.value("state", std::string()) == "running" &&
        task.value("params", Json::object()).value("kind", std::string()) == "auto_takeoff";
    if (command == "STICK_START") {
        if (taking_off)
            throw std::runtime_error("Takeoff is active; wait for height stabilization");
        if (s.value("mode", std::string()) != "OFFBOARD" || !streaming ||
            !fresh(s, "attitude", .5) || task.value("state", std::string()) == "landing")
            throw std::runtime_error("Joystick requires OFFBOARD and fresh yaw, not LAND");
        if (sticks)
            throw std::runtime_error("Joystick already active");
        if (task.value("state", std::string()) == "running")
            finish("cancelled", "Joystick supersedes old task", s);
        sticks = true;
        velocity_active = false;
        holding = false;
        stick_session = id;
        sequence = -1;
        stick_at = last_tick = now_s();
        stick_command = {};
        stick_hold = {NAN_VALUE, NAN_VALUE, NAN_VALUE};
        stick_yaw = NAN_VALUE;
        axes = {{"throttle", 0.}, {"yaw", 0.}, {"pitch", 0.}, {"roll", 0.}};
        return {{"ok", true},
                {"confirmed", true},
                {"session", id},
                {"message", "摇杆接管，等待中立输入"}};
    }
    if (command == "STICK_STOP") {
        if (p.value("session", std::string()) != stick_session)
            throw std::runtime_error("Stale joystick session");
        sticks = false;
        if (!taking_off && task.value("state", std::string()) != "landing" &&
            s.value("mode", std::string()) != "LAND")
            capture(s);
        return {{"ok", true}, {"confirmed", true}, {"message", "摇杆已退出；LAND期间不覆盖降落目标"}};
    }
    if (command == "STICK_INPUT") {
        if (!sticks || p.value("session", std::string()) != stick_session ||
            now_s() - stick_at >= .5)
            throw std::runtime_error("Joystick session expired");
        if (!p.contains("sequence") || !p["sequence"].is_number_integer())
            throw std::runtime_error("Integer sequence required");
        int64_t seq = p["sequence"];
        if (seq < 0 || seq <= sequence || seq > 9007199254740991LL)
            throw std::runtime_error("Duplicate or stale joystick input");
        auto a = p.at("axes");
        if (!a.is_object() || a.size() != 4)
            throw std::runtime_error("Exactly four axes required");
        for (const char *k : {"throttle", "yaw", "pitch", "roll"}) {
            double v = bounded(a, k, 2, -1, 1);
            if (sequence == -1 && std::abs(v) > .08)
                throw std::runtime_error("First joystick frame must be neutral");
        }
        axes = a;
        sequence = seq;
        stick_at = now_s();
        return {{"ok", true},
                {"confirmed", true},
                {"joystick", joystick_status(now_s())},
                {"message", "Input received"}};
    }
    throw std::runtime_error("Unknown joystick command");
}

Json Controller::joystick_status(double now) {
    return {
        {"active", sticks},
        {"state", sticks ? "active" : "stopped"},
        {"session", stick_session},
        {"sequence", sequence},
        {"axes", sticks ? axes : Json{{"throttle", 0.}, {"yaw", 0.}, {"pitch", 0.}, {"roll", 0.}}},
        {"input_age_s", stick_at > 0 ? Json(now - stick_at) : Json(nullptr)},
        {"command_ned", sticks ? stick_command : std::array<double, 4>{}},
        {"limits", {{"horizontal", .2}, {"vertical", .2}, {"yaw_deg", 30}, {"timeout_s", .5}}},
        {"setpoint", target.json()}};
}
