#include "core.hpp"
#include <algorithm>

// Mode selection, arming and the single LAND/disable policy.
Json Controller::confirmed(int cmd, const std::array<float, 7> &params,
                           const std::function<bool(const Json &)> &predicate) {
    uint64_t revision = px4.status().value("heartbeat_revision", 0u);
    int ack = px4.command(cmd, params);
    bool confirmed = false;
    if (ack == 0)
        for (int i = 0; i < 60 && !stopping; i++) {
            auto s = px4.status();
            if (s.value("connected", false) && s.value("heartbeat_revision", 0u) > revision &&
                predicate(s)) {
                confirmed = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    auto s = px4.status();
    return {{"ok", confirmed},
            {"confirmed", confirmed},
            {"ack", ack},
            {"armed", s.value("armed", Json(nullptr))},
            {"mode", s.value("mode", Json(nullptr))},
            {"last_text", s.value("last_text", Json(nullptr))},
            {"message", confirmed ? "新心跳已确认目标状态" : "飞控拒绝或未确认状态变化"}};
}

Json Controller::perform_safety(const std::string &command, const Json &p, const std::string &id) {
    auto s = px4.status();
    if (command == "EMERGENCY_STOP") {
        {
            std::lock_guard<std::mutex> l(mutex);
            streaming = holding = sticks = velocity_active = close_after_land = false;
            motor_cancel = true;
            waypoints.clear();
            target = Setpoint{};
            task = {{"id", id}, {"state", "cancelled"}, {"message", "一键停转：任务与控制输出已终止"}};
        }
        auto result = confirmed(400, {0, 21196, 0, 0, 0, 0, 0},
                                [](auto &state) { return state["armed"] == false; });
        result["message"] = result.value("confirmed", false)
                                ? "飞控已确认强制上锁，电机停转"
                                : "停转命令未获飞控确认，请检查飞控状态";
        return result;
    }
    if (command == "SET_MODE") {
        if (!s.value("connected", false) || !s.value("bidirectional", false))
            throw std::runtime_error("PX4 link unavailable");
        if (!s["armed"].is_boolean() || s["armed"].get<bool>())
            throw std::runtime_error("Mode selection requires confirmed disarmed state");
        std::string mode = p.value("mode", std::string());
        {
            std::lock_guard<std::mutex> l(mutex);
            if (task.value("state", std::string()) == "landing")
                throw std::runtime_error("LAND is active; wait for touchdown and disarm");
        }
        const std::map<std::string, std::pair<int, int>> modes = {{"OFFBOARD", {6, 0}},
                                                               {"LOITER", {4, 3}},
                                                                  {"ALTCTL", {2, 0}},
                                                                  {"POSCTL", {3, 0}},
                                                                  {"STABILIZED", {7, 0}}};
        if (!modes.count(mode))
            throw std::runtime_error("Unsupported mode");
        if (mode == "OFFBOARD") {
            ready(s, false);
            emergency_stopped = false; // Only explicit manual mode setup resumes output.
            {
                std::lock_guard<std::mutex> l(mutex);
                target = Setpoint{};
                holding = false;
                streaming = true;
                stream_error.clear();
                stream_started = sent_at = 0;
            }
            bool complete = false;
            for (int i = 0; i < 60 && !stopping; i++) {
                {
                    std::lock_guard<std::mutex> l(mutex);
                    complete = streaming && stream_started > 0 && now_s() - stream_started >= 1.5 &&
                               now_s() - sent_at < .3;
                }
                if (complete)
                    break;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            if (!complete)
                throw std::runtime_error("Offboard prestream failed");
            s = px4.status();
            ready(s, false);
            if (s.value("armed", true))
                throw std::runtime_error("Arming state changed during prestream");
        }
        auto result = confirmed(
            176, {1, float(modes.at(mode).first), float(modes.at(mode).second), 0, 0, 0, 0},
            [mode](auto &s) { return s.value("mode", std::string()) == mode; });
        if (result.value("confirmed", false) && mode != "OFFBOARD") {
            std::lock_guard<std::mutex> l(mutex);
            streaming = holding = sticks = velocity_active = false;
            target = Setpoint{};
        }
        return result;
    }
    if (command == "ARM") {
        ready(s, false);
        if (s.value("armed", true) || s.value("mode", std::string()) != "OFFBOARD" ||
            s.value("landed_state", 0) != 1 || !fresh(s, "landed", 3))
            throw std::runtime_error("ARM requires disarmed OFFBOARD on ground");
        if (!fresh(s, "battery", 3) || !s.value("battery_v", Json()).is_number() ||
            s["battery_v"].get<double>() < 14.4)
            throw std::runtime_error("Existing 4S 14.4V battery check not satisfied");
        {
            std::lock_guard<std::mutex> l(mutex);
            if (!streaming || now_s() - sent_at > .3)
                throw std::runtime_error("Offboard stream unavailable");
            sticks = false;
        }
        return confirmed(400, {1, 0, 0, 0, 0, 0, 0},
                         [](auto &s) { return s.value("armed", false); });
    }
    if (command == "DISARM") {
        if (!s.value("connected", false))
            throw std::runtime_error("PX4 disconnected");
        if (s["armed"] == false)
            return {{"ok", true}, {"confirmed", true}, {"message", "Already disarmed"}};
        if (s.value("landed_state", 0) != 1 || !fresh(s, "landed", 3))
            throw std::runtime_error("Use LAND while airborne; no forced disarm");
        {
            std::lock_guard<std::mutex> l(mutex);
            sticks = velocity_active = false;
            capture(s);
        }
        return confirmed(400, {0, 0, 0, 0, 0, 0, 0}, [](auto &s) { return s["armed"] == false; });
    }
    // LAND, DISABLE and the legacy RELEASE command share one landing policy.
    // Never disarm first or substitute LOITER/POSCTL when disabling control.
    if (command == "LAND" || command == "DISABLE" || command == "RELEASE") {
        const bool disabling = command != "LAND";
        if (!s.value("connected", false))
            throw std::runtime_error("PX4 link unavailable; LAND cannot be confirmed");
        if (!s["armed"].is_boolean())
            throw std::runtime_error("Arming state unknown; LAND cannot be confirmed");
        {
            std::lock_guard<std::mutex> l(mutex);
            close_after_land = close_after_land || disabling;
            sticks = velocity_active = false;
            if (task.value("state", std::string()) == "running")
                event(task_command_id, "failed", "Superseded by LAND");
            // Freeze at the current position while waiting for LAND confirmation.
            // Once confirmed, PX4 owns descent and no Offboard setpoints are sent.
            capture(s);
            waypoints.clear();
            task_command_id = id;
            begun = now_s();
            settled_at = 0;
            task = {{"id", id}, {"state", "landing"}, {"land_confirmed", false},
                    {"message", "等待PX4 LAND模式与真实落地反馈"}};
        }
        auto result = confirmed(176, {1, 4, 6, 0, 0, 0, 0},
                                [](auto &state) { return state.value("mode", std::string()) == "LAND"; });
        s = px4.status();
        {
            std::lock_guard<std::mutex> l(mutex);
            if (result.value("confirmed", false)) {
                streaming = holding = false;
                target = Setpoint{};
                task["land_confirmed"] = true;
                result["accepted"] = true;
                if (s.value("connected", false) && s["armed"] == false &&
                    s.value("landed_state", 0) == 1 && fresh(s, "landed", 3)) {
                    finish("succeeded", "LAND已确认，飞控已落地且上锁", s, false);
                    if (close_after_land) {
                        owner.clear();
                        lease_until = 0;
                        close_after_land = false;
                        result["control_disabled"] = true;
                    }
                    result["stage"] = "completed";
                    result["message"] = "LAND已确认，飞控已落地且上锁";
                } else {
                    event(id, "running", "LAND mode confirmed; touchdown still pending");
                    result["confirmed"] = false;
                    result["stage"] = "running";
                    result["control_disabled"] = false;
                    result["message"] = "PX4 LAND已确认，等待落地/上锁；不会切回其他模式";
                }
            } else {
                close_after_land = false;
                result["control_disabled"] = false;
                finish("failed", "LAND模式未确认；未关闭控制，不切换其他模式", s, false);
            }
        }
        return result;
    }
    throw std::runtime_error("Unknown safety command");
}
