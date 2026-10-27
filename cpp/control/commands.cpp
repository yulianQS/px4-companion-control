#include "core.hpp"
#include <algorithm>
#include "mission.hpp"

// Control lease, command dispatch, motor test and motion command entry points.
Json Controller::perform(const std::string &command, const Json &p, const std::string &id) {
    if (command == "SET_MODE" || command == "ARM" || command == "DISARM" || command == "EMERGENCY_STOP" ||
        command == "LAND" || command == "DISABLE" || command == "RELEASE")
        return perform_safety(command, p, id);
    if (command == "VISION_START" || command == "VISION_STOP" || command == "VISION_UPDATE") return perform_vision(command, p, id);
    {
        std::lock_guard<std::mutex> l(mutex);
        if (command == "STICK_START" && (task.value("state", std::string()) == "running" || task.value("state", std::string()) == "landing"))
            throw std::runtime_error("Stop current task before selecting joystick control");
        if (sticks && (command == "MOVE" || command == "SET_POSITION" || command == "EXECUTE_TRAJECTORY" || command == "SET_VELOCITY" || command == "TAKEOFF"))
            throw std::runtime_error("Exit joystick control before starting another task");
        if (task.value("state", std::string()) == "running" &&
            task.value("params", Json::object()).value("kind", std::string()) == "vision_follow" &&
            command != "HOLD" && command != "STOP_TRAJECTORY")
            throw std::runtime_error("Stop vision control before selecting joystick or another task");
    }
    if (command == "STICK_START" || command == "STICK_STOP" || command == "STICK_INPUT")
        return perform_joystick(command, p, id);
    if (command == "TAKEOFF")
        return perform_takeoff(p, id);
    auto s = px4.status();
    {
        std::lock_guard<std::mutex> l(mutex);
        if (task.value("state", std::string()) == "running" &&
            task.value("params", Json::object()).value("kind", std::string()) == "auto_takeoff")
            throw std::runtime_error("Takeoff is active; use LAND to descend");
    }
    if (command == "MOTOR_TEST") {
        if (p.value("props_removed", false) != true || s["armed"] != false ||
            !s.value("connected", false) || !s.value("bidirectional", false))
            throw std::runtime_error(
                "Explicit props_removed and disarmed bidirectional link required");
        {
            std::lock_guard<std::mutex> l(mutex);
            if (streaming)
                throw std::runtime_error("Exit OFFBOARD before standalone motor test");
        }
        motor_cancel = false;
        auto stop_motors = [&] {
            for (int motor = 1; motor <= 4; motor++)
                try {
                    px4.command(
                        310, {std::numeric_limits<float>::quiet_NaN(), 0, 0, 0, float(motor), 0, 0},
                        .3);
                } catch (...) {
                };
        };
        try {
            double deadline = now_s() + 10;
            while (now_s() < deadline && !stopping && !motor_cancel) {
                auto current = px4.status();
                if (!current.value("connected", false) || current["armed"] != false)
                    throw std::runtime_error("Motor test heartbeat or arming state changed");
                for (int motor = 1; motor <= 4 && now_s() < deadline && !motor_cancel; motor++)
                    if (px4.command(310, {.05f, 1, 0, 0, float(motor), 0, 0}, .3) != 0)
                        throw std::runtime_error("Motor test rejected");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            stop_motors();
            return {{"ok", true},
                    {"confirmed", false},
                    {"stage", "completed"},
                    {"message", "10秒请求窗口结束，已尝试四路停止；必须观察停转"}};
        } catch (...) {
            stop_motors();
            throw;
        }
    }
    std::lock_guard<std::mutex> l(mutex);
    if (command == "MOVE" || command == "SET_POSITION" ||
        command == "EXECUTE_TRAJECTORY") {
        Json params = p;
        if (command == "SET_POSITION" || command == "MOVE") {
            if (!params.contains("kind"))
                params["kind"] = "position";
        }
        if (command == "EXECUTE_TRAJECTORY")
            params["kind"] = "trajectory";
        start_task(params, id, s);
        return {{"ok", true},         {"accepted", true},
                {"confirmed", false}, {"stage", "running"},
                {"task_id", id},      {"message", "C++执行中，完成由实际反馈确认"}};
    }
    if (command == "HOLD" || command == "STOP_TRAJECTORY") {
        ready(s);
        if (s.value("mode", std::string()) != "OFFBOARD" || !streaming)
            throw std::runtime_error("HOLD requires OFFBOARD stream");
        if (task.value("state", std::string()) == "landing")
            throw std::runtime_error("Cannot cancel PX4 LAND with HOLD");
        if (task.value("state", std::string()) == "running")
            finish("cancelled", "原轨迹已取消，捕获当前位置保持", s);
        capture(s);
        sticks = velocity_active = false;
        return {{"ok", true},
                {"confirmed", true},
                {"message", "已捕获当前位置；C++继续20Hz位置设定值，不等同停机"}};
    }
    if (command == "SET_VELOCITY") {
        ready(s);
        if (s["armed"] != true || s.value("mode", std::string()) != "OFFBOARD" || !streaming ||
            task.value("state", std::string()) == "landing")
            throw std::runtime_error("Velocity requires armed OFFBOARD, not LAND");
        std::string frame = p.value("frame", std::string());
        if (frame != "LOCAL_NED" && frame != "BODY_FRD")
            throw std::runtime_error("Explicit LOCAL_NED or BODY_FRD frame required");
        Vec v = {number(p, "vx", 0), number(p, "vy", 0), number(p, "vz", 0)};
        double duration = bounded(p, "duration", 1, .1, 10);
        if (task.value("state", std::string()) == "running")
            finish("cancelled", "Velocity supersedes trajectory", s);
        sticks = false;
        capture(s);
        requested_velocity = v;
        velocity_frame = frame;
        velocity_until = now_s() + duration;
        velocity_active = true;
        task_command_id = id;
        begun = now_s();
        task = {{"id", id},
                {"state", "running"},
                {"params", {{"kind", "velocity"}}},
                {"message", "C++速度控制，定时结束后保持位置"},
                {"duration", duration}};
        event(id, "running", "Velocity applied until duration, then position HOLD");
        return {{"ok", true},
                {"accepted", true},
                {"confirmed", false},
                {"stage", "running"},
                {"message", "速度设定已接受；单位m/s"}};
    }
    throw std::runtime_error("Unknown command");
}

Json Controller::execute(const std::string &command, const Json &p, const std::string &client,
                         const std::string &id, bool internal) {
    if (command == "EMERGENCY_STOP") {
        emergency_stopped = true;
        mission->stop("kill", "emergency_stop");
        auto result = perform_safety(command, p, id);
        std::lock_guard<std::mutex> l(mutex);
        last_action = result;
        last_action["id"] = id;
        last_action["action"] = command;
        event(id, result.value("confirmed", false) ? "completed" : "failed", result["message"]);
        return result;
    }
    if (internal && emergency_stopped && command != "GET_STATUS")
        throw std::runtime_error("任务已被一键停转终止");
    if (command == "MISSION_STATUS") return {{"ok",true},{"mission",mission->status()}};
    if (command == "MISSION_START") return mission->start(p,client);
    if (command == "MISSION_STOP") {
        const auto mode = p.value("mode", std::string("hold"));
        if (mode != "hold" && mode != "land")
            throw std::runtime_error("停止方式仅支持hold/land；停转使用EMERGENCY_STOP");
        return mission->stop(mode,p.value("source",std::string("user_request")));
    }
    if (!internal && mission->active() && command != "GET_STATUS" && command != "RENEW")
        throw std::runtime_error("C++比赛调度中，请使用比赛停止/降落接口");
    if (command == "GET_STATUS")
        return status();
    if (command == "DOCTOR") {
        int ack = px4.command(512, {148, 0, 0, 0, 0, 0, 0});
        auto result = status();
        result["ok"] = ack == 0;
        result["confirmed"] = ack == 0;
        result["ack"] = ack;
        result["message"] = "只读固件请求已返回";
        return result;
    }
    if (client.size() < 8 || client.size() > 100)
        throw std::runtime_error("Client ID must contain 8..100 characters");
    {
        std::lock_guard<std::mutex> l(mutex);
        double now = now_s();
        if (command == "CLAIM") {
            if (!owner.empty() && owner != client && now < lease_until)
                throw std::runtime_error("Another browser owns control");
            owner = client;
            lease_until = now + 20;
            return {{"ok", true},
                    {"confirmed", true},
                    {"lease_seconds", 20},
                    {"message", "C++控制会话已开启"}};
        }
        if (command != "VISION_START" && command != "VISION_STOP" && command != "VISION_UPDATE" && (owner != client || now >= lease_until))
            throw std::runtime_error("Enable control first; session expired");
        if (command == "STOP_MOTOR_TEST") {
            motor_cancel = true;
            return {{"ok", true},
                    {"confirmed", false},
                    {"stage", "accepted"},
                    {"message", "Motor-test cancellation requested; stop ACKs pending"}};
        }
        if (command == "RENEW") {
            lease_until = now + 20;
            return {{"ok", true}, {"confirmed", true}, {"lease_seconds", 20}};
        }
        if (command != "STICK_INPUT")
            event(id, "accepted", "Command accepted by C++ dispatcher");
    }
    try {
        auto result = perform(command, p, id);
        if (command != "STICK_INPUT") {
            std::lock_guard<std::mutex> l(mutex);
            if (!result.contains("stage"))
                result["stage"] = result.value("confirmed", false) ? "completed" : "failed";
            if (result["stage"] != "running")
                event(id, result["stage"], result.value("message", std::string()));
            last_action = result;
            last_action["id"] = id;
            last_action["action"] = command;
        }
        return result;
    } catch (const std::exception &e) {
        std::lock_guard<std::mutex> l(mutex);
        event(id, "failed", e.what());
        throw;
    }
}
