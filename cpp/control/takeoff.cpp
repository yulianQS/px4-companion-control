#include "core.hpp"
#include <algorithm>

// Indoor takeoff uses the existing Offboard position loop and trajectory
// reference. Ground Z is captured once; Navigator/Home altitude is not used.
Json Controller::perform_takeoff(const Json &p, const std::string &id) {
    const auto s = px4.status();
    ready(s);
    if (s["armed"] != true || s.value("landed_state", 0) != 1 || !fresh(s, "landed", 3))
        throw std::runtime_error("Takeoff requires manual arming and confirmed ground state");
    if (!p.contains("speed") && !fresh(s, "takeoff_speed", 30))
        throw std::runtime_error("Waiting for PX4 MPC_TKO_SPEED readback");
    const double height =
        bounded(p, "height", number(p, "distance", number(s, "takeoff_alt_m", .5)), .1, 5);
    const double speed = positive(p, "speed", number(s, "takeoff_speed_m_s", .2));
    const double tol = bounded(p, "tolerance", .05, .01, .5);
    const double acceleration=number(p,"acceleration",.3);
    if(acceleration<0)throw std::runtime_error("acceleration must be nonnegative");
    const double limit = bounded(p, "timeout", 30, 2, 3600);
    if (tol >= height)
        throw std::runtime_error("Takeoff tolerance must be smaller than height");
    std::lock_guard<std::mutex> l(mutex);
    if (now_s() >= lease_until || sticks || !streaming ||
        s.value("mode", std::string()) != "OFFBOARD" ||
        task.value("state", std::string()) == "running" ||
        task.value("state", std::string()) == "landing")
        throw std::runtime_error("Takeoff requires idle armed OFFBOARD with control ownership");
    capture(s);
    velocity_active = false;
    start = goal = position(s);
    takeoff_ground_z = start[2];
    goal[2] = takeoff_ground_z - height;
    max_vertical = speed;
    tolerance = tol;
    timeout = limit;
    begun = now_s();
    settled_at = 0;
    settled_samples = 0;
    settled_revision = 0;
    task_command_id = id;
    task = {{"id", id},
            {"state", "running"},
            {"phase", "climbing"},
            {"params", {{"kind", "auto_takeoff"}, {"height", height}, {"speed", speed}, {"acceleration",acceleration}}},
            {"start_position", start},
            {"goal_position", goal},
            {"height_reference", "takeoff_ground_local_ned"},
            {"ground_z_m", takeoff_ground_z},
            {"auto_confirmed", true},
            {"target_reached", false},
            {"progress", 0.},
            {"message", "C++相对地面起飞，PX4位置控制执行；到高后保持"}};
    event(id, "running", task["message"]);
    return {{"ok", true},         {"accepted", true}, {"confirmed", false},
            {"stage", "running"}, {"task_id", id},    {"message", task["message"]}};
}

// Called by the existing 20 Hz controller with its mutex held. No new PID.
void Controller::update_takeoff(const Json &s, double now) {
    task["elapsed_s"] = now - begun;
    try {
        ready(s);
        // An accepted onboard task continues when the browser's command lease expires.
        if (!streaming || s["armed"] != true || !fresh(s, "landed", 3) ||
            s.value("mode", std::string()) != "OFFBOARD")
            throw std::runtime_error("起飞设定值发送、解锁或OFFBOARD状态变化");
        const auto sample = s.value("position_sample_ms", 0u);
        if (sample < sample_ms)
            throw std::runtime_error("飞控位置时间戳回退，起飞坐标任务已停止");
        sample_ms = sample;
        if (now - begun >= timeout)
            throw std::runtime_error("相对地面起飞超时，保持当前位置");
    } catch (const std::exception &e) {
        if (s.value("mode", std::string()) != "OFFBOARD")
            streaming = false;
        finish("failed", e.what(), s);
        return;
    }
    const auto ref = reference(start, goal, max_vertical, task["params"].value("acceleration",.3), now - begun);
    target.p = ref.first;
    target.v = ref.second;
    const double measured = takeoff_ground_z - position(s)[2];
    const double error = task["params"]["height"].get<double>() - measured;
    const double vz = std::abs(velocity(s)[2]);
    task["actual_position"] = position(s);
    task["measured"] = measured;
    task["error"] = error;
    task["progress"] = std::clamp(measured / task["params"]["height"].get<double>(), 0., 1.);
    task["completion"] = {{"height_error_m", std::abs(error)},
                          {"height_tolerance_m", tolerance},
                          {"vertical_speed_m_s", vz},
                          {"ground_z_m", takeoff_ground_z},
                          {"target_z_m", goal[2]},
                          {"landed_state", s.value("landed_state", 0)}};
    const int landed = s.value("landed_state", 0);
    if ((landed != 2 && landed != 3) || std::abs(error) > tolerance || vz >= .08 ||
        norm(ref.second) > 1e-6) {
        settled_at = 0;
        settled_samples = 0;
        return;
    }
    if (!settled_at)
        settled_at = now;
    const uint64_t revision = s.value("position_revision", 0u);
    if (revision != settled_revision) {
        settled_revision = revision;
        ++settled_samples;
    }
    if (now - settled_at >= .5 && settled_samples >= 3) {
        const auto hold = target;
        finish("succeeded", "相对起飞地面到高，OFFBOARD位置保持", s, false);
        target = hold;
        target.p = goal;
        target.v = {0, 0, 0};
        holding = true;
        task["phase"] = "completed";
        task["progress"] = 1.;
    }
}
