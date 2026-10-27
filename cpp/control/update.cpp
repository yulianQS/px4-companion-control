#include "core.hpp"
#include <algorithm>

// 20Hz state transitions: LAND completion, joystick, velocity and trajectory tracking.
// Called with the controller mutex held. Blocking ACK waits belong in safety.cpp.
void Controller::update(const Json &s, double now) {
    if (emergency_stopped) {
        streaming = holding = sticks = velocity_active = false;
        return;
    }
    double dt = std::clamp(now - last_tick, .001, .1);
    last_tick = now;
    if (task.value("state", std::string()) == "landing") {
        task["elapsed_s"] = now - begun;
        if (task.value("land_confirmed", false) && s.value("connected", false) &&
            s["armed"] == false &&
            s.value("landed_state", 0) == 1 && fresh(s, "landed", 3)) {
            finish("succeeded", "飞控确认落地且已上锁", s, false);
            if (close_after_land) {
                owner.clear();
                lease_until = 0;
                close_after_land = false;
            }
        } else if (now - begun > 60)
            finish("timed_out", "LAND等待超时；PX4可能仍在降落", s, false);
        return;
    }
    if (task.value("state", std::string()) == "running" &&
        task.value("params", Json::object()).value("kind", std::string()) == "auto_takeoff") {
        update_takeoff(s, now);
        return;
    }
    if (!streaming)
        return;
    const bool visual = task.value("state",std::string())=="running" &&
        task.value("params",Json::object()).value("kind",std::string())=="vision_follow";
    if (!s.value("connected", false) || (!visual && sent_at > 0 && now - sent_at > .3)) {
        streaming = false;
        stream_error = "心跳失联或设定值线程超时；不自动恢复";
        sticks = velocity_active = false;
        finish("failed", "心跳失联或设定值线程超时；不自动恢复", s, false);
        return;
    }
    if (task.value("state",std::string())=="running" && task.value("phase",std::string())=="handover" &&
        task.value("params",Json::object()).value("kind",std::string())=="vision_follow") {
        try { check_vision_handover(s); }
        catch (const std::exception &e) { streaming=holding=false;finish("failed",e.what(),s,false); }
        return; // Hold only: no visual yaw/vertical velocity until OFFBOARD heartbeat is confirmed.
    }
    if (visual) {
        if (s["armed"]!=true || s.value("mode",std::string())!="OFFBOARD") {
            finish("failed","解锁或OFFBOARD状态变化",s,false);
            return;
        }
        update_vision(s,now,dt);
        return;
    }
    if (sticks) {
        if (now - stick_at >= .5 || now >= lease_until ||
            s.value("mode", std::string()) != "OFFBOARD" || !fresh(s, "attitude", .5)) {
            sticks = false;
            capture(s);
            return;
        }
        auto pair = [](double x, double y) {
            double r = std::hypot(x, y);
            if (r <= .08)
                return std::array<double, 2>{0, 0};
            double a = (std::min(1., r) - .08) / .92 / r;
            return std::array<double, 2>{x * a, y * a};
        };
        auto xy = pair(axes["roll"], axes["pitch"]);
        double up = pair(axes["throttle"], 0)[0], turn = pair(axes["yaw"], 0)[0], h = yaw(s);
        std::array<double, 4> desired = {.2 * (std::cos(h) * xy[1] - std::sin(h) * xy[0]),
                                         .2 * (std::sin(h) * xy[1] + std::cos(h) * xy[0]), -.2 * up,
                                         PI / 6 * turn};
        double dx = desired[0] - stick_command[0], dy = desired[1] - stick_command[1],
               d = std::hypot(dx, dy);
        if (d > .3 * dt) {
            desired[0] = stick_command[0] + dx * .3 * dt / d;
            desired[1] = stick_command[1] + dy * .3 * dt / d;
        }
        desired[2] =
            stick_command[2] + std::clamp(desired[2] - stick_command[2], -.3 * dt, .3 * dt);
        desired[3] =
            stick_command[3] + std::clamp(desired[3] - stick_command[3], -PI / 3 * dt, PI / 3 * dt);
        if (xy[0] == 0 && xy[1] == 0)
            desired[0] = desired[1] = 0;
        if (up == 0)
            desired[2] = 0;
        if (turn == 0)
            desired[3] = 0;
        stick_command = desired;
        target = Setpoint{};
        target.v = {desired[0], desired[1], desired[2]};
        target.rate = desired[3];
        bool valid = false;
        try {
            ready(s);
            valid = s["armed"] == true;
        } catch (...) {
        }
        if (valid) {
            int epoch = s.at("odometry_reset_counter");
            if (epoch != stick_epoch) {
                stick_hold = {NAN_VALUE, NAN_VALUE, NAN_VALUE};
                stick_yaw = NAN_VALUE;
            }
            stick_epoch = epoch;
            Vec actual = position(s);
            for (int i = 0; i < 3; i++) {
                bool moving = i < 2 ? (xy[0] != 0 || xy[1] != 0) : up != 0;
                if (moving)
                    stick_hold[i] = NAN_VALUE;
                else if (!std::isfinite(stick_hold[i]))
                    stick_hold[i] = actual[i];
            }
            if (turn != 0)
                stick_yaw = NAN_VALUE;
            else if (!std::isfinite(stick_yaw))
                stick_yaw = h;
            target.p = stick_hold;
            target.heading = stick_yaw;
        } else {
            stick_hold = {NAN_VALUE, NAN_VALUE, NAN_VALUE};
            stick_yaw = NAN_VALUE;
            stick_epoch = -1;
        }
        return;
    }
    bool running = task.value("state", std::string()) == "running";
    if (!running && !holding && !velocity_active) {
        if (task.value("state", std::string()) == "idle" && s["armed"] == true &&
            s.value("mode", std::string()) == "OFFBOARD" && s.value("landed_state", 0) == 2 &&
            fresh(s, "landed", 3) && now < lease_until)
            capture(s);
        return;
    }
    try {
        ready(s);
    } catch (const std::exception &e) {
        if (running)
            finish("failed", e.what(), s, false);
        else {
            holding = false;
            target = Setpoint{};
        }
        velocity_active = false;
        return;
    }
    uint32_t sample = s.value("position_sample_ms", 0u);
    // ODOMETRY counter changes do not prove LOCAL_POSITION_NED origin changes.
    // Keep the actual sample-clock restart check, without aborting on that counter.
    if (sample < sample_ms) {
        if (running)
            finish("failed", "飞控位置时间戳回退，旧坐标任务已停止", s, false);
        else {
            holding = false;
            target = Setpoint{};
        }
        velocity_active = false;
        sample_ms = sample;
        return;
    }
    reset = s.value("odometry_reset_counter", reset);
    sample_ms = sample;
    if (s["armed"] != true || s.value("mode", std::string()) != "OFFBOARD") {
        if (running)
            finish("failed", "解锁或OFFBOARD状态变化", s, false);
        else {
            holding = false;
            target = Setpoint{};
        }
        velocity_active = false;
        return;
    }
    // The lease authorizes new browser commands, not ongoing onboard execution.
    // Joystick input retains its separate freshness/lease check above.
    if (velocity_active) {
        if (now >= velocity_until) {
            velocity_active = false;
            finish("succeeded", "速度时段结束，当前位置保持已启动", s);
            return;
        }
        Vec v = requested_velocity;
        if (velocity_frame == "BODY_FRD") {
            double h = yaw(s), x = v[0], y = v[1];
            v[0] = std::cos(h) * x - std::sin(h) * y;
            v[1] = std::sin(h) * x + std::cos(h) * y;
        }
        target = Setpoint{};
        target.v = v;
        task["elapsed_s"] = now - begun;
        task["progress"] = (now - begun) / task["duration"].get<double>();
        return;
    }
    if (!running)
        return;
    if (task.value("params", Json::object()).value("kind", std::string()) == "vision_follow") {
        update_vision(s, now, dt);
        return;
    }
    task["elapsed_s"] = now - begun;
    task["actual_position"] = position(s);
    if (now - begun >= timeout) {
        finish("timed_out", "任务超时，当前位置保持", s);
        return;
    }
    const auto &p = task["params"];
    std::string kind = p.value("kind", std::string());
    if (s.value("landed_state", 0) != 2) {
        finish("failed", "飞控不再确认在空中", s);
        return;
    }
    if (update_move_leveling(s, now))
        return;
    double current_yaw = yaw(s), delta = wrap(current_yaw - last_yaw);
    travel_yaw += delta;
    last_yaw = current_yaw;
    bool circle = kind == "trajectory" && p.value("trajectory", std::string()) == "circle";
    bool in_circle = task.value("circle_started", false);
    bool circle_done = false;
    if (circle && in_circle) {
        double radius = p.value("radius", .5), omega = p.value("angular_speed_deg", 10.) * PI / 180;
        const double total=p.value("turns",1.)*2*PI;
        const auto ref=reference({0,0,0},{total,0,0},omega,p.value("acceleration",.3)/radius,
                                 now-task["circle_at"].get<double>());
        const double angle=ref.first[0];omega=ref.second[0];
        circle_done=angle>=total && omega==0;
        Vec origin = task["circle_origin"].get<Vec>();
        goal = {origin[0] + radius * (std::cos(angle) - 1), origin[1] + radius * std::sin(angle),
                origin[2]};
        target.p = goal;
        target.v = {-radius * std::sin(angle) * omega, radius * std::cos(angle) * omega, 0};
        target.heading = start_yaw;
        target.rate = 0;
        task["progress"] = std::min(.99, angle / total);
        task["goal_position"] = goal;
    } else {
        double elapsed = now - leg_begun;
        const double acceleration=p.value("acceleration",.3);
        auto xy = reference({start[0], start[1], 0}, {goal[0], goal[1], 0}, max_speed, acceleration, elapsed);
        auto z = reference({start[2], 0, 0}, {goal[2], 0, 0}, max_vertical, acceleration, elapsed);
        auto h = reference({start_yaw, 0, 0}, {goal_yaw, 0, 0}, yaw_speed,
                           p.value("angular_acceleration",60.)*PI/180, elapsed);
        target.p = {xy.first[0], xy.first[1], z.first[0]};
        target.v = {xy.second[0], xy.second[1], z.second[0]};
        target.heading = wrap(h.first[0]);
        target.rate = h.second[0];
    }
    Vec error = sub(goal, position(s));
    double angular_error = goal_yaw - (task["start_yaw_deg"].get<double>() * PI / 180 + travel_yaw);
    double measure = norm(sub(position(s), task["start_position"].get<Vec>())),
           distance_error = norm(error);
    if (kind == "rotate") {
        distance_error = std::abs(angular_error) * 180 / PI;
        measure = (goal_yaw >= task["start_yaw_deg"].get<double>() * PI / 180 ? 1 : -1) *
                  travel_yaw * 180 / PI;
    } else if (kind == "move") {
        Vec delta_goal = sub(goal, task["start_position"].get<Vec>());
        Vec delta_actual = sub(position(s), task["start_position"].get<Vec>());
        measure = (delta_goal[0] * delta_actual[0] + delta_goal[1] * delta_actual[1]) /
                  std::max(.001, std::hypot(delta_goal[0], delta_goal[1]));
        distance_error = std::hypot(error[0], error[1]);
    } else if (kind == "height") {
        double initial_z = task["start_position"][2];
        measure = (goal[2] >= initial_z ? 1 : -1) * (position(s)[2] - initial_z);
        distance_error = std::abs(error[2]);
    }
    task["measured"] = measure;
    task["error"] = distance_error;
    if (!circle)
        task["progress"] =
            waypoints.empty()
                ? std::clamp(
                      1 - distance_error /
                              std::max(.001, norm(sub(goal, task["start_position"].get<Vec>()))),
                      0., 1.)
                : double(waypoint) / waypoints.size();
    if (kind == "rotate" || kind == "move" || kind == "height")
        task["progress"] = std::clamp(measure / p.value("distance", .5), 0., 1.);
    double yaw_rate = s.value("angular_rate_rad_s", Json::object()).value("yawspeed", 1e6);
    bool reached = norm(velocity(s)) < .08 && fresh(s, "landed", 3);
    if (kind == "rotate")
        reached = reached && distance_error <= tolerance && std::hypot(error[0], error[1]) <= .15 &&
                  std::abs(error[2]) <= .1 && std::abs(yaw_rate) < 5 * PI / 180;
    else if (kind == "height")
        reached = reached && std::abs(error[2]) <= tolerance &&
                  std::hypot(error[0], error[1]) <= .15 &&
                  std::abs(wrap(goal_yaw - current_yaw)) < 5 * PI / 180;
    else if (kind == "move")
        reached = reached && std::hypot(error[0], error[1]) <= tolerance &&
                  std::abs(error[2]) <= .1 && std::abs(wrap(goal_yaw - current_yaw)) < 5 * PI / 180;
    else
        reached = reached && distance_error <= tolerance &&
                  std::abs(wrap(goal_yaw - current_yaw)) < 5 * PI / 180;
    if (circle && in_circle && !circle_done)
        reached = false;
    if (reached) {
        if (!settled_at)
            settled_at = now;
        uint64_t revision = s.value("position_revision", 0u);
        if (revision != settled_revision) {
            settled_revision = revision;
            settled_samples++;
        }
        if (now - settled_at >= .5 && settled_samples >= 3) {
            if (circle && !in_circle) {
                task["circle_started"] = true;
                task["circle_at"] = now;
                task["circle_origin"] = goal;
                settled_at = 0;
                settled_samples = 0;
                return;
            }
            if (waypoint + 1 < waypoints.size()) {
                start = goal;
                // The previous leg already settled at the requested heading.
                // Do not rewind a fixed-yaw trajectory at each waypoint.
                start_yaw = goal_yaw;
                goal = waypoints[++waypoint];
                task["goal_position"] = goal;
                leg_begun = now;
                settled_at = 0;
                settled_samples = 0;
                return;
            }
            if (kind == "trajectory" && p.value("trajectory", std::string()) == "hover" &&
                now - begun < p.value("duration", 5.))
                return;
            finish("succeeded", "真实反馈到达并稳定，继续保持目标位置", s, false);
            target.p = goal;
            target.v = {0, 0, 0};
            target.heading = wrap(goal_yaw);
            target.rate = 0;
            holding = true;
            task["progress"] = 1.;
        }
    } else {
        settled_at = 0;
        settled_samples = 0;
    }
}
