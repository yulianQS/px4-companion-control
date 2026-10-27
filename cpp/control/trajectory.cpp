#include "core.hpp"
#include <algorithm>

// Trajectory construction and analytic reference; no I/O or command waits.
std::pair<Vec, Vec> reference(Vec start, Vec goal, double speed, double acc, double elapsed) {
    Vec delta = sub(goal, start), v{};
    double distance = norm(delta);
    if (distance < 1e-9)
        return {goal, v};
    if (acc == 0) {
        const double t=std::max(0.,elapsed), travel=std::min(distance,speed*t);
        for(int i=0;i<3;++i) {
            start[i]+=delta[i]/distance*travel;
            v[i]=travel<distance ? delta[i]/distance*speed : 0;
        }
        return {start,v};
    }
    double ramp = std::min(speed / acc, std::sqrt(distance / acc)), peak = acc * ramp;
    double cruise = std::max(0., distance / peak - ramp), duration = 2 * ramp + cruise;
    double t = std::max(0., elapsed), travel = distance, value = 0;
    if (t < ramp) {
        travel = .5 * acc * t * t;
        value = acc * t;
    } else if (t < ramp + cruise) {
        travel = .5 * acc * ramp * ramp + peak * (t - ramp);
        value = peak;
    } else if (t < duration) {
        double remaining = duration - t;
        travel = distance - .5 * acc * remaining * remaining;
        value = acc * remaining;
    }
    for (int i = 0; i < 3; i++) {
        start[i] += delta[i] / distance * travel;
        v[i] = delta[i] / distance * value;
    }
    return {start, v};
}

void Controller::start_task(const Json &p, const std::string &id, const Json &s) {
    if(number(p,"acceleration",.3)<0)throw std::runtime_error("acceleration must be nonnegative");
    if(number(p,"angular_acceleration",60)<0)throw std::runtime_error("angular_acceleration must be nonnegative");
    if (p.value("kind", std::string()) == "takeoff")
        throw std::runtime_error("Use TAKEOFF for a ground-relative climb");
    ready(s);
    if (s.value("armed", false) != true || s.value("mode", std::string()) != "OFFBOARD" ||
        !streaming)
        throw std::runtime_error("Task requires armed OFFBOARD with setpoint stream");
    if (sticks)
        throw std::runtime_error("Exit joystick before starting task");
    if (task.value("state", std::string()) == "landing")
        throw std::runtime_error("LAND is active");
    if (!fresh(s, "landed", 3))
        throw std::runtime_error("Landed state stale");
    std::string kind = p.value("kind", std::string("position"));
    const bool level = p.value("level_before_move", false) && (kind == "move" || kind == "height");
    if (level) {
        bounded(p, "level_angle_deg", 5, 1, 30);
        bounded(p, "level_speed_m_s", .2, .01, 1);
        bounded(p, "level_settle_s", .5, .1, 5);
        bounded(p, "level_timeout_s", 10, 1, 60);
    }
    if (s.value("landed_state", 0) != 2)
        throw std::runtime_error("Task landed state incompatible");
    Vec next_start = position(s), next_goal = next_start;
    double next_yaw = yaw(s), next_goal_yaw = next_yaw;
    // Optional mission heading; absent input preserves the existing interface.
    if (kind == "trajectory" && p.value("trajectory", std::string()) == "waypoints" && p.contains("fixed_yaw_deg"))
        next_goal_yaw = next_yaw + wrap(bounded(p, "fixed_yaw_deg", 0, -180, 180) * PI / 180 - next_yaw);
    double speed =
        positive(p, "speed", kind == "rotate" ? 30 : .2);
    double tol = kind == "rotate" ? bounded(p, "tolerance", 3, 1, 15)
                                  : bounded(p, "tolerance", .05, .01, .5);
    double limit = bounded(p, "timeout", 30, 2, 3600);
    std::vector<Vec> next_points;
    if (kind == "move" || kind == "height" || kind == "rotate") {
        const std::string direction = p.value("direction", std::string());
        double distance =
            bounded(p, "distance", kind == "rotate" ? 90 : .5, kind == "rotate" ? 1 : .1,
                    kind == "rotate" ? 360
                    : kind == "move" ? 5
                                     : 2);
        if (kind == "move") {
            if (direction != "forward" && direction != "back" && direction != "left" &&
                direction != "right")
                throw std::runtime_error("Invalid move direction");
            double forward = direction == "forward" ? distance
                             : direction == "back"  ? -distance
                                                    : 0,
                   right = direction == "right"  ? distance
                           : direction == "left" ? -distance
                                                 : 0;
            next_goal[0] += std::cos(next_yaw) * forward - std::sin(next_yaw) * right;
            next_goal[1] += std::sin(next_yaw) * forward + std::cos(next_yaw) * right;
        } else if (kind == "rotate") {
            if (direction != "clockwise" && direction != "counterclockwise")
                throw std::runtime_error("Invalid yaw direction");
            speed = positive(p, "speed", 30);
            tol = bounded(p, "tolerance", 3, 1, 15);
            next_goal_yaw += distance * PI / 180 * (direction == "clockwise" ? 1 : -1);
        } else {
            if (direction != "up" && direction != "down")
                throw std::runtime_error("Invalid height direction");
            speed = positive(p, "speed", .2);
            next_goal[2] += (direction == "up" ? -distance : distance);
        }
        if (tol >= distance)
            throw std::runtime_error("Tolerance must be smaller than distance");
    } else if (kind == "position") {
        if (p.value("frame", std::string()) != "LOCAL_NED" ||
            p.value("unit", std::string()) != "cm")
            throw std::runtime_error("Position requires LOCAL_NED and cm");
        next_goal = {bounded(p, "x", 0, -10000, 10000) / 100,
                     bounded(p, "y", 0, -10000, 10000) / 100,
                     bounded(p, "z", 0, -10000, 10000) / 100};
    } else if (kind == "trajectory") {
        std::string type = p.value("trajectory", std::string());
        double height = bounded(p, "height", .5, .1, 4.5);
        Vec base = next_start;
        base[2] = -height;
        if (type == "hover") {
            bounded(p, "duration", 5, 1, 600);
            next_goal = base;
        } else if (type == "line") {
            next_points.push_back(base);
            base[0] += bounded(p, "length", 1, .1, 5);
            next_points.push_back(base);
            next_goal = next_points[0];
        } else if (type == "rectangle") {
            double width = bounded(p, "width", 1, .1, 5), length = bounded(p, "length", 1, .1, 5);
            next_points = {base, add(base, {length, 0, 0}), add(base, {length, width, 0}),
                           add(base, {0, width, 0}), base};
            next_goal = base;
        } else if (type == "circle") {
            bounded(p, "radius", .5, .1, 5);
            positive(p, "angular_speed_deg", 10);
            bounded(p, "turns", 1, 1, 10);
            next_goal = base;
        } else if (type == "waypoints") {
            if (p.value("frame", std::string()) != "LOCAL_NED" ||
                p.value("unit", std::string()) != "cm")
                throw std::runtime_error("Waypoints require LOCAL_NED / cm");
            auto w = p.at("waypoints");
            if (!w.is_array() || w.empty() || w.size() > 100)
                throw std::runtime_error("1..100 waypoints required");
            for (auto &row : w) {
                if (!row.is_array() || row.size() != 3)
                    throw std::runtime_error("Each waypoint needs X,Y,Z");
                Vec v;
                for (int i = 0; i < 3; i++) {
                    if (!row[i].is_number() || !std::isfinite(row[i].get<double>()) ||
                        std::abs(row[i].get<double>()) > 10000)
                        throw std::runtime_error("Invalid waypoint");
                    v[i] = row[i].get<double>() / 100;
                }
                next_points.push_back(v);
            }
            next_goal = next_points[0];
        } else
            throw std::runtime_error("Unknown trajectory");
    } else
        throw std::runtime_error("Unknown task kind");
    if (task.value("state", std::string()) == "running")
        event(task_command_id, "failed", "Superseded by new target");
    start = next_start;
    goal = next_goal;
    start_yaw = next_yaw;
    goal_yaw = next_goal_yaw;
    last_yaw = start_yaw;
    travel_yaw = 0;
    waypoints = next_points;
    waypoint = 0;
    begun = leg_begun = last_tick = now_s();
    settled_at = 0;
    settled_samples = 0;
    settled_revision = 0;
    leveling_attitude_revision = 0;
    max_speed = max_vertical = kind == "rotate" ? .2 : speed;
    yaw_speed = kind == "rotate" ? speed * PI / 180 : PI / 6;
    tolerance = tol;
    timeout = limit;
    reset = s.at("odometry_reset_counter");
    sample_ms = s.value("position_sample_ms", 0u);
    target = Setpoint{};
    target.p = start;
    target.heading = start_yaw;
    holding = true;
    velocity_active = false;
    task_command_id = id;
    task = {{"id", id},
            {"state", "running"},
            {"params", p},
            {"message", "C++轨迹已启动，等待真实反馈"},
            {"start_position", start},
            {"goal_position", goal},
            {"start_yaw_deg", start_yaw * 180 / PI},
            {"progress", 0.},
            {"target_reached", false},
            {"circle_started", false}};
    if (level) {
        // Brake through PX4 velocity control without pulling back to the old XY
        // position. Keep altitude/yaw; do not bypass PX4 with attitude/thrust commands.
        target.p = {NAN_VALUE, NAN_VALUE, start[2]};
        target.v = {0, 0, 0};
        task["phase"] = "leveling";
        task["message"] = "相对位移：先制动并等待横滚、俯仰回正，尚未开始位移";
    }
    event(id, "running", "PX4 native position loop tracking C++ reference");
}

bool Controller::update_move_leveling(const Json &s, double now) {
    if (task.value("phase", std::string()) != "leveling")
        return false;
    const auto &p = task["params"];
    if (now - begun >= p.value("level_timeout_s", 10.)) {
        finish("failed", "相对位移回正超时，未执行位移；当前位置保持", s);
        return true;
    }
    const auto &attitude = s.at("attitude_deg");
    const double roll = attitude.at(0).is_number() ? attitude.at(0).get<double>() : NAN_VALUE,
                 pitch = attitude.at(1).is_number() ? attitude.at(1).get<double>() : NAN_VALUE;
    const double speed = norm(velocity(s));
    const double angle_limit = p.value("level_angle_deg", 5.);
    const double speed_limit = p.value("level_speed_m_s", .2);
    const bool level = fresh(s, "attitude", .25) && fresh(s, "position", .25) &&
                       std::isfinite(roll) && std::isfinite(pitch) &&
                       std::abs(roll) <= angle_limit && std::abs(pitch) <= angle_limit &&
                       speed <= speed_limit;
    const auto revision = s.value("position_revision", uint64_t(0));
    // PX4::status exposes receive counts, not the private receive timestamps.
    const auto attitude_revision = s.value("counts", Json::object()).value("ATTITUDE", uint64_t(0));
    const bool new_feedback = revision != settled_revision &&
                              attitude_revision != leveling_attitude_revision;
    if (!level) {
        settled_at = 0;
        settled_samples = 0;
    } else if (new_feedback) {
        if (!settled_at)
            settled_at = now;
        ++settled_samples;
    }
    if (new_feedback) {
        settled_revision = revision;
        leveling_attitude_revision = attitude_revision;
    }
    task["leveling"] = {{"roll_deg", roll}, {"pitch_deg", pitch}, {"speed_m_s", speed},
                        {"angle_limit_deg", angle_limit}, {"speed_limit_m_s", speed_limit},
                        {"stable_s", settled_at ? now - settled_at : 0.},
                        {"elapsed_s", now - begun}};
    if (!level || !new_feedback || !settled_at || settled_samples < 3 ||
        now - settled_at < p.value("level_settle_s", .5))
        return true;

    // Braking drift is not part of the requested relative displacement. Preserve
    // the entry heading/direction, but begin the distance at the settled position.
    const Vec displacement = sub(goal, start);
    start = position(s);
    goal = add(start, displacement);
    target.p = start;
    target.v = {0, 0, 0};
    leg_begun = now;
    last_yaw = yaw(s);
    travel_yaw = wrap(last_yaw - start_yaw);
    settled_at = 0;
    settled_samples = 0;
    settled_revision = 0;
    task["start_position"] = start;
    task["goal_position"] = goal;
    task["phase"] = "moving";
    task["message"] = "回正并制动完成，开始执行相对位移";
    event(task_command_id, "running", "回正并制动完成，开始执行相对位移");
    return true;
}
