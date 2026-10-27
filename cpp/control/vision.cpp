#include "core.hpp"
#include "vision_track.hpp"
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace {
Vec slew_velocity(Vec current, const Vec &desired, double step) {
    if(step==0)return desired;
    const auto delta = sub(desired, current);
    const double size = norm(delta);
    if (size <= step || size < 1e-9)
        return desired;
    for (int i = 0; i < 3; ++i)
        current[i] += delta[i] * step / size;
    return current;
}
bool vision_airborne(const Json &s) {
    // Flight-state gate only: PX4 airborne or taking off.
    if (s.value("landed_state", 0) == 2)
        return true;
    return s.value("landed_state", 0) == 3;
}
Json vision_parameters(const Json &input, Json settings = Json::object()) {
    const Json defaults = {{"yaw_gain", .3},
                           {"yaw_ki", 0.},
                           {"yaw_kd", 0.},
                           {"yaw_speed_rad_s", 0.},
                           {"vertical_gain", .15},
                           {"vertical_ki", 0.},
                           {"vertical_kd", 0.},
                           {"vertical_speed_m_s", 0.},
                           {"distance_gain", .3},
                           {"distance_ki", 0.},
                           {"distance_kd", 0.},
                           {"forward_speed_m_s", 0.},
                           {"lateral_gain", .15}, {"lateral_ki", 0.}, {"lateral_kd", 0.},
                           {"lateral_speed_m_s", 0.},
                           {"target_distance_m", 1.},
                           {"tag_size_m", .12},
                           {"lost_timeout_s", 3.},
                           {"max_distance_m", 0.}};
    settings["kind"] = "vision_follow";
    for (auto it = defaults.begin(); it != defaults.end(); ++it) {
        const double value =
            number(input, it.key().c_str(), settings.value(it.key(), it.value().get<double>()));
        if ((it.key() == "target_distance_m" || it.key() == "tag_size_m") && value <= 0)
            throw std::runtime_error("目标距离和码纸实际边长必须为正数");
        if (it.key().find("speed_") != std::string::npos && value < 0)
            throw std::runtime_error("速度幅值必须大于等于0，0表示按PID原值输出");
        if ((it.key() == "lost_timeout_s" || it.key() == "max_distance_m") && value < 0)
            throw std::runtime_error("丢失续行时间和距离范围不能为负数");
        settings[it.key()] = value;
    }
    for (const auto *name : {"vertical_enabled", "distance_enabled", "lateral_enabled"})
        settings[name] = input.value(name, settings.value(name, false));
    if (input.contains("distance_constant_speed") || settings.contains("distance_constant_speed")) {
        settings["distance_constant_speed"] = input.value("distance_constant_speed", settings.value("distance_constant_speed", false));
        settings["distance_tolerance_m"] = number(input, "distance_tolerance_m", settings.value("distance_tolerance_m", .1));
        if (settings["distance_tolerance_m"].get<double>() <= 0)
            throw std::runtime_error("距离容差必须为正数");
    }
    if (input.contains("search_approach") || settings.contains("search_approach")) {
        settings["search_approach"] =
            input.value("search_approach", settings.value("search_approach", false));
        const Json search = {{"search_bearing_deg", 0.},      {"search_observed_at", 0.},
                             {"search_speed_m_s", .1},        {"search_acceleration", .2},
                             {"search_turn_rate_deg_s", 15.}, {"search_scan_angle_deg", 10.}};
        for (auto it = search.begin(); it != search.end(); ++it)
            settings[it.key()] =
                number(input, it.key().c_str(), settings.value(it.key(), it.value().get<double>()));
        if (settings["search_speed_m_s"].get<double>() < 0 ||
            settings["search_acceleration"].get<double>() < 0 ||
            settings["search_turn_rate_deg_s"].get<double>() <= 0 ||
            settings["search_scan_angle_deg"].get<double>() < 0)
            throw std::runtime_error("搜索接近速度/角度/加速度不能为负，转向速度必须为正");
    }
    return settings;
}
std::string vision_config_path() {
    const char *configured = std::getenv("DRONE_VISION_CONTROL_CONFIG");
    return configured ? configured : (project_root() / "config/vision_control.json").string();
}
Json saved_vision_parameters() {
    const auto path=vision_config_path();
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        if (std::filesystem::exists(path))
            throw std::runtime_error("无法读取视觉默认参数");
        return Json::object();
    }
    char data[16385];
    in.read(data, sizeof(data));
    const auto size = in.gcount();
    if (!in.eof() || size <= 0 || size > 16384)
        throw std::runtime_error("视觉默认参数文件无效");
    const auto root = Json::parse(data, data + size);
    if (root.at("version") != 1 || !root.at("parameters").is_object())
        throw std::runtime_error("视觉默认参数文件格式无效");
    const auto settings = vision_parameters(root.at("parameters"));
    for (auto it=root.at("parameters").begin();it!=root.at("parameters").end();++it)
        if(!settings.contains(it.key()) || it.key()=="kind" || it.key()=="lost_timeout_s" || it.key()=="max_distance_m")
            throw std::runtime_error("未知视觉默认参数");
    if (root.at("parameters").size() < settings.size() - 8)
        throw std::runtime_error("视觉默认参数字段不完整");
    for (auto it = settings.begin(); it != settings.end(); ++it)
        if (it.key() != "kind" && it.key() != "lost_timeout_s" && it.key() != "max_distance_m" &&
            it.key().rfind("lateral_",0)!=0 &&
            !root.at("parameters").contains(it.key()))
            throw std::runtime_error("视觉默认参数字段不完整");
    return settings;
}
} // namespace

Json Controller::vision_defaults() {
    return vision_parameters(Json::object(), saved_vision_parameters());
}
Json Controller::vision_snapshot() {
    std::lock_guard<std::mutex> lock(vision_mutex);
    const double age = now_s() - number(vision_sample, "written_monotonic", 0);
    const bool retained = vision_sample.value("target_retained", false) &&
                          now_s() - number(vision_sample, "last_observed_monotonic", 0) <=
                              vision_sample.value("tracking_retention_seconds", .5);
    if ((age < 0 || age > .5 || vision_sample.value("source", std::string()) != "ros2_cpp") &&
        !retained)
        return {{"camera_ready", false}, {"target_found", false}, {"message", "视觉快照已过期"}};
    return vision_sample;
}

void Controller::check_vision_handover(const Json &s) {
    const auto mode = s.value("mode", std::string());
    if (!s.value("connected", false) || s["armed"] != true || !vision_airborne(s) ||
        (mode != "AUTO_TAKEOFF" && mode != "LOITER" && mode != "OFFBOARD"))
        throw std::runtime_error("视觉接管取消：连接、空中状态或飞控模式已改变");
}

// Fixed local snapshot bridge: ROS owns image measurement, this service owns MAVLink and task
// arbitration.
void Controller::read_vision() {
    name_thread("vision-cache");
    VisionTrack tracker(vision_tracking_config);
    std::filesystem::file_time_type config_stamp{};
    double config_check=0;
    const char *configured = std::getenv("DRONE_VISION_SNAPSHOT_DIR");
    const std::string path =
        std::string(configured ? configured : "/dev/shm/drone_vision") + "/status.json";
    while (!stopping) {
        if(now_s()>=config_check) {
            config_check=now_s()+.25;
            std::error_code error;
            const auto stamp=std::filesystem::last_write_time(vision_config_path(),error);
            if(!error && stamp!=config_stamp) {
                config_stamp=stamp;
                try {
                    auto shared=saved_vision_parameters();
                    // Phase enable flags belong to the task, gains/amplitudes share one source.
                    for(const auto* key:{"kind","vertical_enabled","distance_enabled","lost_timeout_s","max_distance_m"})
                        shared.erase(key);
                    perform_vision("VISION_UPDATE",shared,"shared-defaults");
                } catch(const std::exception&) {
                    // No active visual task, or invalid file: never start a controller from a file write.
                }
            }
        }
        Json sample = Json::object();
        try {
            std::ifstream in(path, std::ios::binary);
            char buffer[65537];
            in.read(buffer, sizeof(buffer));
            const auto count = in.gcount();
            if (!in.eof() || count <= 0 || count > 65536)
                throw std::runtime_error("Vision snapshot unavailable");
            sample = Json::parse(buffer, buffer + count);
            if (!sample.is_object())
                sample = Json::object();
        } catch (...) {
        }
        try {
            sample = tracker.update(std::move(sample), now_s());
        } catch (...) {
            sample = tracker.update(Json::object(), now_s());
        }
        {
            std::lock_guard<std::mutex> lock(vision_mutex);
            vision_sample = std::move(sample);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

Json Controller::vision_measurement(double /*now*/, double /*age_limit*/) {
    Json value = vision_snapshot();
    if (value.value("source", std::string()) != "ros2_cpp" || !value.value("camera_ready", false))
        return {{"valid", false},
                {"reason", "等待机载检测测量"},
                {"retained", value.value("target_retained", false)}};
    const double age = number(value, "measurement_age_s", 1e6);
    // Fresh detection of an empty scene is normal: enable/keep the controller in position hold.
    if (!value.value("target_found", false) || !value.value("target_observed", false))
        return {
            {"valid", false}, {"age_s", age}, {"retained", value.value("target_retained", false)}};
    if (value.value("tag_id", -1) != 0)
        return {{"valid", false}};
    const double w = number(value, "width", 0), h = number(value, "height", 0);
    if (w < 2 || h < 2 || w > 4096 || h > 4096 || !value.contains("center") ||
        !value["center"].is_array() || value["center"].size() != 2)
        return {{"valid", false}};
    for (const auto &n : value["center"])
        if (!n.is_number() || !std::isfinite(n.get<double>()))
            return {{"valid", false}};
    const double u = value["center"][0], v = value["center"][1];
    if (u < 0 || u >= w || v < 0 || v >= h)
        return {{"valid", false}};
    Json result = {{"valid", true},
                   {"ex", (u - (w - 1) * .5) / (w * .5)},
                   {"ey", (v - (h - 1) * .5) / (h * .5)},
                   {"age_s", age},
                   {"pose_valid", false}};
    if (value.value("pose_valid", false) && value.value("calibrated", false) &&
        value.contains("distance_m") && value["distance_m"].is_number() &&
        value.contains("tag_size_m") && value["tag_size_m"].is_number()) {
        const double distance = number(value, "distance_m", 0),
                     side = number(value, "tag_size_m", 0);
        if (distance > 0 && side > 0) {
            result["pose_valid"] = true;
            result["distance_tag_units"] = distance / side;
        }
    }
    return result;
}

Json Controller::perform_vision(const std::string &command, const Json &p, const std::string &id) {
    auto s = px4.status();
    std::unique_lock<std::mutex> lock(mutex);
    const bool active =
        task.value("state", std::string()) == "running" &&
        task.value("params", Json::object()).value("kind", std::string()) == "vision_follow";
    if (command == "VISION_STOP") {
        if (active)
            finish("cancelled", "视觉控制停止，捕获当前位置与航向保持", s);
        return {{"ok", true}, {"confirmed", true}, {"message", "视觉控制已停止；不会取消其他任务"}};
    }
    if (command == "VISION_UPDATE") {
        if (!active)
            throw std::runtime_error("视觉控制尚未运行；新参数将在下次启动时提交");
        const auto settings = vision_parameters(p, task.at("params"));
        const bool handoff = task["params"].value("search_approach", false) &&
                             !settings.value("search_approach", false);
        auto changed = [&](std::initializer_list<const char *> keys) {
            for (auto key : keys)
                if (settings[key] != task["params"][key])
                    return true;
            return false;
        };
        if (changed({"yaw_gain", "yaw_ki", "yaw_kd", "yaw_speed_rad_s"}))
            vision_yaw_pid.reset();
        if (changed({"vertical_gain", "vertical_ki", "vertical_kd", "vertical_speed_m_s",
                     "vertical_enabled"}))
            vision_vertical_pid.reset();
        if(changed({"lateral_gain","lateral_ki","lateral_kd","lateral_speed_m_s","lateral_enabled"}))
            vision_lateral_pid.reset();
        if (changed({"distance_gain", "distance_ki", "distance_kd", "forward_speed_m_s",
                     "distance_enabled", "target_distance_m", "tag_size_m"}) ||
            settings.value("distance_constant_speed", false) != task["params"].value("distance_constant_speed", false) ||
            settings.value("distance_tolerance_m", .1) != task["params"].value("distance_tolerance_m", .1)) {
            vision_distance_pid.reset();
            vision_loss_started = 0;
            if (!handoff) {
                vision_last_forward = 0;
                vision_last_forward_velocity = {0, 0, 0};
            }
        }
        task["params"] = settings;
        task["message"] = "视觉PID、目标距离与速度设置已更新";
        return {{"ok", true},
                {"confirmed", true},
                {"params", task["params"]},
                {"message", "机载C++已应用视觉控制参数"}};
    }
    const bool taking_off =
        task.value("state", std::string()) == "running" &&
        task.value("params", Json::object()).value("kind", std::string()) == "auto_takeoff";
    const auto kind = task.value("params", Json::object()).value("kind", std::string());
    const bool search_takeover =
        p.value("search_approach", false) && (kind == "rotate" || kind == "move");
    if (active || sticks || velocity_active ||
        (task.value("state", std::string()) == "running" && !taking_off && !search_takeover) ||
        task.value("state", std::string()) == "landing")
        throw std::runtime_error("Stop current controller before starting vision");
    const auto mode = s.value("mode", std::string());
    if (!s.value("connected", false) || s["armed"] != true)
        throw std::runtime_error("视觉控制未启动：飞控未连接或未解锁");
    if (!vision_airborne(s))
        throw std::runtime_error("视觉控制未启动：飞控未确认空中或起飞中");
    if (mode != "OFFBOARD" && mode != "AUTO_TAKEOFF" && mode != "LOITER")
        throw std::runtime_error("视觉控制未启动：当前模式为" + mode +
                                 "，不接管降落或其他人工模式");
    Json settings = vision_parameters(p, saved_vision_parameters());
    const bool offboard_ready = streaming && mode == "OFFBOARD" && stream_started > 0 &&
                                now_s() - stream_started >= 1.5 && sent_at > 0 &&
                                now_s() - sent_at < .3;
    if (taking_off)
        finish("cancelled", "用户请求视觉接管，停止原自动起飞观察任务", s, false);
    if (search_takeover)
        finish("cancelled", "搜索发现目标，连续接近接管原扫描", s, false);
    vision_origin = position(s);
    vision_z = vision_origin[2];
    vision_heading = yaw(s);
    target = Setpoint{};
    target.p = vision_origin;
    target.heading = vision_heading;
    holding = true;
    vision_vz = vision_rate = 0;
    vision_yaw_pid.reset();
    vision_vertical_pid.reset();
    vision_distance_pid.reset();
    vision_lateral_pid.reset();
    vision_xy_moving = false;
    vision_loss_started = vision_last_forward = 0;
    vision_last_forward_velocity = {0, 0, 0};
    vision_search_seen = settings.value("search_observed_at", 0.);
    vision_search_bearing = settings.value("search_bearing_deg", 0.) * PI / 180;
    if (settings.value("search_approach", false)) {
        const auto measured = velocity(s);
        vision_last_forward_velocity = {measured[0], measured[1], 0};
    }
    task_command_id = id;
    begun = now_s();
    task = {{"id", id},
            {"state", "running"},
            {"phase", "handover"},
            {"params", settings},
            {"message", "视觉接管：预发送当前位置保持，再确认OFFBOARD"}};
    streaming = true;
    stream_error.clear();
    const Json response = {{"ok", true},
                           {"accepted", true},
                           {"confirmed", false},
                           {"stage", "running"},
                           {"message", "C++视觉控制已启动，等待实际反馈"}};
    if (offboard_ready) {
        task["phase"] =
            vision_measurement(now_s(), 0).value("valid", false) ? "tracking" : "waiting_target";
        task["message"] = "复用已确认的OFFBOARD控制流，视觉控制直接衔接";
        return response;
    }
    stream_started = sent_at = 0;
    lock.unlock();
    try {
        bool prepared = false;
        for (int i = 0; i < 60 && !stopping; ++i) {
            {
                std::lock_guard<std::mutex> guard(mutex);
                if (!streaming || task_command_id != id ||
                    task.value("state", std::string()) != "running")
                    throw std::runtime_error("视觉接管已停止：" +
                                             task.value("message", std::string()));
                check_vision_handover(px4.status());
                prepared =
                    stream_started > 0 && now_s() - stream_started >= 1.5 && now_s() - sent_at < .3;
            }
            if (prepared)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (!prepared)
            throw std::runtime_error("视觉OFFBOARD位置保持预发送未完成");
        if (px4.status().value("mode", std::string()) != "OFFBOARD") {
            const auto result = confirmed(176, {1, 6, 0, 0, 0, 0, 0}, [](auto &state) {
                return state.value("mode", std::string()) == "OFFBOARD";
            });
            if (!result.value("confirmed", false))
                throw std::runtime_error("飞控未确认视觉OFFBOARD接管；不自动重发");
        }
        s = px4.status();
        lock.lock();
        check_vision_handover(s);
        if (!streaming || task.value("state", std::string()) != "running" ||
            s.value("mode", std::string()) != "OFFBOARD")
            throw std::runtime_error("视觉接管未完成");
        const auto latest = vision_measurement(now_s(), 0);
        task["phase"] = latest.value("valid", false) ? "tracking" : "waiting_target";
        task["message"] = "OFFBOARD已确认，视觉控制已启用；无目标时保持位置";
        begun = now_s();
    } catch (const std::exception &e) {
        if (!lock.owns_lock())
            lock.lock();
        streaming = holding = false;
        target = Setpoint{};
        if (task_command_id == id && task.value("state", std::string()) == "running")
            finish("failed", e.what(), px4.status(), false);
        throw;
    }
    return response;
}

void Controller::update_vision(const Json &s, double now, double dt) {
    try {
        const auto &p = task.at("params");
        if (!vision_airborne(s))
            throw std::runtime_error(
                "视觉空中反馈不可用：landed_state=" + std::to_string(s.value("landed_state", 0)) +
                ", age_s=" + std::to_string(number(s, "landed_age_s", -1)) +
                ", range_m=" + std::to_string(number(s, "range_m", -1)));
        const Vec actual = position(s);
        // Visual following has no elapsed-time or accumulated-displacement cutoff.
        // Flight/measurement validity and the user-selected command speeds still apply.
        const auto measure = vision_measurement(now, 0);
        task["elapsed_s"] = now - begun;
        task["vision_measurement"] = measure;
        const bool metric = measure.value("pose_valid", false);
        const double distance =
            metric ? measure["distance_tag_units"].get<double>() * p["tag_size_m"].get<double>()
                   : 0.;
        const bool outside = metric && p.value("max_distance_m", 0.) > 0 &&
                             distance > p["max_distance_m"].get<double>();
        if (p.value("search_approach", false)) {
            const auto v = vision_snapshot();
            const bool observed = measure.value("valid", false);
            const double stamp = observed ? v.value("last_observed_monotonic", 0.) : 0;
            if (observed && stamp > vision_search_seen) {
                vision_search_bearing = target_heading(v, s) * PI / 180;
                vision_search_seen = stamp;
            }
            const double age = std::max(0., now - vision_search_seen);
            const bool recent = age < p["lost_timeout_s"].get<double>();
            const double rate = p["search_turn_rate_deg_s"].get<double>() * PI / 180;
            const double angle = p["search_scan_angle_deg"].get<double>() * PI / 180;
            const double scan = recent
                                    ? 0
                                    : angle * std::sin((age - p["lost_timeout_s"].get<double>()) *
                                                       rate / std::max(angle, .001));
            if (!std::isfinite(vision_heading))
                vision_heading = yaw(s);
            vision_heading = wrap(vision_heading +
                                  std::clamp(wrap(vision_search_bearing + scan - vision_heading),
                                             -rate * dt, rate * dt));
            const double speed =
                recent && !outside && (!metric || distance > p["target_distance_m"].get<double>())
                    ? p["search_speed_m_s"].get<double>()
                    : 0;
            const Vec wanted = {speed * std::cos(vision_search_bearing),
                                speed * std::sin(vision_search_bearing), 0};
            const auto smooth = slew_velocity(vision_last_forward_velocity, wanted,
                                              p["search_acceleration"].get<double>() * dt);
            if (!std::isfinite(vision_z))
                vision_z = actual[2];
            vision_vz = observed && p.value("vertical_enabled", false)
                            ? vision_vertical_pid.step(measure["ey"], dt, p["vertical_gain"],
                                                       p["vertical_ki"], p["vertical_kd"],
                                                       p["vertical_speed_m_s"])
                            : 0;
            if (vision_xy_moving && norm(smooth) <= 1e-6)
                vision_origin = actual;
            vision_xy_moving = norm(smooth) > 1e-6;
            if (vision_vz != 0)
                vision_z = NAN_VALUE;
            target = Setpoint{};
            target.p = {vision_xy_moving ? NAN_VALUE : vision_origin[0],
                        vision_xy_moving ? NAN_VALUE : vision_origin[1],
                        vision_vz ? NAN_VALUE : vision_z};
            target.v = {smooth[0], smooth[1], vision_vz};
            target.heading = vision_heading;
            holding = true;
            vision_last_forward_velocity = smooth;
            vision_last_forward = norm(smooth);
            task["phase"] = observed ? "search_approach"
                            : recent ? "search_memory_forward"
                                     : "search_reacquire";
            task["lost_elapsed_s"] = observed ? 0. : age;
            task["search_bearing_deg"] = vision_search_bearing * 180 / PI;
            task["vision_output"] = {{"forward_m_s", vision_last_forward},
                                     {"vertical_m_s", vision_vz},
                                     {"yaw_rad_s", 0.}};
            task["message"] = observed ? "发现目标：连续缓慢接近，平滑修正方向"
                              : recent ? "短暂漏检：沿记忆方向继续接近，不重启任务"
                                       : "接近等待重捕获：保持位置并在记忆方位附近搜索，不重启任务";
            return;
        }
        const bool distance_lost =
            p.value("distance_enabled", false) && (!metric || !measure.value("valid", false));
        if (distance_lost && !outside) {
            if (!vision_loss_started) {
                vision_loss_started = now;
                vision_z = actual[2];
                vision_heading = yaw(s);
            }
            const double elapsed = now - vision_loss_started;
            task["lost_elapsed_s"] = elapsed;
            if (vision_last_forward > 0 && elapsed < p.value("lost_timeout_s", 3.)) {
                // Continue only the last valid forward vector; freeze altitude
                // and heading rather than integrating stale pixel corrections.
                target = Setpoint{};
                target.p = {NAN_VALUE, NAN_VALUE, vision_z};
                target.v = vision_last_forward_velocity;
                target.heading = vision_heading;
                target.rate = 0;
                holding = true;
                vision_xy_moving = true;
                task["phase"] = "lost_target_forward";
                task["vision_output"] = {
                    {"forward_m_s", vision_last_forward}, {"vertical_m_s", 0.}, {"yaw_rad_s", 0.}};
                task["message"] = "目标短时丢失，沿丢失前方向和前进速度续行；超时后停止续行";
                return;
            }
        } else if (!outside) {
            vision_loss_started = 0;
            task["lost_elapsed_s"] = 0.;
        }
        if (outside || (distance_lost && !measure.value("valid", false))) {
            // Expired grace, no prior forward command, or explicit range gate:
            // do not synthesize a new direction or restart the controller.
            if (task.value("phase", std::string()) != "waiting_target") {
                vision_origin = actual;
                vision_z = actual[2];
                vision_heading = yaw(s);
            }
            vision_xy_moving = false;
            target = Setpoint{};
            target.p = {vision_origin[0], vision_origin[1], vision_z};
            target.v = {0, 0, 0};
            target.heading = vision_heading;
            holding = true;
            task["phase"] = "waiting_target";
            task["vision_output"] = {{"forward_m_s", 0.}, {"vertical_m_s", 0.}, {"yaw_rad_s", 0.}};
            task["message"] = outside ? "目标超出配置距离范围，等待恢复"
                                      : "丢失续行已结束或尚无前进方向，等待目标";
            return;
        }
        if (!measure.value("valid", false)) {
            // Capture Z/yaw once on loss; never retain the last visual velocity or chase missing
            // pixels.
            if (task.value("phase", std::string()) != "waiting_target") {
                vision_z = actual[2];
                vision_heading = yaw(s);
                if (p.value("distance_enabled", false) || vision_xy_moving) {
                    vision_origin[0] = actual[0];
                    vision_origin[1] = actual[1];
                }
            }
            vision_xy_moving = false;
            if (!measure.value("retained", false)) {
                vision_yaw_pid.reset();
                vision_vertical_pid.reset();
                vision_distance_pid.reset();
                vision_lateral_pid.reset();
            }
            vision_vz = vision_rate = 0;
            target = Setpoint{};
            target.p = {vision_origin[0], vision_origin[1], vision_z};
            target.v = {0, 0, 0};
            target.heading = vision_heading;
            target.rate = 0;
            holding = true;
            task["phase"] = "waiting_target";
            task["vision_output"] = {{"forward_m_s", 0.}, {"vertical_m_s", 0.}, {"yaw_rad_s", 0.}};
            task["message"] = "视觉控制已启用，等待目标；保持位置，检测到有效目标后自动继续";
            return;
        }
        task["phase"] = "tracking";
        const double ex = measure["ex"], ey = measure["ey"];
        // Camera optical u right/v down. PX4 FRD yaw right and down velocity are positive.
        vision_rate = vision_yaw_pid.step(ex, dt, p["yaw_gain"], p["yaw_ki"], p["yaw_kd"],
                                          p["yaw_speed_rad_s"]);
        vision_vz = p["vertical_enabled"].get<bool>()
                        ? vision_vertical_pid.step(ey, dt, p["vertical_gain"], p["vertical_ki"],
                                                   p["vertical_kd"], p["vertical_speed_m_s"])
                        : 0;
        double forward = 0;
        const bool constant_distance = p.value("distance_constant_speed", false) &&
                                       p["forward_speed_m_s"].get<double>() > 0;
        double lateral=p["lateral_enabled"].get<bool>()
            ? vision_lateral_pid.step(ex,dt,p["lateral_gain"],p["lateral_ki"],p["lateral_kd"],p["lateral_speed_m_s"])
            : 0;
        Json telemetry = measure;
        if (p["distance_enabled"].get<bool>() && measure.value("pose_valid", false)) {
            const double distance =
                measure["distance_tag_units"].get<double>() * p["tag_size_m"].get<double>();
            const double error = distance - p["target_distance_m"].get<double>();
            telemetry["distance_m"] = distance;
            telemetry["distance_error_m"] = error;
            forward = constant_distance
                ? (std::abs(error) <= p["distance_tolerance_m"].get<double>() ? 0. :
                   std::copysign(p["forward_speed_m_s"].get<double>(), error))
                : vision_distance_pid.step(error, dt, p["distance_gain"], p["distance_ki"],
                                           p["distance_kd"], p["forward_speed_m_s"]);
        } else
            vision_distance_pid.reset();
        if (vision_rate != 0)
            vision_heading = NAN_VALUE;
        else if (!std::isfinite(vision_heading))
            vision_heading = yaw(s);
        if (vision_vz != 0)
            vision_z = NAN_VALUE;
        else if (!std::isfinite(vision_z))
            vision_z = actual[2];
        target = Setpoint{};
        target.p = {vision_origin[0], vision_origin[1], vision_z};
        target.v = {0, 0, vision_vz};
        target.heading = vision_heading;
        target.rate = vision_rate;
        if (forward != 0 || lateral != 0) {
            target.p[0] = target.p[1] = NAN_VALUE;
            target.v[0] = forward * std::cos(yaw(s)) - lateral * std::sin(yaw(s));
            target.v[1] = forward * std::sin(yaw(s)) + lateral * std::cos(yaw(s));
            vision_xy_moving = true;
        } else if (vision_xy_moving) {
            vision_origin[0] = actual[0];
            vision_origin[1] = actual[1];
            target.p[0] = actual[0];
            target.p[1] = actual[1];
            vision_xy_moving = false;
        }
        if (p.contains("search_acceleration") && !constant_distance) {
            const auto smooth =
                slew_velocity(vision_last_forward_velocity, {target.v[0], target.v[1], 0},
                              p["search_acceleration"].get<double>() * dt);
            target.v[0] = smooth[0];
            target.v[1] = smooth[1];
            if (norm(smooth) > 1e-6) {
                target.p[0] = target.p[1] = NAN_VALUE;
                vision_xy_moving = true;
            }
            forward = smooth[0] * std::cos(yaw(s)) + smooth[1] * std::sin(yaw(s));
            lateral = -smooth[0] * std::sin(yaw(s)) + smooth[1] * std::cos(yaw(s));
        }
        vision_last_forward = forward;
        vision_last_forward_velocity = {target.v[0], target.v[1], 0};
        holding = true;
        task["elapsed_s"] = now - begun;
        task["vision_measurement"] = telemetry;
        task["forward_control_mode"] = constant_distance ? "constant_speed" : "distance_pid";
        task["vision_output"] = {
            {"forward_m_s", forward}, {"lateral_m_s", lateral}, {"vertical_m_s", vision_vz}, {"yaw_rad_s", vision_rate}};
        task["message"] = p["distance_enabled"].get<bool>()
                              ? measure.value("pose_valid", false)
                                    ? (constant_distance ? "视觉偏航、升降与前后恒速距离控制执行中" : "视觉偏航、升降与前后距离PID执行中")
                                    : "偏航/升降执行中；等待有效米制距离"
                              : "视觉偏航与升降执行中，前后距离控制关闭";
    } catch (const std::exception &e) {
        vision_vz = vision_rate = 0;
        finish("failed", std::string(e.what()) + "；视觉动作已停止并保持，需手动重新启动", s);
    }
}
