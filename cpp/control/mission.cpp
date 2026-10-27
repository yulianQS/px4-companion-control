#include "mission.hpp"
#include <filesystem>
#include <regex>

namespace {
struct Cancelled {};
std::string uid() {
    static std::atomic<unsigned> n{0};
    return "mission-" + std::to_string(uint64_t(now_s() * 1000000)) + "-" + std::to_string(n++);
}
std::string next(const Json &b) {
    return b.contains("next") && b["next"].is_string() ? b["next"].get<std::string>() : "";
}
Json load(const std::filesystem::path &p) {
    std::ifstream f(p);
    if (!f)
        throw std::runtime_error("无法读取配置：" + p.string());
    return Json::parse(f);
}
Json validate(Json doc) {
    if (doc.value("version", 0) != 1 || !doc.contains("blocks") || !doc["blocks"].is_array() ||
        doc["blocks"].empty() || doc["blocks"].size() > 200 || doc.dump().size() > 262144)
        throw std::runtime_error("比赛任务格式/容量无效");
    const auto catalogue = load(project_root() / "config/mission_blocks.json");
    Json defs = Json::object();
    for (const auto &d : catalogue["blocks"])
        defs[d["type"].get<std::string>()] = d;
    const std::regex id("^[A-Za-z0-9_-]{1,100}$");
    if (!doc.contains("id") || !doc["id"].is_string() ||
        !std::regex_match(doc["id"].get<std::string>(), id) || !doc.contains("name") ||
        !doc["name"].is_string() || doc["name"].get<std::string>().empty() ||
        doc["name"].get<std::string>().size() > 300)
        throw std::runtime_error("任务ID或名称无效");
    std::set<std::string> ids;
    for (auto &b : doc["blocks"]) {
        const auto key = b.value("id", std::string()), type = b.value("type", std::string());
        if (!std::regex_match(key, id) || !ids.insert(key).second || !defs.contains(type) ||
            !defs[type].value("executable", false))
            throw std::runtime_error("积木ID重复、未知类型或尚未接入");
        auto &p = b["params"];
        if (p.is_null())
            p = Json::object();
        if (!p.is_object())
            throw std::runtime_error("积木参数需要对象");
        std::set<std::string> keys;
        for (const auto &f : defs[type]["fields"]) {
            const std::string k = f["key"], kind = f["kind"];
            keys.insert(k);
            if (!p.contains(k))
                p[k] = f["default"];
            const auto &v = p[k];
            if (kind == "optional_number" && v.is_null())
                continue;
            if (kind == "number" || kind == "integer" || kind == "optional_number") {
                if (!v.is_number() || !std::isfinite(v.get<double>()) ||
                    (kind == "integer" && std::floor(v.get<double>()) != v.get<double>()) ||
                    v.get<double>() < f.value("min", -std::numeric_limits<double>::infinity()) ||
                    v.get<double>() > f.value("max", std::numeric_limits<double>::infinity()))
                    throw std::runtime_error("积木数值超范围：" + k);
            } else if (kind == "select" || kind == "ref" || kind == "text") {
                if (!v.is_string() || v.get<std::string>().size() > 2000)
                    throw std::runtime_error("积木文本无效：" + k);
                if (kind == "select" &&
                    std::find(f["options"].begin(), f["options"].end(), v) == f["options"].end())
                    throw std::runtime_error("积木选项无效：" + k);
            }
        }
        for (auto it = p.begin(); it != p.end(); ++it)
            if (!keys.count(it.key()))
                throw std::runtime_error("未知积木参数：" + it.key());
        if (p.contains("target_id") && p["target_id"] != 0)
            throw std::runtime_error("当前视觉接口仅支持ID 0");
        if (p.contains("waypoints")) {
            const auto &w = p["waypoints"];
            if (!w.is_array() || w.empty() || w.size() > 100)
                throw std::runtime_error("航点数量需要1～100");
            for (const auto &r : w) {
                if (!r.is_array() || r.size() != 3)
                    throw std::runtime_error("航点需要XYZ");
                for (const auto &v : r)
                    if (!v.is_number() || !std::isfinite(v.get<double>()) ||
                        std::abs(v.get<double>()) > 10000)
                        throw std::runtime_error("航点超范围");
            }
        }
        if (b.contains("branches") && (!b["branches"].is_object() || !b["branches"].empty()))
            throw std::runtime_error("当前积木不支持条件分支");
        bounded(b, "timeout", 120, 2, 3600);
        b["timeout"] = b.value("timeout", 120.);
        b["name"] = b.value("name", defs[type]["name"].get<std::string>());
    }
    auto ref = [&](const Json &v) {
        if (!v.is_null() && v != "" && (!v.is_string() || !ids.count(v.get<std::string>())))
            throw std::runtime_error("任务引用不存在");
    };
    if (!doc.contains("entry") || doc["entry"].is_null())
        doc["entry"] = doc["blocks"][0]["id"];
    ref(doc["entry"]);
    for (const auto &b : doc["blocks"]) {
        if (b.contains("next"))
            ref(b["next"]);
        if (b["params"].contains("lost_target"))
            ref(b["params"]["lost_target"]);
    }
    return doc;
}
} // namespace

bool Mission::active() {
    std::lock_guard<std::mutex> l(mutex);
    return state.value("active", false);
}
Json Mission::status(bool full) {
    std::lock_guard<std::mutex> l(mutex);
    Json out = Json::object();
    for (auto it = state.begin(); it != state.end(); ++it)
        if (full || (it.key() != "document" && it.key() != "events"))
            out[it.key()] = it.value();
    if (out.value("active", false))
        out["elapsed_s"] = now_s() - begun;
    out["log_error"] = log.error();
    return out;
}
Mission::~Mission() {
    cancelled = true;
    wake.notify_all();
    if (worker.joinable())
        worker.join();
}
void Mission::event(const std::string &name, Json data) {
    data["event"] = name;
    data["elapsed_s"] = now_s() - begun;
    data["utc_iso"] = log_storage::utc();
    data["monotonic_s"] = now_s();
    data["boot_id"] = log_storage::boot_id();
    std::lock_guard<std::mutex> l(mutex);
    data["run_id"] = state.value("run_id", std::string());
    auto &events = state["events"];
    events.push_back(data);
    if (events.size() > 100)
        events.erase(events.begin());
    log.append(data.dump());
}
void Mission::pause(double seconds) {
    std::unique_lock<std::mutex> l(mutex);
    wake.wait_for(l, std::chrono::duration<double>(seconds), [&] { return cancelled.load(); });
    if (cancelled)
        throw Cancelled{};
}
Json Mission::command(const std::string &name, Json p, bool check_cancel) {
    if (check_cancel && cancelled)
        throw Cancelled{};
    const auto id = uid();
    sent.insert(id);
    if (name != "CLAIM" && name != "RENEW")
        actuated = true;
    auto r = control.execute(name, p, owner, id, true);
    if (!r.value("ok", true))
        throw std::runtime_error(r.value("message", "底层命令失败"));
    event("command", {{"command", name},
                      {"request_id", id},
                      {"stage", r.value("stage", std::string())},
                      {"confirmed", r.value("confirmed", false)}});
    r["id"] = id;
    return r;
}
Json Mission::check(double deadline) {
    if (cancelled)
        throw Cancelled{};
    if (now_s() >= deadline)
        throw std::runtime_error("积木超时，未报告完成");
    auto d = control.status()["data"];
    const auto &px = d["px4"];
    if (flight) {
        if (!px.value("connected", false) || !px.value("bidirectional", false))
            throw std::runtime_error("飞控链路中断");
        const std::string type = current["type"], mode = px.value("mode", std::string());
        if (!landed && px["armed"] != true && type != "land")
            throw std::runtime_error("飞控已上锁");
        if (type == "takeoff" ? (mode != "OFFBOARD" && mode != "AUTO_TAKEOFF" && mode != "LOITER")
            : (type == "land" || landed) ? (mode != "OFFBOARD" && mode != "LAND")
                                         : mode != "OFFBOARD")
            throw std::runtime_error("任务期间飞控模式改变");
        if (claimed && now_s() >= renew_at) {
            command("RENEW");
            renew_at = now_s() + 4;
        }
    }
    return d;
}
Json Mission::target(bool pose) {
    auto v = control.vision_snapshot();
    return (v.value("camera_ready", false) || v.value("target_retained", false)) &&
                   v.value("target_found", false) && v.value("tag_id", -1) == 0 &&
                   (!pose || v.value("pose_valid", false))
               ? v
               : Json();
}
void Mission::track_target(const Json &target, double missing, const std::string &message) {
    std::lock_guard<std::mutex> l(mutex);
    state["blocks"][current["id"].get<std::string>()]["status"] = missing ? "waiting" : "running";
    state["lost_elapsed_s"] = missing ? now_s() - missing : 0;
    if (!target.is_null())
        state["retained_target"] = target;
    if (!message.empty())
        state["message"] = message;
}
bool Mission::motion(const std::string &name, Json p, double deadline, bool search) {
    auto r = command(name, p);
    const std::string id = r["id"];
    double feedback = 0;
    std::string previous_phase;
    while (true) {
        auto d = check(deadline), m = d["motion"];
        if (m.value("id", std::string()) != id)
            throw std::runtime_error("底层任务ID已改变");
        const auto found = search ? target() : Json();
        if (!found.is_null() && found.value("target_observed", false)) {
            event("target_found", {{"target", found}});
            return true;
        }
        const std::string st = m.value("state", std::string());
        if (st == "succeeded") {
            if (name == "LAND") {
                if (d["px4"]["armed"] != false || d["px4"].value("landed_state", 0) != 1)
                    throw std::runtime_error("落地反馈不完整");
                landed = true;
            }
            return false;
        }
        if (st == "failed" || st == "timed_out" || st == "cancelled")
            throw std::runtime_error(m.value("message", "底层任务失败"));
        {
            std::lock_guard<std::mutex> l(mutex);
            state["block_progress"] = m.value("progress", Json());
            state["message"] = m.value("message", std::string());
        }
        const auto phase = m.value("phase", std::string());
        if (now_s() >= feedback || phase != previous_phase) {
            event("motion_feedback", {{"request_id", id},
                                      {"command", name},
                                      {"state", st},
                                      {"phase", m.value("phase", Json())},
                                      {"message", m.value("message", Json())},
                                      {"measured", m.value("measured", Json())},
                                      {"error", m.value("error", Json())},
                                      {"completion", m.value("completion", Json())},
                                      {"mode", d["px4"]["mode"]}});
            feedback = now_s() + 2;
            previous_phase = phase;
        }
        pause();
    }
}
bool Mission::acquire(const Json &b, double deadline) {
    const auto &p = b["params"];
    auto seed = target();
    if (seed.is_null())
        return false;
    double bearing = target_heading(seed, check(deadline)["px4"]);
    double evidence = 0, previous = 0;
    unsigned epoch = seed.value("tracking_epoch", 0u);
    double feedback = 0;
    bool observed = false;
    const auto following = next(b);
    const bool recognition = !following.empty() && (nodes.at(following)["type"] == "recognize" ||
                                                    nodes.at(following)["type"] == "lock" ||
                                                    nodes.at(following)["type"] == "apriltag");
    const bool need_pose = recognition && nodes.at(following)["type"] == "apriltag";
    const double required = recognition ? nodes.at(following)["params"].value("settle_seconds", .5)
                                        : p["confirm_seconds"].get<double>();
    auto settings = control.vision_defaults();
    settings.update(Json{{"search_approach", true},
                         {"search_bearing_deg", bearing},
                         {"search_observed_at", seed.value("last_observed_monotonic", 0.)},
                         {"search_speed_m_s", p["approach_speed"]},
                         {"search_acceleration", p["approach_acceleration"]},
                         {"search_turn_rate_deg_s", p["scan_speed"]},
                         {"search_scan_angle_deg", p["local_scan_angle"]},
                         {"lost_timeout_s", p["local_search_seconds"]},
                         {"max_distance_m", 0.},
                         {"vertical_enabled", false},
                         {"distance_enabled", false}});
    if (vision_session.empty())
        vision_session = command("VISION_START", settings)["id"];
    else
        command("VISION_UPDATE", settings);
    search_confirmation = Json();
    event("search_approach_started",
          {{"block_id", b["id"]}, {"bearing_deg", bearing}, {"session_id", vision_session}});
    while (true) {
        const auto d = check(deadline), v = target(), m = d["motion"];
        if (m.value("id", std::string()) != vision_session ||
            m.value("state", std::string()) != "running")
            throw std::runtime_error(m.value("message", "搜索接近控制已停止"));
        const bool fresh = !v.is_null() && v.value("target_observed", false);
        const double stamp = fresh ? v.value("last_observed_monotonic", 0.) : 0;
        const bool usable = fresh && (!need_pose || v.value("pose_valid", false));
        if (fresh && epoch != v.value("tracking_epoch", 0u)) {
            epoch = v.value("tracking_epoch", 0u);
            evidence = previous = 0;
            observed = false;
        }
        if (fresh && stamp > previous) {
            bearing = target_heading(v, d["px4"]);
            if (!need_pose)
                evidence = v.value("confirmation_seconds", 0.);
            else if (observed && usable)
                evidence += std::min(.1, stamp - previous);
            previous = stamp;
        }
        observed = usable;
        if (usable && evidence >= required) {
            search_confirmation = v;
            search_confirmation["confirmation_seconds"] = evidence;
            search_confirmation["search_bearing_deg"] = bearing;
            search_confirmation["pose_confirmed"] = need_pose;
            event("search_confirmed", {{"block_id", b["id"]},
                                       {"bearing_deg", bearing},
                                       {"confirmation_seconds", evidence},
                                       {"tracking_epoch", epoch},
                                       {"session_id", vision_session}});
            return true;
        }
        if (now_s() >= feedback) {
            event("search_feedback", {{"block_id", b["id"]},
                                      {"phase", m.value("phase", std::string())},
                                      {"target_observed", fresh},
                                      {"tracking_epoch", epoch},
                                      {"confirmation_seconds", evidence},
                                      {"confirmation_required_s", required},
                                      {"lost_elapsed_s", m.value("lost_elapsed_s", 0.)}});
            feedback = now_s() + 2;
        }
        track_target(v, 0, m.value("message", std::string()));
        {
            std::lock_guard<std::mutex> lock(mutex);
            state["search_phase"] = m.value("phase", std::string());
            state["search_bearing_deg"] = bearing;
            state["lost_elapsed_s"] = m.value("lost_elapsed_s", 0.);
            state["confirmation_seconds"] = evidence;
            state["confirmation_required_s"] = required;
        }
        pause();
    }
}
std::string Mission::lost(const Json &b) {
    const auto dest = b["params"].value("lost_target", std::string());
    event("target_lost", {{"block_id", b["id"]}, {"jump", dest}});
    if (dest.empty())
        throw std::runtime_error("目标连续丢失，未配置恢复跳转");
    return dest;
}
std::string Mission::observe(const Json &b, double deadline, bool pose) {
    const auto &p = b["params"];
    // Search already fulfilled its immediate recognition successor's requirement.
    // Keep old saved graphs working without confirming the same evidence twice.
    check(deadline);
    if (!last_search_id.empty() && next(nodes.at(last_search_id)) == b["id"] &&
        !search_confirmation.is_null() &&
        (!pose || search_confirmation.value("pose_confirmed", false)) &&
        search_confirmation.value("confirmation_seconds", 0.) >= p.value("settle_seconds", .1)) {
        event("target_observed", {{"block_id", b["id"]},
                                  {"reused_search_confirmation", true},
                                  {"target", search_confirmation}});
        search_confirmation = Json();
        return next(b);
    }
    double evidence = 0, previous = 0, missing = 0;
    unsigned epoch = 0;
    bool observed = false;
    while (true) {
        check(deadline);
        auto v = target(pose);
        const bool fresh = !v.is_null() && v.value("target_observed", false);
        if (fresh) {
            const auto current_epoch = v.value("tracking_epoch", 0u);
            const double stamp = v.value("last_observed_monotonic", 0.);
            if (current_epoch != epoch) {
                epoch = current_epoch;
                evidence = pose ? 0 : v.value("confirmation_seconds", 0.);
                previous = 0;
            }
            if (!pose)
                evidence = v.value("confirmation_seconds", 0.);
            else if (observed && stamp > previous)
                evidence += std::min(.1, stamp - previous);
            observed = true;
            previous = stamp;
            if (!search_confirmation.is_null() &&
                (!pose || search_confirmation.value("pose_confirmed", false)) &&
                v.value("tracking_epoch", 0u) == search_confirmation.value("tracking_epoch", 0u))
                evidence =
                    std::max(evidence, search_confirmation.value("confirmation_seconds", 0.));
            missing = 0;
            track_target(v, 0, "累计真实识别时间；短暂漏检不清零");
            if (evidence >= p.value("settle_seconds", .1)) {
                event("target_observed", {{"block_id", b["id"]}, {"target", v}});
                return next(b);
            }
        } else {
            observed = false;
            if (!missing)
                missing = now_s();
            track_target(v, missing, "短暂漏检保留识别进度，等待真实检测");
            if (p.contains("lost_timeout") &&
                now_s() - missing >= p["lost_timeout"].get<double>()) {
                stop_vision();
                return lost(b);
            }
            if (!last_search_id.empty() && target().is_null() &&
                now_s() - missing >= nodes.at(last_search_id)["params"]["local_search_seconds"].get<double>()) {
                event("recognition_research",
                      {{"block_id", b["id"]}, {"jump", last_search_id}});
                stop_vision();
                return last_search_id;
            }
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            state["confirmation_seconds"] = evidence;
            state["confirmation_required_s"] = p.value("settle_seconds", .1);
        }
        pause();
    }
}
Json Mission::visual_successor(std::string id) {
    std::set<std::string> visited;
    while (!id.empty() && visited.insert(id).second) {
        const auto &b = nodes.at(id);
        const std::string type = b["type"];
        if (type == "align" || type == "center" || type == "distance" || type == "follow")
            return b;
        if (type != "recognize" && type != "lock" && type != "apriltag")
            break;
        id = next(b);
    }
    return Json();
}
void Mission::start_vision(const Json &b, bool distance) {
    auto settings = control.vision_defaults();
    settings["distance_enabled"] = distance;
    settings["distance_constant_speed"] = b["type"] == "distance";
    settings["distance_tolerance_m"] = b["params"]["distance_tolerance"];
    settings["lost_timeout_s"] = b["params"]["lost_timeout"];
    settings["max_distance_m"] = b["params"]["max_distance"];
    const auto running = control.status()["data"]["motion"].value("params", Json::object());
    if (running.value("search_approach", false)) {
        settings["search_approach"] = !distance;
        if (!distance)
            settings["lost_timeout_s"] = running["lost_timeout_s"];
    }
    event("vision_parameters",
          {{"block_id", b["id"]}, {"parameters", settings}, {"session_id", vision_session}});
    if (vision_session.empty())
        vision_session = command("VISION_START", settings)["id"];
    else {
        const auto m = control.status()["data"]["motion"];
        if (m.value("id", std::string()) != vision_session ||
            m.value("state", std::string()) != "running")
            throw std::runtime_error(m.value("message", "视觉控制已停止"));
        command("VISION_UPDATE", settings);
    }
}
void Mission::stop_vision() {
    if (!vision_session.empty() &&
        control.status()["data"]["motion"].value("id", std::string()) == vision_session)
        command("VISION_STOP", Json::object(), false);
    vision_session.clear();
}
std::string Mission::visual(const Json &b, double deadline) {
    const auto &p = b["params"];
    const std::string type = b["type"];
    const bool distance = type == "distance" || type == "follow";
    const auto following = next(b);
    const bool coarse =
        type == "align" && !following.empty() &&
        (nodes.at(following)["type"] == "distance" || nodes.at(following)["type"] == "follow");
    const double horizontal = p[coarse ? "coarse_horizontal_threshold" : "horizontal_threshold"],
                 vertical = p["vertical_threshold"],
                 settle = p[coarse ? "coarse_settle_seconds" : "settle_seconds"];
    double stable = 0, previous = 0, missing = 0, started = now_s();
    unsigned epoch = 0;
    bool was_aligned = false;
    start_vision(b, distance);
    while (true) {
        auto d = check(deadline), v = target();
        const auto m = d["motion"];
        if (m.value("id", std::string()) != vision_session ||
            m.value("state", std::string()) != "running")
            throw std::runtime_error(m.value("message", "视觉控制已停止"));
        const auto& shared=m.at("params");
        bool pose = !v.is_null() && v.value("pose_valid", false) && v.value("calibrated", false) &&
                    v.value("distance_m", Json()).is_number();
        double metres = pose ? v["distance_m"].get<double>() * shared["tag_size_m"].get<double>() /
                                   v.value("tag_size_m", .12)
                             : 0;
        const bool usable = !v.is_null() && v.value("target_observed", false) &&
                            (!(distance || coarse) || (pose && metres > 0)) &&
                            (!pose || metres <= p["max_distance"].get<double>());
        if (!usable) {
            was_aligned = false;
            if (!missing) {
                missing = now_s();
                event(
                    "vision_unavailable",
                    {{"block_id", b["id"]}, {"target_found", !v.is_null()}, {"pose_valid", pose}});
            }
            track_target(v, missing, m.value("message", std::string()));
            if (now_s() - missing >= p["lost_timeout"].get<double>()) {
                const auto dest = lost(b);
                stop_vision();
                return dest;
            }
            pause();
            continue;
        }
        if (missing && started)
            started += now_s() - missing;
        missing = 0;
        track_target(v, 0, m.value("message", std::string()));
        const double ex = v["center"][0].get<double>() - (v["width"].get<double>() - 1) / 2,
                     ey = v["center"][1].get<double>() - (v["height"].get<double>() - 1) / 2;
        // Hand off after coarse heading alignment; vertical correction continues
        // concurrently with range control in the same vision session.
        bool aligned = std::abs(ex) <= horizontal &&
                       (coarse || !shared.value("vertical_enabled", false) ||
                        std::abs(ey) <= vertical);
        if (distance)
            aligned = aligned && std::abs(metres - shared["target_distance_m"].get<double>()) <=
                                     p["distance_tolerance"].get<double>();
        const auto stamp = v.value("last_observed_monotonic", 0.);
        const bool new_observation = stamp > previous;
        if (epoch != v.value("tracking_epoch", 0u) || !aligned)
            stable = 0;
        else if (was_aligned && stamp > previous)
            stable += std::min(.1, stamp - previous);
        epoch = v.value("tracking_epoch", 0u);
        previous = stamp;
        was_aligned = aligned;
        if (new_observation && aligned && stable >= settle &&
            (!distance || now_s() - started >= p["duration"].get<double>())) {
            event(coarse ? "coarse_alignment_handoff" : "vision_complete",
                  {{"block_id", b["id"]},
                   {"session_id", vision_session},
                   {"error_px", {ex, ey}},
                   {"distance_m", pose ? Json(metres) : Json()}});
            return next(b);
        }
        pause();
    }
}
std::string Mission::execute(const Json &b, double deadline) {
    const std::string type = b["type"];
    auto p = b["params"];
    p["timeout"] = b["timeout"];
    if (type == "takeoff") {
        if (p["speed"].is_null())
            p.erase("speed");
        motion("TAKEOFF", p, deadline);
    } else if (type == "land")
        motion("LAND", Json::object(), deadline);
    else if (type == "move") {
        p["kind"] = (p["direction"] == "up" || p["direction"] == "down") ? "height" : "move";
        p["level_before_move"] = true;
        motion("MOVE", p, deadline);
    } else if (type == "rotate") {
        p["kind"] = "rotate";
        p["distance"] = p["angle"];
        motion("MOVE", p, deadline);
    } else if (type == "position") {
        p["frame"] = "LOCAL_NED";
        p["unit"] = "cm";
        motion("SET_POSITION", p, deadline);
    } else if (type == "hover") {
        p["trajectory"] = "hover";
        motion("EXECUTE_TRAJECTORY", p, deadline);
    } else if (type == "trajectory" || type == "trajectory_loop") {
        const int count = p["repetitions"];
        p.erase("repetitions");
        if (p["fixed_yaw_deg"].is_null())
            p.erase("fixed_yaw_deg");
        p["trajectory"] = "waypoints";
        p["frame"] = "LOCAL_NED";
        p["unit"] = "cm";
        for (int i = 0; i < count; i++) {
            event("trajectory_iteration", {{"block_id", b["id"]}, {"iteration", i + 1}});
            motion("EXECUTE_TRAJECTORY", p, deadline);
        }
    } else if (type == "search") {
        last_search_id = b["id"];
        search_confirmation = Json();
        if (acquire(b, deadline))
            return next(b);
        for (int i = 0; i < p["max_attempts"].get<int>(); i++) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                state["search_phase"] = "wide_scan";
                state["search_bearing_deg"] = Json();
            }
            event("search_attempt", {{"block_id", b["id"]}, {"attempt", i + 1}});
            for (int j = 0; j < 3; j++)
                if (motion("MOVE",
                           {{"kind", "rotate"},
                            {"direction", j == 1 ? "counterclockwise" : "clockwise"},
                            {"distance", p["scan_angle"].get<double>() * (j == 1 ? 2 : 1)},
                            {"speed", p["scan_speed"]},
                            {"tolerance", std::min(3., p["scan_angle"].get<double>() / 2)},
                            {"timeout", b["timeout"]}},
                           deadline, true) &&
                    acquire(b, deadline))
                    return next(b);
            if (motion("MOVE",
                       {{"kind", "move"},
                        {"direction", "forward"},
                        {"distance", p["forward_distance"]},
                        {"speed", p["speed"]},
                        {"tolerance", std::min(.05, p["forward_distance"].get<double>() / 2)},
                        {"timeout", b["timeout"]}},
                       deadline, true) &&
                acquire(b, deadline))
                return next(b);
        }
        throw std::runtime_error("已达到本轮最大搜索次数，未发现目标");
    } else if (type == "recognize" || type == "lock" || type == "apriltag")
        return observe(b, deadline, type == "apriltag");
    else if (type == "align" || type == "center" || type == "follow" || type == "distance")
        return visual(b, deadline);
    else
        throw std::runtime_error("积木尚未接入");
    return next(b);
}
void Mission::cleanup() {
    {
        std::lock_guard<std::mutex> l(mutex);
        if (stop_mode == "kill")
            return; // Emergency stop owns shutdown; never send HOLD or LAND afterwards.
    }
    stop_vision();
    if (!flight || !claimed || !actuated)
        return;
    auto d = control.status()["data"], px = d["px4"], m = d["motion"];
    if (!px.value("connected", false))
        throw std::runtime_error("收尾时飞控离线");
    if (m.value("state", std::string()) == "landing" || px.value("mode", std::string()) == "LAND" ||
        px["armed"] != true)
        return;
    if (m.contains("id") && !sent.count(m["id"].get<std::string>()))
        throw std::runtime_error("收尾不覆盖外部任务");
    std::string mode;
    {
        std::lock_guard<std::mutex> l(mutex);
        mode = stop_mode;
    }
    if (mode == "land" ||
        m.value("params", Json::object()).value("kind", std::string()) == "auto_takeoff")
        command("LAND", Json::object(), false);
    else if (px.value("mode", std::string()) == "OFFBOARD")
        command("HOLD", Json::object(), false);
}
Json Mission::start(Json doc, const std::string &client) {
    doc = validate(std::move(doc));
    if (client.size() < 8 || client.size() > 100)
        throw std::runtime_error("客户端ID无效");
    if (active())
        throw std::runtime_error("已有比赛任务运行中");
    if (worker.joinable())
        worker.join();
    auto d = control.status()["data"];
    flight = false;
    const std::set<std::string> observes = {"recognize", "lock", "apriltag"};
    for (const auto &b : doc["blocks"])
        if (!observes.count(b["type"].get<std::string>()))
            flight = true;
    if (flight) {
        const auto &px = d["px4"];
        if (!px.value("connected", false) || !px.value("bidirectional", false) ||
            px["armed"] != true)
            throw std::runtime_error("比赛需要双向飞控连接且先人工解锁");
        if (d["joystick"].value("active", false) ||
            d["motion"].value("state", std::string()) == "running" ||
            d["motion"].value("state", std::string()) == "landing")
            throw std::runtime_error("先结束其他控制任务");
        for (const auto &b : doc["blocks"])
            if (b["id"] == doc["entry"]) {
                if (b["type"] == "takeoff" ? px.value("landed_state", 0) != 1
                                           : (px.value("mode", std::string()) != "OFFBOARD" ||
                                              !d["control"].value("zero_stream", false)))
                    throw std::runtime_error("任务入口飞行状态不满足");
            }
    }
    document = std::move(doc);
    nodes = Json::object();
    Json blocks = Json::object();
    for (const auto &b : document["blocks"]) {
        nodes[b["id"].get<std::string>()] = b;
        blocks[b["id"].get<std::string>()] = {{"status", "pending"}, {"visits", 0}};
    }
    begun = now_s();
    owner = client;
    renew_at = 0;
    landed = claimed = actuated = false;
    cancelled = false;
    sent.clear();
    vision_session.clear();
    last_search_id.clear();
    search_confirmation = Json();
    stop_mode = "hold";
    const auto dir = project_root() / "mission_runs";
    const auto log_path = log.open(dir);
    const auto filename = log_path.filename().string();
    const auto run_id = filename.substr(0, filename.find(".partial.jsonl"));
    log.append(Json{{"event", "configuration"}, {"document", document},
                    {"utc_iso", log_storage::utc()}, {"monotonic_s", begun},
                    {"boot_id", log_storage::boot_id()}, {"run_id", run_id},
                    {"runtime_session", log_storage::session()},
                    {"flight_log_file", d["trajectory"].value("filename", Json())}}.dump());
    {
        std::lock_guard<std::mutex> l(mutex);
        state = {{"state", "running"},
                 {"active", true},
                 {"run_id", run_id},
                 {"mission_id", document["id"]},
                 {"name", document["name"]},
                 {"document", document},
                 {"blocks", blocks},
                 {"events", Json::array()},
                 {"progress", 0.},
                 {"transitions", 0},
                 {"completed_visits", 0},
                 {"elapsed_s", 0.},
                 {"message", "C++比赛调度启动"},
                 {"log_file", filename}};
    }
    worker = std::thread(&Mission::run, this);
    return {{"ok", true},
            {"message", "比赛已由C++接管，关闭网页或Python不会中断执行"},
            {"mission", status()}};
}
Json Mission::stop(const std::string &mode, const std::string &source) {
    if (mode != "hold" && mode != "land" && mode != "kill")
        throw std::runtime_error("停止方式仅支持hold/land");
    {
        std::lock_guard<std::mutex> l(mutex);
        if (!state.value("active", false))
            return {{"ok", true}, {"mission", state}};
        if (mode == "kill" || (mode == "land" && stop_mode != "kill"))
            stop_mode = mode;
        cancelled = true;
        state["state"] = "stopping";
    }
    event("stop_requested", {{"mode", mode}, {"source", source}});
    wake.notify_all();
    return {{"ok", true}, {"message", "C++已接收停止请求，等待收尾反馈"}, {"mission", status()}};
}
void Mission::run() {
    name_thread("mission-run");
    std::string final = "succeeded", message = "任务完成，位置保持；降落需显式LAND积木";
    try {
        if (flight) {
            command("CLAIM");
            claimed = true;
            renew_at = now_s() + 4;
        }
        auto id = document["entry"].get<std::string>();
        while (!id.empty()) {
            current = nodes.at(id);
            const double deadline = now_s() + current["timeout"].get<double>();
            {
                std::lock_guard<std::mutex> l(mutex);
                state["current_id"] = id;
                state["current_name"] = current["name"];
                state["blocks"][id]["status"] = "running";
                state["blocks"][id]["visits"] = state["blocks"][id]["visits"].get<int>() + 1;
                state["transitions"] = state["transitions"].get<int>() + 1;
            }
            event("block_started", {{"block_id", id}, {"type", current["type"]}});
            check(deadline);
            const auto dest = execute(current, deadline);
            if (!vision_session.empty() && visual_successor(dest).is_null())
                stop_vision();
            const bool redirected = dest != next(current);
            {
                std::lock_guard<std::mutex> l(mutex);
                state["blocks"][id]["status"] = redirected ? "redirected" : "succeeded";
                state["completed_visits"] = state["completed_visits"].get<int>() + 1;
                int done = 0;
                for (const auto &b : state["blocks"])
                    if (b["status"] == "succeeded")
                        done++;
                state["progress"] = double(done) / nodes.size();
            }
            event(redirected ? "block_redirected" : "block_completed", {{"block_id", id}});
            id = dest;
        }
    } catch (const Cancelled &) {
        final = "stopped";
        message = "比赛调度已停止";
    } catch (const std::exception &e) {
        final = "failed";
        message = e.what();
    }
    try {
        cleanup();
    } catch (const std::exception &e) {
        message += "；收尾：" + std::string(e.what());
        if (final == "succeeded")
            final = "failed";
    }
    {
        std::lock_guard<std::mutex> l(mutex);
        if (state.contains("current_id")) {
            auto &b = state["blocks"][state["current_id"].get<std::string>()];
            if (b["status"] == "running" || b["status"] == "waiting")
                b["status"] = final == "stopped" ? "cancelled" : "failed";
        }
        state["state"] = final;
        state["message"] = message;
        state["elapsed_s"] = now_s() - begun;
    }
    event("run_finished", {{"state", final}, {"message", message}});
    log.close(); // Drain and fsync without holding the status mutex.
    {
        std::lock_guard<std::mutex> l(mutex);
        state["log_file"] = log.filename().string();
        state["log_error"] = log.error();
        state["active"] = false;
    }
}
