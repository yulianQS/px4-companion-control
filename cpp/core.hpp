#pragma once
#include "json.hpp"
#include "vision_pid.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <sys/prctl.h>
#include <vector>
using Json = nlohmann::json;
using Vec = std::array<double, 3>;
inline void name_thread(const char *name) { prctl(PR_SET_NAME, name, 0, 0, 0); }
inline std::filesystem::path project_root() {
    const char *root = std::getenv("DRONE_PROJECT_ROOT");
    return root ? std::filesystem::path(root)
                : std::filesystem::read_symlink("/proc/self/exe").parent_path().parent_path();
}
inline double now_s() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
constexpr double PI = 3.14159265358979323846;
constexpr double NAN_VALUE = std::numeric_limits<double>::quiet_NaN();
inline double wrap(double x) { return std::remainder(x, 2 * PI); }
inline double norm(Vec v) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }
inline Vec sub(Vec a, Vec b) {
    for (int i = 0; i < 3; i++)
        a[i] -= b[i];
    return a;
}
inline Vec add(Vec a, Vec b) {
    for (int i = 0; i < 3; i++)
        a[i] += b[i];
    return a;
}
inline double number(const Json &j, const char *k, double d) {
    if (!j.contains(k))
        return d;
    if (!j[k].is_number() || !std::isfinite(j[k].get<double>()))
        throw std::runtime_error(std::string(k) + " must be finite numeric");
    return j[k].get<double>();
}
inline double bounded(const Json &j, const char *k, double d, double lo, double hi) {
    double x = number(j, k, d);
    if (x < lo || x > hi)
        throw std::runtime_error(std::string(k) + " outside allowed range");
    return x;
}
inline double positive(const Json &j, const char *k, double d) {
    const double value = number(j, k, d);
    if (value <= 0) throw std::runtime_error(std::string(k) + " must be positive");
    return value;
}
inline bool fresh(const Json &s, const std::string &k, double max_age) {
    return s.contains(k + "_age_s") && s[k + "_age_s"].is_number() &&
           s[k + "_age_s"].get<double>() < max_age;
}
inline Vec position(const Json &s) {
    const auto &p = s.at("local_position");
    return {p.at("x"), p.at("y"), p.at("z")};
}
inline Vec velocity(const Json &s) {
    const auto &p = s.at("local_position");
    return {p.at("vx"), p.at("vy"), p.at("vz")};
}
inline double yaw(const Json &s) { return s.at("attitude_deg").at(2).get<double>() * PI / 180; }
inline double target_heading(const Json &v, const Json &px) {
    if (v.contains("search_bearing_deg"))
        return v["search_bearing_deg"];
    double heading = yaw(px);
    // Existing front-facing camera: optical right/forward -> PX4 heading.
    if (v.value("calibrated", false) && v.value("pose_valid", false) &&
        v.contains("translation_m") && v["translation_m"].is_array() &&
        v["translation_m"].size() == 3 && v["translation_m"][0].is_number() &&
        v["translation_m"][2].is_number()) {
        const double x = v["translation_m"][0], z = v["translation_m"][2];
        if (std::isfinite(x) && std::isfinite(z) && z > 0)
            heading += std::atan2(x, z);
    }
    return wrap(heading) * 180 / PI;
}
struct Setpoint {
    Vec p{NAN_VALUE, NAN_VALUE, NAN_VALUE}, v{0, 0, 0};
    double heading = NAN_VALUE, rate = 0;
    Json json() const {
        return {{"position", p}, {"velocity", v}, {"yaw", heading}, {"yaw_rate", rate}};
    }
};
// Analytic trapezoid/triangle retained from native_position.py. PX4 closes the position loop.
std::pair<Vec, Vec> reference(Vec start, Vec goal, double speed, double acc, double elapsed);

class PX4 {
    int fd = -1;
    std::mutex mutex, tx_mutex, command_mutex, callback_mutex;
    std::condition_variable changed;
    std::thread reader;
    std::atomic<bool> stopping{false};
    Json state;
    Json counts = Json::object();
    std::map<int, std::pair<double, int>> acks;
    uint32_t target_system = 0;
    uint8_t target_component = 0;
    double probe_at = 0, parameter_at = 0;
    void read_loop();
    void receive(const void *message);
    void write_message(const void *message);

  public:
    std::function<void(const std::string &, const Json &)> on_sample;
    void set_sample_handler(std::function<void(const std::string &, const Json &)> handler) {
        std::lock_guard<std::mutex> l(callback_mutex);
        on_sample = std::move(handler);
    }
    PX4(const std::string &device, int baud, int system = 42, int component = 191);
    ~PX4();
    int source_system, source_component;
    Json status();
    void send(const Setpoint &);
    void set_takeoff_height(float height, double timeout = 3);
    int command(int command, const std::array<float, 7> &params, double timeout = 3);
};

class FlightLog {
    std::string directory;
    std::mutex mutex;
    std::condition_variable wake;
    std::deque<Json> queue;
    std::thread writer;
    bool stopping = false;
    Json info;
    void run();

  public:
    explicit FlightLog(std::string path);
    ~FlightLog();
    void submit(const std::string &kind, const Json &state, const Setpoint &target);
    Json status();
};

class Mission;
class Controller {
    std::unique_ptr<Mission> mission;
    PX4 &px4;
    FlightLog &log;
    std::mutex mutex;
    std::thread ticker;
    std::atomic<bool> stopping{false};
    std::atomic<bool> emergency_stopped{false};
    // 20Hz setpoint output and control ownership.
    bool streaming = false;
    std::string stream_error;
    double sent_at = 0, stream_started = 0;
    std::string owner;
    double lease_until = 0;
    Setpoint target;
    bool holding = false;
    int reset = -1;
    uint32_t sample_ms = 0;
    // Motion task and trajectory feedback state (control/trajectory.cpp, update.cpp).
    Json task = {{"state", "idle"}}, last_action = Json::object(), events = Json::array();
    std::string task_command_id;
    Vec start{}, goal{};
    double start_yaw = 0, goal_yaw = 0, travel_yaw = 0, last_yaw = 0;
    double begun = 0, settled_at = 0, last_tick = 0;
    uint64_t leveling_attitude_revision = 0;
    double takeoff_ground_z = NAN_VALUE;
    int settled_samples = 0;
    uint64_t settled_revision = 0;
    std::vector<Vec> waypoints;
    size_t waypoint = 0;
    double leg_begun = 0;
    double max_speed = .2, max_vertical = .2, yaw_speed = PI / 6, tolerance = .05, timeout = 30;
    // Joystick session and axis state (control/joystick.cpp, update.cpp).
    bool sticks = false;
    std::string stick_session;
    int64_t sequence = -1;
    double stick_at = 0;
    Json axes = {{"throttle", 0.}, {"yaw", 0.}, {"pitch", 0.}, {"roll", 0.}};
    std::array<double, 4> stick_command{};
    Vec stick_hold{NAN_VALUE, NAN_VALUE, NAN_VALUE};
    double stick_yaw = NAN_VALUE;
    int stick_epoch = -1;
    // Timed velocity and LAND/disable completion state.
    bool velocity_active = false;
    double velocity_until = 0;
    Vec requested_velocity{};
    std::string velocity_frame = "LOCAL_NED";
    bool close_after_land = false;
    std::atomic<bool> motor_cancel{false};
    // Vision snapshots are read off the 20Hz flight thread; only validated cached pixels enter
    // control.
    std::mutex vision_mutex;
    std::thread vision_reader;
    Json vision_sample = Json::object();
    Json vision_tracking_config = Json::object();
    double vision_vz = 0, vision_rate = 0, vision_z = NAN_VALUE, vision_heading = NAN_VALUE;
    Vec vision_origin{};
    Vec vision_last_forward_velocity{};
    double vision_loss_started = 0, vision_last_forward = 0;
    double vision_search_seen = 0, vision_search_bearing = 0;
    VisionPID vision_yaw_pid, vision_vertical_pid, vision_distance_pid, vision_lateral_pid;
    bool vision_xy_moving = false;
    void read_vision();
    Json vision_measurement(double, double);
    Json perform_vision(const std::string &, const Json &, const std::string &);
    void check_vision_handover(const Json &);
    void update_vision(const Json &, double, double);
    void tick();
    void update(const Json &, double);
    void capture(const Json &);
    void finish(const std::string &, const std::string &, const Json &,
                bool capture_position = true);
    void ready(const Json &, bool need_position = true);
    Json confirmed(int, const std::array<float, 7> &, const std::function<bool(const Json &)> &);
    void event(const std::string &, const std::string &, const std::string &);
    void start_task(const Json &, const std::string &, const Json &);
    bool update_move_leveling(const Json &, double);
    Json joystick_status(double);
    // Command worker helpers; update()/tick() stay on the 20Hz control thread.
    Json perform(const std::string &, const Json &, const std::string &);
    Json perform_takeoff(const Json &, const std::string &);
    void update_takeoff(const Json &, double);
    Json perform_safety(const std::string &, const Json &, const std::string &);
    Json perform_joystick(const std::string &, const Json &, const std::string &);

  public:
    Controller(PX4 &, FlightLog &);
    ~Controller();
    Json execute(const std::string &command, const Json &params, const std::string &client,
                 const std::string &id, bool internal = false);
    Json vision_snapshot();
    Json vision_defaults();
    Json status();
    void maintain();
    void request_stop() {
        motor_cancel = true;
        stopping = true;
    }
};
