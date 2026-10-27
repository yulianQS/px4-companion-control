#include "core.hpp"
#define MAVLINK_USE_MESSAGE_INFO
#include "mavlink/common/mavlink.h"
#include "mavlink/mavlink_get_info.h"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

PX4::PX4(const std::string &device, int baud, int system, int component)
    : source_system(system), source_component(component) {
    if (baud != 115200)
        throw std::runtime_error("This project preserves UART 115200; unsupported baud");
    fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
        throw std::runtime_error("Cannot open UART: " + std::string(strerror(errno)));
    try {
        if (flock(fd, LOCK_EX | LOCK_NB) < 0 || ioctl(fd, TIOCEXCL) < 0)
            throw std::runtime_error("UART is occupied");
        termios t{};
        if (tcgetattr(fd, &t))
            throw std::runtime_error("tcgetattr failed");
        cfmakeraw(&t);
        cfsetispeed(&t, B115200);
        cfsetospeed(&t, B115200);
        t.c_cflag = (t.c_cflag & ~(CSIZE | PARENB | CSTOPB | CRTSCTS)) | CS8 | CLOCAL | CREAD;
        t.c_cc[VMIN] = 0;
        t.c_cc[VTIME] = 0;
        if (tcsetattr(fd, TCSANOW, &t))
            throw std::runtime_error("tcsetattr failed");
        state = {{"message", "UART opened; awaiting PX4 heartbeat"},
                 {"heartbeat_at", 0.},
                 {"bidirectional", false},
                 {"armed", nullptr},
                 {"mode", nullptr},
                 {"last_text", nullptr}};
        reader = std::thread(&PX4::read_loop, this);
    } catch (...) {
        close(fd);
        fd = -1;
        throw;
    }
}
PX4::~PX4() {
    stopping = true;
    changed.notify_all();
    if (reader.joinable())
        reader.join();
    if (fd >= 0) {
        ioctl(fd, TIOCNXCL);
        close(fd);
    }
}
void PX4::write_message(const void *ptr) {
    const auto &m = *static_cast<const mavlink_message_t *>(ptr);
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    auto len = mavlink_msg_to_send_buffer(bytes, &m);
    size_t done = 0;
    const double deadline = now_s() + .02;
    while (done < len) {
        ssize_t n = ::write(fd, bytes + done, len - done);
        if (n > 0)
            done += n;
        else if ((errno == EAGAIN || errno == EINTR) && now_s() < deadline) {
            pollfd p{fd, POLLOUT, 0};
            poll(&p, 1, 2);
        } else
            throw std::runtime_error("UART write failed or timed out");
    }
}
Json PX4::status() {
    std::lock_guard<std::mutex> l(mutex);
    Json s = state;
    double now = now_s();
    for (const char *k : {"heartbeat", "position", "attitude", "battery", "estimator", "range",
                          "flow", "rc", "landed", "odometry", "takeoff_alt", "takeoff_speed", "position_target"}) {
        std::string key = k;
        s[key + "_age_s"] = state.contains(key + "_at")
                                ? Json(now - state[key + "_at"].get<double>())
                                : Json(nullptr);
        s.erase(key + "_at");
    }
    s["connected"] = fresh(s, "heartbeat", 3);
    s["bidirectional"] = s["connected"].get<bool>() && now - parameter_at < 30 && parameter_at > 0;
    s["target"] = Json::array({target_system, target_component});
    s["counts"] = counts;
    return s;
}
void PX4::read_loop() {
    name_thread("mavlink-rx");
    mavlink_message_t message{};
    mavlink_status_t parser{};
    uint8_t bytes[2048];
    while (!stopping) {
        pollfd p{fd, POLLIN, 0};
        int result = poll(&p, 1, 50);
        if (result > 0 && p.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            std::lock_guard<std::mutex> l(mutex);
            state["heartbeat_at"] = 0.;
            state["message"] = "UART disconnected; restart service after recovery";
            break;
        }
        if (result > 0 && p.revents & POLLIN) {
            ssize_t n = read(fd, bytes, sizeof(bytes));
            for (ssize_t i = 0; i < n; i++)
                if (mavlink_parse_char(MAVLINK_COMM_1, bytes[i], &message, &parser))
                    receive(&message);
        }
        int ts, tc;
        {
            std::lock_guard<std::mutex> l(mutex);
            ts = target_system;
            tc = target_component;
        }
        if (ts && now_s() - probe_at >= 10) {
            try {
                std::lock_guard<std::mutex> l(tx_mutex);
                mavlink_message_t m;
                probe_at = now_s();
                // Periodic readback; writes are only issued by an explicit grounded TAKEOFF request.
                for (const char *name : {"MAV_SYS_ID", "MIS_TAKEOFF_ALT", "MPC_TKO_SPEED"}) {
                    char parameter[16]{};
                    std::strcpy(parameter, name);
                    mavlink_msg_param_request_read_pack(source_system, source_component, &m, ts, tc,
                                                        parameter, -1);
                    write_message(&m);
                }
            } catch (const std::exception &e) {
                std::lock_guard<std::mutex> l(mutex);
                state["message"] = e.what();
            }
        }
    }
}
void PX4::receive(const void *ptr) {
    const auto &m = *static_cast<const mavlink_message_t *>(ptr);
    double now = now_s();
    std::string kind;
    bool sample = false;
    {
        std::lock_guard<std::mutex> l(mutex);
        auto name = mavlink_get_message_info_by_id(m.msgid);
        kind = name ? name->name : std::to_string(m.msgid);
        counts[kind] = counts.value(kind, 0) + 1;
        if (m.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            mavlink_heartbeat_t h;
            mavlink_msg_heartbeat_decode(&m, &h);
            if (h.autopilot != 12 ||
                (target_system && (m.sysid != target_system || m.compid != target_component)))
                return;
            target_system = m.sysid;
            target_component = m.compid;
            int main = (h.custom_mode >> 16) & 255, submode = (h.custom_mode >> 24) & 255;
            std::string mode = main == 4 && submode == 2 ? "AUTO_TAKEOFF"
                               : main == 6                   ? "OFFBOARD"
                               : main == 3                 ? "POSCTL"
                               : main == 2                 ? "ALTCTL"
                               : main == 7                 ? "STABILIZED"
                               : main == 4 && submode == 6 ? "LAND"
                               : main == 4 && submode == 3
                                   ? "LOITER"
                                   : "PX4_" + std::to_string(main) + "_" + std::to_string(submode);
            state.update(
                {{"heartbeat_at", now},
                 {"heartbeat_revision", state.value("heartbeat_revision", 0) + 1},
                 {"armed", bool(h.base_mode & 128)},
                 {"base_mode", h.base_mode},
                 {"custom_mode", h.custom_mode},
                 {"mode", mode},
                 {"message", parameter_at > 0 ? "双向验证通过 · C++ MAVLink"
                                              : "PX4 heartbeat; awaiting parameter reply"}});
            sample = true;
        } else if (m.sysid != target_system || m.compid != target_component)
            return;
        else
            switch (m.msgid) {
            case MAVLINK_MSG_ID_PARAM_VALUE: {
                mavlink_param_value_t v;
                mavlink_msg_param_value_decode(&m, &v);
                const std::string parameter(v.param_id, strnlen(v.param_id, 16));
                if (std::isfinite(v.param_value) && v.param_value > 0 &&
                    v.param_type == MAV_PARAM_TYPE_REAL32) {
                    if (parameter == "MIS_TAKEOFF_ALT")
                        state.update({{"takeoff_alt_at", now}, {"takeoff_alt_m", v.param_value}});
                    else if (parameter == "MPC_TKO_SPEED")
                        state.update({{"takeoff_speed_at", now}, {"takeoff_speed_m_s", v.param_value}});
                }
                changed.notify_all();
                if (parameter == "MAV_SYS_ID" &&
                    now - probe_at < 5) {
                    parameter_at = now;
                    state["parameter_reply"] = v.param_value;
                }
                break;
            }
            case MAVLINK_MSG_ID_COMMAND_ACK: {
                mavlink_command_ack_t v;
                mavlink_msg_command_ack_decode(&m, &v);
                if (v.target_system == 0 || v.target_system == source_system) {
                    acks[v.command] = {now, v.result};
                    changed.notify_all();
                }
                break;
            }
            case MAVLINK_MSG_ID_STATUSTEXT: {
                mavlink_statustext_t v;
                mavlink_msg_statustext_decode(&m, &v);
                state["last_text"] = std::string(v.text, strnlen(v.text, 50));
                break;
            }
            case MAVLINK_MSG_ID_ATTITUDE: {
                mavlink_attitude_t v;
                mavlink_msg_attitude_decode(&m, &v);
                state.update(
                    {{"attitude_at", now},
                     {"attitude_deg", {v.roll * 180 / PI, v.pitch * 180 / PI, v.yaw * 180 / PI}},
                     {"angular_rate_rad_s",
                      {{"rollspeed", v.rollspeed},
                       {"pitchspeed", v.pitchspeed},
                       {"yawspeed", v.yawspeed}}}});
                break;
            }
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
                mavlink_local_position_ned_t v;
                mavlink_msg_local_position_ned_decode(&m, &v);
                state.update({{"position_at", now},
                              {"position_sample_ms", v.time_boot_ms},
                              {"position_revision", state.value("position_revision", 0) + 1},
                              {"local_position",
                               {{"x", v.x},
                                {"y", v.y},
                                {"z", v.z},
                                {"vx", v.vx},
                                {"vy", v.vy},
                                {"vz", v.vz}}}});
                sample = true;
                break;
            }
            case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: {
                mavlink_position_target_local_ned_t v;
                mavlink_msg_position_target_local_ned_decode(&m, &v);
                state.update({{"position_target_at", now},
                              {"position_target", {{"sample_ms", v.time_boot_ms},
                                                   {"frame", v.coordinate_frame},
                                                   {"type_mask", v.type_mask},
                                                   {"z", std::isfinite(v.z) ? Json(v.z) : Json(nullptr)},
                                                   {"vz", std::isfinite(v.vz) ? Json(v.vz) : Json(nullptr)}}}});
                break;
            }
            case MAVLINK_MSG_ID_ODOMETRY: {
                mavlink_odometry_t v;
                mavlink_msg_odometry_decode(&m, &v);
                state.update({{"odometry_at", now}, {"odometry_reset_counter", v.reset_counter}});
                break;
            }
            case MAVLINK_MSG_ID_ESTIMATOR_STATUS: {
                mavlink_estimator_status_t v;
                mavlink_msg_estimator_status_decode(&m, &v);
                state.update({{"estimator_at", now}, {"estimator_flags", v.flags}});
                break;
            }
            case MAVLINK_MSG_ID_SYS_STATUS: {
                mavlink_sys_status_t v;
                mavlink_msg_sys_status_decode(&m, &v);
                state.update(
                    {{"battery_at", now},
                     {"battery_v",
                      v.voltage_battery == 65535 ? Json(nullptr) : Json(v.voltage_battery / 1000.)},
                     {"battery_current_a",
                      v.current_battery == -1 ? Json(nullptr) : Json(v.current_battery / 100.)},
                     {"battery_remaining_pct",
                      v.battery_remaining == -1 ? Json(nullptr) : Json(v.battery_remaining)},
                     {"cpu_load_pct", v.load / 10.}});
                break;
            }
            case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
                mavlink_extended_sys_state_t v;
                mavlink_msg_extended_sys_state_decode(&m, &v);
                state.update({{"landed_at", now}, {"landed_state", v.landed_state}});
                break;
            }
            case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
                mavlink_distance_sensor_t v;
                mavlink_msg_distance_sensor_decode(&m, &v);
                if (v.orientation == 25)
                    state.update({{"range_at", now}, {"range_m", v.current_distance / 100.},
                                  {"range_valid", v.max_distance>v.min_distance && v.current_distance!=65535 &&
                                      v.current_distance>=v.min_distance && v.current_distance<=v.max_distance}});
                break;
            }
            case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD: {
                mavlink_optical_flow_rad_t v;
                mavlink_msg_optical_flow_rad_decode(&m, &v);
                state.update({{"flow_at", now}, {"flow_quality", v.quality}});
                break;
            }
            case MAVLINK_MSG_ID_RC_CHANNELS: {
                mavlink_rc_channels_t v;
                mavlink_msg_rc_channels_decode(&m, &v);
                state.update({{"rc_at", now},
                              {"rc_channels", v.chancount},
                              {"rc_rssi", v.rssi == 255 ? Json(nullptr) : Json(v.rssi)}});
                break;
            }
            case MAVLINK_MSG_ID_AUTOPILOT_VERSION: {
                mavlink_autopilot_version_t v;
                mavlink_msg_autopilot_version_decode(&m, &v);
                state["firmware_version_raw"] = v.flight_sw_version;
                break;
            }
            default:
                break;
            }
    }
    if (sample) {
        std::lock_guard<std::mutex> l(callback_mutex);
        if (on_sample)
            on_sample(kind, status());
    }
}
void PX4::send(const Setpoint &s) {
    int ts, tc;
    {
        std::lock_guard<std::mutex> l(mutex);
        ts = target_system;
        tc = target_component;
    }
    if (!ts)
        throw std::runtime_error("PX4 not connected");
    uint16_t mask = 64 | 128 | 256;
    Vec p = s.p, v = s.v;
    for (int i = 0; i < 3; i++) {
        if (!std::isfinite(p[i])) {
            mask |= 1 << i;
            p[i] = 0;
        }
        if (!std::isfinite(v[i])) {
            mask |= 8 << i;
            v[i] = 0;
        }
    }
    double h = s.heading, r = s.rate;
    if (!std::isfinite(h)) {
        mask |= 1024;
        h = 0;
    }
    if (!std::isfinite(r)) {
        mask |= 2048;
        r = 0;
    }
    std::lock_guard<std::mutex> l(tx_mutex);
    mavlink_message_t m;
    mavlink_msg_set_position_target_local_ned_pack(source_system, source_component, &m,
                                                   uint32_t(now_s() * 1000), ts, tc, 1, mask, p[0],
                                                   p[1], p[2], v[0], v[1], v[2], 0, 0, 0, h, r);
    write_message(&m);
}
int PX4::command(int cmd, const std::array<float, 7> &p, double timeout) {
    std::lock_guard<std::mutex> one(command_mutex);
    double sent = now_s();
    int ts, tc;
    {
        std::lock_guard<std::mutex> l(mutex);
        ts = target_system;
        tc = target_component;
        acks.erase(cmd);
    }
    if (!ts)
        throw std::runtime_error("PX4 not connected");
    {
        std::lock_guard<std::mutex> l(tx_mutex);
        mavlink_message_t m;
        mavlink_msg_command_long_pack(source_system, source_component, &m, ts, tc, cmd, 0, p[0],
                                      p[1], p[2], p[3], p[4], p[5], p[6]);
        write_message(&m);
    }
    std::unique_lock<std::mutex> l(mutex);
    double end = sent + timeout;
    while (!stopping && now_s() < end) {
        auto it = acks.find(cmd);
        if (it != acks.end() && it->second.first >= sent && it->second.second != 5)
            return it->second.second;
        changed.wait_for(l, std::chrono::milliseconds(50));
    }
    throw std::runtime_error("COMMAND_ACK timeout; result unknown, command not retried");
}

// Only the explicit takeoff request may change this REAL32 parameter. A fresh
// matching PARAM_VALUE is required; stale cached values cannot confirm the write.
void PX4::set_takeoff_height(float height, double timeout) {
    if (!std::isfinite(height) || height < .1f || height > 5.f)
        throw std::runtime_error("Takeoff height must be between 0.1 and 5 metres");
    std::lock_guard<std::mutex> one(command_mutex);
    int ts, tc;
    {
        std::lock_guard<std::mutex> l(mutex);
        ts = target_system; tc = target_component;
    }
    if (!ts)
        throw std::runtime_error("PX4 not connected");
    const double sent = now_s();
    {
        std::lock_guard<std::mutex> l(tx_mutex);
        mavlink_message_t m;
        mavlink_msg_param_set_pack(source_system, source_component, &m, ts, tc,
                                  "MIS_TAKEOFF_ALT", height, MAV_PARAM_TYPE_REAL32);
        write_message(&m);
    }
    std::unique_lock<std::mutex> l(mutex);
    while (!stopping && now_s() - sent < timeout) {
        if (state.value("takeoff_alt_at", 0.) >= sent &&
            std::abs(state.value("takeoff_alt_m", -1.) - height) < .001)
            return;
        changed.wait_for(l, std::chrono::milliseconds(50));
    }
    throw std::runtime_error("MIS_TAKEOFF_ALT write/readback unconfirmed; takeoff not sent, parameter may have changed");
}
