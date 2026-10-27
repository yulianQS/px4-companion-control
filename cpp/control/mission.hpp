#pragma once
#include "core.hpp"
#include "log_storage.hpp"
#include <condition_variable>
#include <fstream>
#include <set>

// High-level scheduling uses the existing controller; it never writes MAVLink
// or generates setpoints. Its worker cannot block the 20 Hz flight thread.
class Mission {
    Controller &control;
    std::mutex mutex;
    std::condition_variable wake;
    std::thread worker;
    std::atomic<bool> cancelled{false};
    Json state = {{"state", "idle"},
                  {"active", false},
                  {"blocks", Json::object()},
                  {"events", Json::array()},
                  {"message", "C++比赛调度未运行"}};
    Json document, nodes, current, search_confirmation;
    std::string owner, stop_mode = "hold", vision_session, last_search_id;
    double begun = 0, renew_at = 0;
    bool flight = false, landed = false, claimed = false, actuated = false;
    std::set<std::string> sent;
    log_storage::JsonLog log;
    Json command(const std::string &, Json = Json::object(), bool check_cancel = true);
    Json check(double);
    Json target(bool pose = false);
    void track_target(const Json &, double missing, const std::string &message = "");
    void event(const std::string &, Json = Json::object());
    void pause(double = .05);
    bool motion(const std::string &, Json, double, bool search = false);
    bool acquire(const Json &, double);
    std::string execute(const Json &, double);
    std::string visual(const Json &, double);
    std::string observe(const Json &, double, bool pose = false);
    std::string lost(const Json &);
    Json visual_successor(std::string);
    void start_vision(const Json &, bool distance);
    void stop_vision();
    void cleanup();
    void run();

  public:
    explicit Mission(Controller &c) : control(c) {}
    ~Mission();
    bool active();
    Json status(bool full = true);
    Json start(Json, const std::string &);
    Json stop(const std::string &, const std::string &source = "user_request");
};
