#include "core.hpp"
#include "log_storage.hpp"
#include <arpa/inet.h>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

static std::atomic<bool> quit{false};
static void signal_handler(int) { quit = true; }
constexpr uint32_t MAX_FRAME = 1048576;
struct Peer {
    int fd;
    std::mutex mutex;
    std::deque<Json> outgoing;
    std::atomic<bool> closed{false};
    explicit Peer(int socket) : fd(socket) {}
    void push(Json j) {
        std::lock_guard<std::mutex> l(mutex);
        if (outgoing.size() >= 256) {
            closed = true;
            return;
        }
        outgoing.push_back(std::move(j));
    }
};
struct Request {
    std::shared_ptr<Peer> peer;
    Json body;
};
static bool send_frame(int fd, const Json &j) {
    std::string data = j.dump();
    uint32_t length = htonl(data.size());
    std::string frame(reinterpret_cast<char *>(&length), 4);
    frame += data;
    size_t done = 0;
    while (done < frame.size()) {
        ssize_t n = send(fd, frame.data() + done, frame.size() - done, MSG_NOSIGNAL);
        if (n <= 0)
            return false;
        done += n;
    }
    return true;
}
static void peer_loop(std::shared_ptr<Peer> peer, Controller &control, std::mutex &qmutex,
                      std::deque<Request> &queue, std::condition_variable &wake) {
    name_thread("tcp-client");
    timeval timeout{0, 100000};
    setsockopt(peer->fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    std::string buffer;
    double last_status = 0;
    while (!quit && !peer->closed) {
        pollfd p{peer->fd, POLLIN, 0};
        int result = poll(&p, 1, 20);
        if (result < 0 || (p.revents & (POLLHUP | POLLERR | POLLNVAL)))
            break;
        if (result > 0 && (p.revents & POLLIN)) {
            char block[8192];
            ssize_t n = recv(peer->fd, block, sizeof(block), 0);
            if (n <= 0)
                break;
            buffer.append(block, n);
        }
        bool invalid = false;
        while (buffer.size() >= 4) {
            uint32_t bytes;
            std::memcpy(&bytes, buffer.data(), 4);
            bytes = ntohl(bytes);
            if (bytes == 0 || bytes > MAX_FRAME) {
                invalid = true;
                break;
            }
            if (buffer.size() < bytes + 4)
                break;
            try {
                Json body = Json::parse(buffer.substr(4, bytes));
                if (!body.is_object() || body.value("type", std::string()) != "command" ||
                    !body.contains("id") || !body["id"].is_string() ||
                    body["id"].get<std::string>().size() > 100 ||
                    body["id"].get<std::string>().empty() || !body.contains("command") ||
                    !body["command"].is_string() ||
                    !body.value("params", Json::object()).is_object())
                    throw std::runtime_error("Invalid command envelope");
                if (body["command"] == "GET_STATUS" || body["command"] == "MISSION_STATUS") {
                    Json status = body["command"] == "GET_STATUS"
                                      ? control.status()
                                      : control.execute("MISSION_STATUS", Json::object(), "", "");
                    status["type"] = "response";
                    status["id"] = body["id"];
                    status["stage"] = "completed";
                    status["ok"] = true;
                    peer->push(status);
                } else if (body["command"] == "STICK_INPUT" ||
                           body["command"] == "STOP_MOTOR_TEST") {
                    Json reply;
                    try {
                        reply =
                            control.execute(body["command"], body.value("params", Json::object()),
                                            body.value("client", std::string()), body["id"]);
                    } catch (const std::exception &e) {
                        reply = {{"ok", false}, {"stage", "failed"}, {"message", e.what()}};
                    }
                    reply["id"] = body["id"];
                    reply["type"] = "response";
                    if (!reply.contains("stage"))
                        reply["stage"] = "completed";
                    peer->push(reply);
                } else {
                    std::lock_guard<std::mutex> l(qmutex);
                    if (queue.size() >= 128)
                        throw std::runtime_error("Command queue full");
                    queue.push_back({peer, body});
                    peer->push({{"id", body["id"]},
                                {"type", "response"},
                                {"stage", "received"},
                                {"accepted", false},
                                {"message", "Received, validation pending"}});
                    wake.notify_one();
                }
            } catch (const std::exception &e) {
                peer->push({{"type", "response"},
                            {"stage", "failed"},
                            {"ok", false},
                            {"message", e.what()}});
            }
            buffer.erase(0, bytes + 4);
        }
        if (invalid)
            break;
        std::deque<Json> out;
        {
            std::lock_guard<std::mutex> l(peer->mutex);
            out.swap(peer->outgoing);
        }
        for (auto &j : out)
            if (!send_frame(peer->fd, j)) {
                peer->closed = true;
                break;
            }
        if (now_s() - last_status >= .2) {
            auto status = control.status();
            status["type"] = "status";
            if (!send_frame(peer->fd, status))
                break;
            last_status = now_s();
        }
    }
    peer->closed = true;
    shutdown(peer->fd, SHUT_RDWR);
    close(peer->fd);
}
static int self_test() {
    auto r = reference({0, 0, 0}, {1, 0, 0}, .2, .3, 100);
    if (std::abs(r.first[0] - 1) > 1e-9 || norm(r.second) > 1e-9)
        return 1;
    r = reference({0, 0, 0}, {1, 0, 0}, .2, .3, .1);
    if (std::abs(r.first[0] - .0015) > 1e-9 || std::abs(r.second[0] - .03) > 1e-9)
        return 2;
    r = reference({0, 0, 0}, {0, 0, -.5}, .2, .3, 1);
    if (r.first[2] >= 0 || r.second[2] >= 0)
        return 3;
    Setpoint s;
    s.p = {1, 2, -3};
    auto j = Json::parse(s.json().dump());
    if (j["yaw"] != nullptr || j["position"][2] != -3)
        return 4;
    std::cout << "C++ native reference/coordinate/JSON self-test passed\n";
    return 0;
}
int main(int argc, char **argv) {
    std::string device = "/dev/ttyS7", host = "127.0.0.1", logs = "flight_records";
    int port = 8090, baud = 115200, system = 42, component = 191;
    try {
        std::ifstream runtime(project_root() / "config/runtime.json");
        if (runtime) {
            const auto settings = Json::parse(runtime), c = settings.at("control");
            device = c.value("device", device);
            host = c.value("host", host);
            port = c.value("port", port);
            baud = c.value("baud", baud);
            system = c.value("source_system", system);
            component = c.value("source_component", component);
            if (!std::getenv("DRONE_VISION_SNAPSHOT_DIR"))
                setenv("DRONE_VISION_SNAPSHOT_DIR",
                       settings.at("vision_snapshot_dir").get<std::string>().c_str(), 0);
        }
        for (int i = 1; i < argc; i++) {
            std::string key = argv[i];
            if (key == "--self-test")
                return self_test();
            if (key == "--help") {
                std::cout << "drone_service [--device /dev/ttyS7] [--baud 115200] [--host "
                             "127.0.0.1] [--port 8090] [--logs flight_records] [--source-system "
                             "42] [--source-component 191]\nStartup is monitor-only; no mode/arm "
                             "command is sent.\n";
                return 0;
            }
            if (i + 1 >= argc)
                throw std::runtime_error("Missing option value");
            std::string value = argv[++i];
            if (key == "--device")
                device = value;
            else if (key == "--host")
                host = value;
            else if (key == "--port")
                port = std::stoi(value);
            else if (key == "--logs")
                logs = value;
            else if (key == "--baud")
                baud = std::stoi(value);
            else if (key == "--source-system")
                system = std::stoi(value);
            else if (key == "--source-component")
                component = std::stoi(value);
            else
                throw std::runtime_error("Unknown option " + key);
        }
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGPIPE, SIG_IGN);
        int server = socket(AF_INET, SOCK_STREAM, 0);
        if (server < 0)
            throw std::runtime_error("socket failed");
        int yes = 1;
        setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        if (port < 1 || port > 65535 || system < 1 || system > 255 || component < 1 ||
            component > 255 || inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1 ||
            bind(server, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0 ||
            listen(server, 8) < 0)
            throw std::runtime_error("Cannot bind TCP endpoint");
        log_storage::recover(project_root() / "mission_runs", ".partial.jsonl");
        FlightLog logger(logs);
        PX4 px4(device, baud, system, component);
        Controller control(px4, logger);
        std::mutex qmutex;
        std::condition_variable wake;
        std::deque<Request> queue;
        std::thread worker([&] {
            name_thread("tcp-command");
            struct Cached {
                Json body, result;
            };
            std::map<std::string, Cached> cache;
            std::deque<std::string> order;
            while (!quit) {
                Request req;
                {
                    std::unique_lock<std::mutex> l(qmutex);
                    wake.wait_for(l, std::chrono::milliseconds(100),
                                  [&] { return quit || !queue.empty(); });
                    if (!queue.empty()) {
                        req = std::move(queue.front());
                        queue.pop_front();
                    }
                }
                if (quit)
                    break;
                if (req.body.is_null()) {
                    control.maintain();
                    continue;
                }
                auto &b = req.body;
                std::string id = b["id"], client = b.value("client", std::string());
                std::string key = client + ":" + id;
                Json result;
                auto prior = cache.find(key);
                if (prior != cache.end()) {
                    result = prior->second.body == b
                                 ? prior->second.result
                                 : Json{{"ok", false},
                                        {"stage", "failed"},
                                        {"message", "ID reused with different command"}};
                } else {
                    if (req.peer->closed)
                        continue; // Never start a queued command after its submitter disconnected.
                    try {
                        result = control.execute(b["command"], b.value("params", Json::object()),
                                                 client, id);
                        if (!result.contains("stage"))
                            result["stage"] =
                                result.value("confirmed", false) ? "completed" : "accepted";
                        result["accepted"] = result.value("ok", true);
                    } catch (const std::exception &e) {
                        result = {{"ok", false},
                                  {"accepted", false},
                                  {"stage", "failed"},
                                  {"message", e.what()}};
                    }
                    if (b["command"] != "STICK_INPUT" && b["command"] != "RENEW") {
                        cache[key] = {b, result};
                        order.push_back(key);
                        if (order.size() > 512) {
                            cache.erase(order.front());
                            order.pop_front();
                        }
                    }
                }
                result["id"] = id;
                result["type"] = "response";
                req.peer->push(result);
            }
        });
        std::vector<std::pair<std::shared_ptr<Peer>, std::thread>> peers;
        std::cout << "C++ MAVLink service " << host << ':' << port << "; UART " << device << " @"
                  << baud << "; monitor only until explicit command\n"
                  << std::flush;
        while (!quit) {
            pollfd p{server, POLLIN, 0};
            if (poll(&p, 1, 100) > 0) {
                int fd = accept(server, nullptr, nullptr);
                if (fd >= 0) {
                    if (peers.size() >= 8) {
                        close(fd);
                    } else {
                        auto peer = std::make_shared<Peer>(fd);
                        peers.emplace_back(peer, std::thread(peer_loop, peer, std::ref(control),
                                                             std::ref(qmutex), std::ref(queue),
                                                             std::ref(wake)));
                    }
                }
            }
            for (auto it = peers.begin(); it != peers.end();) {
                if (it->first->closed) {
                    it->second.join();
                    it = peers.erase(it);
                } else
                    ++it;
            }
        }
        close(server);
        control.request_stop();
        wake.notify_all();
        for (auto &item : peers) {
            shutdown(item.first->fd, SHUT_RDWR);
            item.second.join();
        }
        worker.join();
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "drone_service: " << e.what() << '\n';
        return 1;
    }
}
