#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <system_error>
#include <thread>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

namespace log_storage {
namespace fs = std::filesystem;
inline std::string utc() {
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const std::time_t seconds = ms / 1000;
    std::tm value{};
    gmtime_r(&seconds, &value);
    std::ostringstream out;
    out << std::put_time(&value, "%Y%m%dT%H%M%S") << '.' << std::setw(3)
        << std::setfill('0') << ms % 1000 << 'Z';
    return out.str();
}
inline const std::string &boot_id() {
    static const std::string value = [] {
        std::ifstream file("/proc/sys/kernel/random/boot_id");
        std::string id;
        file >> id;
        return id.empty() ? std::string("unknown") : id;
    }();
    return value;
}
inline std::string session() {
    const char *value = std::getenv("DRONE_LOG_SESSION");
    return value ? value : "";
}
inline void sync_path(const fs::path &path, bool directory = false) {
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC | (directory ? O_DIRECTORY : 0));
    if (fd < 0)
        throw std::system_error(errno, std::generic_category(), "open log for sync");
    const int result = ::fsync(fd), error = errno;
    ::close(fd);
    if (result < 0)
        throw std::system_error(error, std::generic_category(), "fsync log");
}
inline void checkpoint(std::ofstream &file, const fs::path &path) {
    file.flush();
    if (!file)
        throw std::runtime_error("log write/flush failed: " + path.string());
    sync_path(path);
}
inline fs::path reserve(const fs::path &directory, const std::string &prefix,
                        const std::string &suffix) {
    static std::atomic<unsigned> sequence{0};
    fs::create_directories(directory);
    sync_path(fs::absolute(directory).parent_path(), true);
    for (;;) {
        const auto path = directory / (prefix + utc() + "_b" + boot_id().substr(0, 8) +
            "_p" + std::to_string(getpid()) + "_n" + std::to_string(sequence++) + suffix);
        const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
        if (fd >= 0) {
            ::close(fd);
            sync_path(directory, true);
            return path;
        }
        if (errno != EEXIST)
            throw std::system_error(errno, std::generic_category(), "create log");
    }
}
inline fs::path finish(const fs::path &partial, const std::string &ending) {
    std::string name = partial.filename().string();
    const auto position = name.rfind(".partial.");
    name.replace(position, 9, ending); // ending="." or ".interrupted."
    const auto result = partial.parent_path() / name;
    // Never replace an earlier record, even if a clock or PID is reused.
    if (fs::exists(result))
        throw std::runtime_error("log destination already exists: " + result.string());
    fs::rename(partial, result);
    sync_path(result.parent_path(), true);
    return result;
}
inline void recover(const fs::path &directory, const std::string &extension) {
    fs::create_directories(directory);
    for (const auto &entry : fs::directory_iterator(directory)) {
        const auto name = entry.path().filename().string();
        if (name.size() < extension.size() ||
            name.compare(name.size() - extension.size(), extension.size(), extension) != 0)
            continue;
        const auto boot = name.rfind("_b"), pid = name.rfind("_p");
        if (boot == std::string::npos || pid == std::string::npos)
            continue; // Historical files have no trustworthy writer identity.
        char *end = nullptr;
        const auto writer = static_cast<pid_t>(std::strtol(name.c_str() + pid + 2, &end, 10));
        if (writer <= 0 || !end || *end != '_') continue;
        if (name.substr(boot + 2, 8) != boot_id().substr(0, 8) ||
            (::kill(writer, 0) < 0 && errno == ESRCH)) {
            sync_path(entry.path());
            finish(entry.path(), ".interrupted.");
        }
    }
}

// Mission events never wait for storage on the control/status/emergency path.
class JsonLog {
    std::mutex mutex;
    std::condition_variable wake;
    std::deque<std::string> queue;
    std::thread writer;
    std::ofstream file;
    fs::path path;
    bool stopping = false;
    std::string failure;
    void run() {
        auto synced = std::chrono::steady_clock::now();
        try {
            for (;;) {
                std::string line;
                bool done;
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    wake.wait_until(lock, synced + std::chrono::seconds(1),
                        [&] { return stopping || !queue.empty(); });
                    done = stopping && queue.empty();
                    if (!queue.empty()) {
                        line = std::move(queue.front());
                        queue.pop_front();
                    }
                }
                if (!line.empty()) file << line << '\n';
                if (done || std::chrono::steady_clock::now() - synced >= std::chrono::seconds(1)) {
                    checkpoint(file, path);
                    synced = std::chrono::steady_clock::now();
                }
                if (done) break;
            }
            file.close();
            path = finish(path, error().empty() ? "." : ".interrupted.");
        } catch (const std::exception &error) {
            file.close();
            std::lock_guard<std::mutex> lock(mutex);
            failure = error.what();
        }
    }
  public:
    ~JsonLog() { close(); }
    fs::path open(const fs::path &directory) {
        close();
        recover(directory, ".partial.jsonl");
        path = reserve(directory, "mission_", ".partial.jsonl");
        file.open(path);
        if (!file) throw std::runtime_error("cannot open mission log");
        {
            std::lock_guard<std::mutex> lock(mutex);
            stopping = false;
            failure.clear();
            queue.clear();
        }
        writer = std::thread(&JsonLog::run, this);
        return path;
    }
    void append(std::string line) {
        std::lock_guard<std::mutex> lock(mutex);
        if (queue.size() >= 4096) failure = "mission log queue overflow";
        if (!stopping && failure.empty()) queue.push_back(std::move(line));
        wake.notify_one();
    }
    void close() {
        { std::lock_guard<std::mutex> lock(mutex); stopping = true; }
        wake.notify_one();
        if (writer.joinable()) writer.join();
    }
    std::string error() { std::lock_guard<std::mutex> lock(mutex); return failure; }
    fs::path filename() const { return path.filename(); } // After close only.
};
} // namespace log_storage
