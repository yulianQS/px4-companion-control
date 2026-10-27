#include "log_storage.hpp"
#include <cassert>
#include <iostream>

int main(int argc, char **argv) {
    assert(argc == 2);
    namespace fs = std::filesystem;
    const fs::path directory(argv[1]);
    log_storage::JsonLog log;
    const auto path = log.open(directory);
    assert(path.filename().string().find(log_storage::utc().substr(0, 9)) != std::string::npos);
    for (int i = 0; i < 100; ++i) log.append("{\"event\":\"sample\"}");
    std::this_thread::sleep_for(std::chrono::milliseconds(1250));
    assert(fs::file_size(path) > 0); // Checkpoint before orderly completion.
    log.close();
    assert(log.error().empty());
    const auto complete = directory / log.filename();
    assert(complete.extension() == ".jsonl" && !fs::exists(path));
    std::ifstream input(complete);
    std::string line;
    int lines = 0;
    while (std::getline(input, line)) ++lines;
    assert(lines == 100);
    const auto second = log.open(directory);
    log.close();
    assert(second != path);
    const auto stale = directory / "mission_20200101T000000.000Z_b00000000_p999999_n0.partial.jsonl";
    { std::ofstream file(stale); file << "{\"event\":\"before_crash\"}\n"; }
    const auto live = log_storage::reserve(directory, "mission_", ".partial.jsonl");
    log_storage::recover(directory, ".partial.jsonl");
    assert(!fs::exists(stale));
    assert(fs::exists(directory / "mission_20200101T000000.000Z_b00000000_p999999_n0.interrupted.jsonl"));
    assert(fs::exists(live)); // Another live writer must never be renamed.
    std::cout << "PASS: timestamp identity, periodic checkpoint, complete archive and interrupted/live recovery\n";
}
