#include "vision_track.hpp"
#include <cassert>
#include <iostream>
using Json = nlohmann::json;
int main() {
    VisionTrack tracker;
    auto frame = [](double t, bool visible = true, double x = 320) {
        return Json{{"source", "ros2_cpp"},
                    {"written_monotonic", t},
                    {"observation_monotonic", t},
                    {"camera_ready", true},
                    {"target_found", visible},
                    {"tag_id", 0},
                    {"center", {x, 240}},
                    {"width", 640},
                    {"height", 480}};
    };
    auto v = tracker.update(frame(10), 10.001);
    assert(v["target_observed"] == true && v["confirmation_seconds"] == 0);
    v = tracker.update(frame(10.1), 10.101);
    const double evidence = v["confirmation_seconds"];
    auto repeated = frame(10.1);
    repeated["written_monotonic"] = 10.2;
    v = tracker.update(repeated, 10.201);
    assert(v["confirmation_seconds"] == evidence);
    v = tracker.update(frame(10.22, false), 10.221);
    assert(v["target_observed"] == false && v["target_retained"] == true);
    assert(v["confirmation_seconds"] == evidence && v["center"][0] == 320);
    v = tracker.update(frame(10.3), 10.301);
    assert(v["target_observed"] == true && v["confirmation_seconds"] == evidence);
    v = tracker.update(frame(10.4), 10.401);
    assert(v["confirmation_seconds"].get<double>() > evidence);
    v = tracker.update(frame(10.45, true, 600), 10.451);
    assert(v["target_retained"] == true && v["center"][0] == 320);
    const auto held = v["confirmation_seconds"];
    v = tracker.update(frame(10.48, true, 601), 10.481);
    assert(v["target_observed"] == true && v["center"][0] == 601);
    assert(v["confirmation_seconds"] == held);
    auto wrong = frame(10.6);
    wrong["tag_id"] = 1;
    v = tracker.update(wrong, 10.601);
    assert(v["target_retained"] == true && v["tag_id"] == 0);
    v = tracker.update(frame(11.1, false), 11.101);
    assert(v["target_found"] == false && v["confirmation_seconds"] == 0);
    v = tracker.update(frame(11.2), 11.201);
    assert(v["tracking_epoch"] == 2 && v["confirmation_seconds"] == 0);
    // A snapshot writer cannot manufacture fresh observations or completion.
    repeated = frame(11.2);
    repeated["written_monotonic"] = 11.8;
    v = tracker.update(repeated, 11.801);
    assert(v["target_found"] == false && v["target_observed"] == false);
    std::cout << "PASS: distinct detection evidence, intermittent loss, jump confirmation, "
                 "identity expiry\n";
}
