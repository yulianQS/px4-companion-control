#pragma once
#include "json.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

// Measurement continuity only. Retained geometry is never a new observation.
class VisionTrack {
    using Json = nlohmann::json;
    Json last, candidate;
    double frame = 0, seen = 0, evidence = 0;
    bool observed = false;
    std::string state_reason = "missing";
    unsigned epoch = 0, candidate_hits = 0;
    double retention, jump;
    unsigned jump_frames;

    static bool geometry(const Json &v) {
        if (!v.value("camera_ready", false) || !v.value("target_found", false) ||
            v.value("tag_id", -1) != 0 || !v.contains("center") || !v["center"].is_array() ||
            v["center"].size() != 2)
            return false;
        const double w = v.value("width", 0.), h = v.value("height", 0.);
        if (w < 2 || h < 2 || w > 4096 || h > 4096)
            return false;
        for (int i = 0; i < 2; ++i)
            if (!v["center"][i].is_number() || !std::isfinite(v["center"][i].get<double>()) ||
                v["center"][i].get<double>() < 0 || v["center"][i].get<double>() >= (i ? h : w))
                return false;
        return true;
    }
    static double separation(const Json &a, const Json &b) {
        return std::hypot(a["center"][0].get<double>() / a["width"].get<double>() -
                              b["center"][0].get<double>() / b["width"].get<double>(),
                          a["center"][1].get<double>() / a["height"].get<double>() -
                              b["center"][1].get<double>() / b["height"].get<double>());
    }

  public:
    explicit VisionTrack(const Json &config = Json::object())
        : retention(config.value("retain_seconds", .5)),
          jump(config.value("max_center_jump_normalized", .25)),
          jump_frames(config.value("confirm_jump_frames", 2u)) {
        if (!std::isfinite(retention) || retention <= 0 || !std::isfinite(jump) || jump < 0 ||
            jump_frames < 1)
            throw std::runtime_error("vision_tracking 配置无效");
    }
    Json update(Json raw, double now) {
        if (!raw.is_object())
            raw = Json::object();
        if (!last.is_null() && now - seen > retention) {
            last = candidate = Json();
            evidence = 0;
            candidate_hits = 0;
            observed = false;
        }
        const double stamp = raw.value("written_monotonic", 0.);
        const double observation = raw.value("observation_monotonic", stamp);
        const bool valid = raw.value("source", std::string()) == "ros2_cpp" && stamp > 0 &&
                           observation > 0 && observation <= now && now >= stamp &&
                           now - stamp <= .5 && now - observation <= retention && geometry(raw);
        std::string reason = valid ? state_reason : "missing";
        if (std::isfinite(observation) && observation <= now && now - observation <= .5 &&
            observation > frame + 1e-6) {
            bool accept = valid;
            reason = valid ? "observed" : "missing";
            if (accept && !last.is_null() && jump > 0 && separation(last, raw) > jump) {
                if (!candidate.is_null() && separation(candidate, raw) <= jump)
                    ++candidate_hits;
                else
                    candidate_hits = 1;
                candidate = raw;
                accept = candidate_hits >= jump_frames;
                reason = accept ? "jump_confirmed" : "jump_pending";
            } else {
                candidate = Json();
                candidate_hits = 0;
            }
            if (accept) {
                if (last.is_null()) {
                    ++epoch;
                    evidence = 0;
                } else if (observed)
                    evidence += std::min(.1, observation - frame);
                last = raw;
                seen = observation;
            }
            observed = accept;
            frame = observation;
        } else if (!valid)
            observed = false;
        state_reason = reason;
        const bool retained = !last.is_null() && !observed && now - seen <= retention;
        if (retained) {
            for (const auto *key :
                 {"center", "width", "height", "tag_id", "pose_valid", "calibrated", "distance_m",
                  "translation_m", "tag_size_m", "pose_rms_px"})
                if (last.contains(key))
                    raw[key] = last[key];
            raw["source"] = last["source"];
            if (!raw.contains("written_monotonic"))
                raw["written_monotonic"] = last["written_monotonic"];
            raw["target_found"] = true;
        } else if (!observed)
            raw["target_found"] = false;
        raw["target_observed"] = observed;
        raw["target_retained"] = retained;
        raw["tracking_epoch"] = epoch;
        raw["last_observed_monotonic"] = seen;
        raw["confirmation_seconds"] = evidence;
        raw["tracking_reason"] = reason;
        raw["tracking_retention_seconds"] = retention;
        return raw;
    }
};
