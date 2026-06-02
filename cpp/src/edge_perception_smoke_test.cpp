#include "go2w/edge_perception.hpp"

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main()
{
    const std::string missing = "/tmp/go2w_edge_perception_missing.json";
    std::remove(missing.c_str());
    const auto offline = go2w::loadEdgePerceptionSummary(missing, 3000, 10000);
    require(!offline.value("available", true), "missing summary must stay unavailable");
    require(!offline.value("safety_wired", true), "external node must not auto-wire safety");

    const std::string path = "/tmp/go2w_edge_perception_smoke.json";
    {
        std::ofstream output(path);
        output << R"({
          "schema_version": 1,
          "node_id": "nx_ti_radar_01",
          "sensor_type": "ti_millimeter_wave_radar",
          "source": "nx_ti_radar",
          "timestamp_ms": 9000,
          "confidence": 0.85,
          "latency_ms": 40,
          "health": {"status": "ok"},
          "policy": {"mode": "semantic_only", "calibrated": false, "safety_candidate": false},
          "observations": [{"type": "person", "range_m": 2.4}]
        })";
    }
    const auto semantic_only = go2w::loadEdgePerceptionSummary(path, 3000, 10000);
    require(semantic_only.value("available", false), "fresh summary should load");
    require(semantic_only.value("eligible_for_llm", false), "fresh summary should be visible to LLM");
    require(!semantic_only.value("safety_candidate", true), "uncalibrated summary must not become a safety candidate");
    require(!semantic_only.value("safety_wired", true), "summary loading must not bypass SafetyGate wiring");

    const auto stale = go2w::loadEdgePerceptionSummary(path, 500, 10000);
    require(!stale.value("eligible_for_llm", true), "old summary should not enter LLM context");
    require(stale.value("status", std::string("")) == "stale_or_unhealthy", "stale summary status mismatch");

    std::remove(path.c_str());

    const std::string oversized = "/tmp/go2w_edge_perception_oversized.json";
    {
        std::ofstream output(oversized);
        output << std::string(256 * 1024 + 1, 'x');
    }
    const auto rejected = go2w::loadEdgePerceptionSummary(oversized, 3000, 10000);
    require(!rejected.value("available", true), "oversized summary must be rejected");
    require(rejected.value("status", std::string("")) == "invalid_summary", "oversized summary status mismatch");
    std::remove(oversized.c_str());

    return 0;
}
