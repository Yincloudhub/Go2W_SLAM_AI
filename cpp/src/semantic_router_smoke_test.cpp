#include "go2w/semantic_router.hpp"

#include <iostream>

using nlohmann::json;

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << "\n";
        std::exit(1);
    }
}

json node(const std::string& node_id, const std::string& name, json tags)
{
    return {
        {"node_id", node_id},
        {"name", name},
        {"aliases", json::array({name, node_id})},
        {"tags", std::move(tags)},
        {"pose", {{"x", 1.0}, {"y", 2.0}, {"z", 0.0}, {"q_x", 0.0}, {"q_y", 0.0}, {"q_z", 0.0}, {"q_w", 1.0}}},
    };
}

}  // namespace

int main()
{
    const json registry = {
        {"default_map_id", "go2w_real_site"},
        {"maps", json::array({
                     {
                         {"map_id", "go2w_real_site"},
                         {"topology_nodes", json::array({
                                                node("enabled_node", "enabled target", json::array({"real_site"})),
                                                node("return_node", "return target", json::array({"real_site"})),
                                                node("disabled_node", "disabled target", json::array({"real_site", "disabled"})),
                                            })},
                     },
                 })},
    };

    go2w::SemanticRouter router(registry, "go2w_real_site");
    require(router.planText("go enabled target", 0.3, 0).matched, "enabled node should resolve");
    require(!router.planText("go disabled target", 0.3, 0).matched, "disabled node should not resolve");
    require(router.findNode("disabled_node") == nullptr, "disabled node should not be returned by findNode");
    const auto capture_route = router.planText("去 enabled target 拍一张照，然后回 return target", 0.3, 0);
    require(capture_route.matched, "multi-target capture command should resolve");
    require(capture_route.task_queue["steps"].size() == 3, "capture command should expand to navigate, capture, navigate");
    require(capture_route.task_queue["steps"][0]["action"] == "navigate", "first step should navigate to capture target");
    require(capture_route.task_queue["steps"][1]["action"] == "capture_keyframe", "second step should capture at first target");
    require(capture_route.task_queue["steps"][1]["target_node"] == "enabled_node", "capture should stay attached to first target");
    require(capture_route.task_queue["steps"][2]["action"] == "navigate", "third step should navigate to return target");

    auto ambiguous_registry = registry;
    ambiguous_registry["maps"][0]["topology_nodes"][0]["aliases"].push_back("shared lab");
    ambiguous_registry["maps"][0]["topology_nodes"][1]["aliases"].push_back("shared lab");
    go2w::SemanticRouter ambiguous_router(ambiguous_registry, "go2w_real_site");
    const auto ambiguous = ambiguous_router.planText("go shared lab", 0.3, 0);
    require(!ambiguous.matched, "shared alias without sequence must not resolve");
    require(ambiguous.ambiguous, "shared alias without sequence must be marked ambiguous");
    const auto explicit_sequence = ambiguous_router.planText(
        "go enabled target then return target",
        0.3,
        0);
    require(explicit_sequence.matched, "explicit sequence should resolve");
    require(explicit_sequence.multi_target, "explicit sequence should remain multi-target");
    return 0;
}
