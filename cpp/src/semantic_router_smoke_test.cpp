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
                                                node("disabled_node", "disabled target", json::array({"real_site", "disabled"})),
                                            })},
                     },
                 })},
    };

    go2w::SemanticRouter router(registry, "go2w_real_site");
    require(router.planText("go enabled target", 0.3, 0).matched, "enabled node should resolve");
    require(!router.planText("go disabled target", 0.3, 0).matched, "disabled node should not resolve");
    require(router.findNode("disabled_node") == nullptr, "disabled node should not be returned by findNode");
    return 0;
}
