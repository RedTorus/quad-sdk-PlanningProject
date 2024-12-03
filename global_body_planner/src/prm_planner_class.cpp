// #include "global_body_planner/prm_planner_class.h"

// PRM_PlannerClass::PRM_PlannerClass(const PlannerConfig &planner_config, int direction)
//     : PlannerClass(direction, planner_config) {
//     // Initialize any additional attributes if necessary
// }


// std::vector<State> PRM_PlannerClass::getStateSequence(const std::vector<int>& path) {
//     std::vector<State> state_sequence;
//     for (int index : path) {
//         state_sequence.push_back(getVertex(index)); // Assuming getVertex() exists.
//     }
//     return state_sequence;
// }

// std::vector<Action> PRM_PlannerClass::getActionSequence(const std::vector<int>& path) {
//     std::vector<Action> action_sequence;
//     for (size_t i = 1; i < path.size(); ++i) {
//         auto it = actions.find({path[i - 1], path[i]});
//         if (it != actions.end()) {
//             action_sequence.push_back(it->second);
//         } else {
//             throw std::runtime_error("Action not found in the graph.");
//         }
//     }
//     return action_sequence;
// }
// std::vector<int> PRM_PlannerClass::getNeighbors(int node_index) {
//     std::vector<int> neighbors;
//     for (const auto& edge : edges[node_index]) { // Assuming edges[node_index] is a map/list.
//         neighbors.push_back(edge.first); // Assuming edge.first is the neighbor's index.
//     }
//     return neighbors;
// }

// double PRM_PlannerClass::getEdgeCost(int from_node, int to_node) {
//     auto it = edges[from_node].find(to_node);
//     if (it != edges[from_node].end()) {
//         return it->second; // Assuming the cost is stored as the value in edges[from_node][to_node].
//     }
//     throw std::runtime_error("Edge does not exist between the nodes.");
// }


#include "global_body_planner/prm_planner_class.h"
#include <algorithm> // Ensure this is included


PRM_PlannerClass::PRM_PlannerClass(const PlannerConfig &planner_config, int direction)
    : PlannerClass(direction, planner_config) {}

std::vector<State> PRM_PlannerClass::retrieveStateSequence(const std::vector<int>& path) {
    std::vector<State> state_sequence;
    for (int index : path) {
        state_sequence.push_back(getVertex(index));
    }
    return state_sequence;
}

std::vector<Action> PRM_PlannerClass::retrieveActionSequence(const std::vector<int>& path) {
    std::vector<Action> action_sequence;
    for (size_t i = 1; i < path.size(); ++i) {
        auto it = actions.find({path[i - 1], path[i]});
        if (it != actions.end()) {
            action_sequence.push_back(it->second);
        } else {
            throw std::runtime_error("Action not found in the graph.");
        }
    }
    return action_sequence;
}

// std::vector<int> PRM_PlannerClass::getNeighbors(int node_index) {
//     std::vector<int> neighbors;
//     for (const auto& edge : edges[node_index]) {
//         neighbors.push_back(edge.first);
//     }
//     return neighbors;
// }


std::vector<int> PRM_PlannerClass::getNeighbors(int node_index) {
    std::vector<int> neighbors;
    for (const int &neighbor : edges[node_index]) { // Assuming edges[node_index] is a vector of integers.
        neighbors.push_back(neighbor);
    }
    return neighbors;
}


// double PRM_PlannerClass::getEdgeCost(int from_node, int to_node) {
//     auto it = edges[from_node].find(to_node);
//     if (it != edges[from_node].end()) {
//         return it->second;
//     }
//     throw std::runtime_error("Edge does not exist between nodes.");
// }

double PRM_PlannerClass::getEdgeCost(int from_node, int to_node) {
    auto it = std::find(edges[from_node].begin(), edges[from_node].end(), to_node);
    if (it != edges[from_node].end()) {
        // Replace `cost` below with your actual cost computation if needed.
        return 1.0; // Placeholder for edge cost. Update based on your logic.
    }
    throw std::runtime_error("Edge does not exist between the nodes.");
}