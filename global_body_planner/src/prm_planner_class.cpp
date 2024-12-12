#include "global_body_planner/prm_planner_class.h"

PRM_PlannerClass::PRM_PlannerClass(const PlannerConfig &planner_config, int direction)
    : PlannerClass(direction, planner_config) {
    // Initialize any additional attributes if necessary
}

bool PRM_PlannerClass::isVerticesEmpty() const {
    return vertices.empty();
}

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
            /* ROS_INFO("-----------Trying action calc");
            PRM prm;
            StateActionResult result;
            PlannerConfig planner_config;
            if (prm.calculateDirectAction(getVertex(path[i - 1]), getVertex(path[i]), result, planner_config)) {
                ROS_INFO("-----------manually adding action for path nr %d", i);
                action_sequence.push_back(result.a_new);
            } else { */
            ROS_ERROR("-----------AT path nr %d", i);
            throw std::runtime_error("Action not found in the graph.");
            //}
            /* ROS_INFO("-----------AT path nr %d", i);
            throw std::runtime_error("Action not found in the graph."); */
        }
    }
    return action_sequence;
}