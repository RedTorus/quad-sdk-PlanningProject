// #ifndef PRM_PLANNER_CLASS_H
// #define PRM_PLANNER_CLASS_H

// #include "global_body_planner/planner_class.h"
// #include <unordered_map>
// #include <utility> // For std::pair
// #include <boost/functional/hash.hpp>

// using namespace planning_utils;

// class PRM_PlannerClass : public PlannerClass
// {
// public:
//     PRM_PlannerClass(const PlannerConfig &planner_config, int direction = FORWARD);

//     // Overloaded attribute for actions
//     std::unordered_map<std::pair<int, int>, Action, boost::hash<std::pair<int, int>>> actions;

//     std::unordered_map<int, double> h_dist;
//     std::vector<State> getStateSequence(const std::vector<int>& path);
//     std::vector<Action> getActionSequence(const std::vector<int>& path);
//     std::vector<int> getNeighbors(int node_index);
//     double getEdgeCost(int from_node, int to_node);



//     // Other methods specific to PRM_PlannerClass can be added here
// };

// #endif // PRM_PLANNER_CLASS_H

#ifndef PRM_PLANNER_CLASS_H
#define PRM_PLANNER_CLASS_H

#include "global_body_planner/planner_class.h"
#include <unordered_map>
#include <utility>
#include <boost/functional/hash.hpp>

using namespace planning_utils;

class PRM_PlannerClass : public PlannerClass {
public:
    PRM_PlannerClass(const PlannerConfig &planner_config, int direction = FORWARD);

    std::unordered_map<std::pair<int, int>, Action, boost::hash<std::pair<int, int>>> actions;

    std::unordered_map<int, double> h_dist;

    std::vector<State> retrieveStateSequence(const std::vector<int>& path);
    std::vector<Action> retrieveActionSequence(const std::vector<int>& path);
    std::vector<int> getNeighbors(int node_index);
    double getEdgeCost(int from_node, int to_node);
};

#endif // PRM_PLANNER_CLASS_H
