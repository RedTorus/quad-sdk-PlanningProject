#ifndef PRM_PLANNER_CLASS_H
#define PRM_PLANNER_CLASS_H

#include "global_body_planner/planner_class.h"
#include <unordered_map>
#include <utility> // For std::pair
#include <boost/functional/hash.hpp>

using namespace planning_utils;

class PRM_PlannerClass : public PlannerClass
{
public:
    PRM_PlannerClass(const PlannerConfig &planner_config, int direction = FORWARD);

    // Overloaded attribute for actions
    std::unordered_map<std::pair<int, int>, Action, boost::hash<std::pair<int, int>>> actions;

    std::unordered_map<int, int> parents;

    std::unordered_map<int, double> h_dist;

    friend class PRM;

    //using GraphClass::vertices;
    //using GraphClass::actions;
    //using GraphClass::edges;
    //using GraphClass::successors;
    //using GraphClass::g_values;
    bool isVerticesEmpty() const;
    // Other methods specific to PRM_PlannerClass can be added here
    std::vector<State> retrieveStateSequence(const std::vector<int>& path);

    std::vector<Action> retrieveActionSequence(const std::vector<int>& path);
};

#endif // PRM_PLANNER_CLASS_H