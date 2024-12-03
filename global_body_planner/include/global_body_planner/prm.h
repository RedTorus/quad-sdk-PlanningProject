#ifndef PRM_H
#define PRM_H

// #include "global_body_planner/planner_class.h"
#include "global_body_planner/prm_planner_class.h"
#include "kdtree.h"

using namespace planning_utils;

class PRM
{

private:
    KDTree kd_tree;

public:
    // Constructor
    PRM();

    // Destructor
    ~PRM();

    // Method to build the roadmap
    void buildRoadmap(PRM_PlannerClass &G, const PlannerConfig &planner_config, const int &num_samples, const double &epsilon);

    bool GetAction(State start, State s, const PlannerConfig &planner_config);

    // Method to find a path from start to goal
    std::vector<int> findPath(int start, int goal);

protected:
    // Method to sample random points
    State samplePoint(PRM_PlannerClass &G, const PlannerConfig &planner_config);

    bool calculateDirectAction(const State &s_start, const State &s_goal, StateActionResult &result, const PlannerConfig &planner_config);

    bool sampleLeapAction(const State &s_start, const State &s_goal, StateActionResult &result, const PlannerConfig &planner_config);

    bool IsInGraph(const State &s, PRM_PlannerClass &G, const PlannerConfig &planner_config);

    void updateDistHeuristic(PRM_PlannerClass &G, const PlannerConfig &planner_config, const State &s, const int &s_index, const State &goal);
<<<<<<< HEAD
    std::vector<int> PRM::GetNeighbors(PRM_PlannerClass &G, const State &state, const double &epsilon);
=======
>>>>>>> 022c88680b2cf1b3484e7f801e58d5ca1305f356

    // Method to connect nodes in the roadmap
    // void connectNodes(PRM_PlannerClass &G, const PlannerConfig &planner_config);

    // PlannerClass instance to manage the graph
    // PlannerClass planner_;
    // int num_samples_;
    // PlannerConfig planner_config_;
};

#endif // PRM_H