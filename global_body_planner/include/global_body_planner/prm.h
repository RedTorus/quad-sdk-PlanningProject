// #ifndef PRM_H
// #define PRM_H

// // #include "global_body_planner/planner_class.h"
// #include "global_body_planner/prm_planner_class.h"
// #include "kdtree.h"
// #include <boost/graph/adjacency_list.hpp>
// #include <boost/graph/astar_search.hpp>
// using namespace planning_utils;

// class PRM
// {

// private:
//     KDTree kd_tree;

// public:
//     // Constructor
//     PRM();

//     // Destructor
//     ~PRM();

//     // Method to build the roadmap
//     void buildRoadmap(PRM_PlannerClass &G, const PlannerConfig &planner_config, const int &num_samples, const double &epsilon);

//     std::vector<int> findPathUsingAStar(PRM_PlannerClass &G, int start, int goal);


//     Action getAction(int from_node, int to_node);

//     State getState(int node_index);
//     std::unordered_map<std::pair<int, int>, Action, boost::hash<std::pair<int, int>>> actions;


    

//     // Method to find a path from start to goal
//     std::vector<int> findPath(int start, int goal);

// protected:
//     // Method to sample random points
//     State samplePoint(PRM_PlannerClass &G, const PlannerConfig &planner_config);

//     bool calculateDirectAction(const State &s_start, const State &s_goal, StateActionResult &result, const PlannerConfig &planner_config);

//     bool sampleLeapAction(const State &s_start, const State &s_goal, StateActionResult &result, const PlannerConfig &planner_config);

//     bool IsInGraph(const State &s, PRM_PlannerClass &G, const PlannerConfig &planner_config);

//     void updateDistHeuristic(PRM_PlannerClass &G, const PlannerConfig &planner_config, const State &s, const int &s_index, const State &goal);

//     // Method to connect nodes in the roadmap
//     // void connectNodes(PRM_PlannerClass &G, const PlannerConfig &planner_config);

//     // PlannerClass instance to manage the graph
//     // PlannerClass planner_;
//     // int num_samples_;
//     // PlannerConfig planner_config_;
// };

// #endif // PRM_H

#ifndef PRM_H
#define PRM_H

#include "global_body_planner/prm_planner_class.h"
#include "kdtree.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>

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

    std::vector<int> findPathUsingAStar(PRM_PlannerClass &G, int start, int goal);
    Action findAction(PRM_PlannerClass &G, int from_node, int to_node);
    State findState(PRM_PlannerClass &G, int node_index);

    // Action findAction(int from_node, int to_node); // Renamed
    // State findState(int node_index);              // Renamed

protected:
    State samplePoint(PRM_PlannerClass &G, const PlannerConfig &planner_config);

    bool calculateDirectAction(const State &s_start, const State &s_goal, StateActionResult &result, const PlannerConfig &planner_config);

    bool sampleLeapAction(const State &s_start, const State &s_goal, StateActionResult &result, const PlannerConfig &planner_config);

    bool IsInGraph(const State &s, PRM_PlannerClass &G, const PlannerConfig &planner_config);

    void updateDistHeuristic(PRM_PlannerClass &G, const PlannerConfig &planner_config, const State &s, const int &s_index, const State &goal);
};

#endif // PRM_H
