#ifndef PRM_H
#define PRM_H

// #include "global_body_planner/planner_class.h"
#include "global_body_planner/prm_planner_class.h"
#include "global_body_planner/kdtree.h"


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

    void buildRoadmap2(PRM_PlannerClass &G, const PlannerConfig &planner_config, const int &start, const int &goal, const int &num_samples, const double &epsilon);

    bool GetAction(State start, State s, StateActionResult &result, const PlannerConfig &planner_config, bool &check);

    // Method to find a path from start to goal
    std::vector<int> findPath(int start, int goal);

    std::vector<int> Astar(PRM_PlannerClass &G, const int &start, const int &goal, const double &epsilon, const PlannerConfig &planner_config);

    std::vector<int> WAstar(PRM_PlannerClass &G, const int &start, const int &goal, const double &epsilon, const double &w, const PlannerConfig &planner_config);

    bool IsInGraph(const State &s, PRM_PlannerClass &G, const PlannerConfig &planner_config);

    void reset_gValues(PRM_PlannerClass &G);

    void add_and_updateG(PRM_PlannerClass &G, const State &s, const int &s_index, const State &goal, const double &epsilon, const PlannerConfig &planner_config);

    bool checkTrajectoryCollsion(PRM_PlannerClass &G, const std::vector<State> &state_sequence, const PlannerConfig &planner_config);

    bool testCollision(PRM_PlannerClass &G, std::vector<State> &state_sequence, const PlannerConfig &planner_config);


protected:
    // Method to sample random points
    State samplePoint(PRM_PlannerClass &G, const PlannerConfig &planner_config);

    bool calculateDirectAction(const State &s_start, const State &s_goal, StateActionResult &result, const PlannerConfig &planner_config, bool &check);

    bool sampleLeapAction(const State &s_start, const State &s_goal, StateActionResult &result, const PlannerConfig &planner_config, bool &check);

    void updateDistHeuristic(PRM_PlannerClass &G, const PlannerConfig &planner_config, const State &s, const int &s_index, const State &goal);
    
    std::vector<int> GetNeighbors(PRM_PlannerClass &G, const State &state, const double &epsilon);

    bool checkConnection(PRM_PlannerClass &G, int& s, int& neighbor);

    
    // Method to connect nodes in the roadmap
    // void connectNodes(PRM_PlannerClass &G, const PlannerConfig &planner_config);

    // PlannerClass instance to manage the graph
    // PlannerClass planner_;
    // int num_samples_;
    // PlannerConfig planner_config_;

    struct CompareFValues {
        CompareFValues(const PRM_PlannerClass &graph) : graph_(graph) {}

        bool operator()(int lhs, int rhs) const {
            return graph_.g_values.at(lhs) + graph_.h_dist.at(lhs) > graph_.g_values.at(rhs) + graph_.h_dist.at(rhs);
        }

        const PRM_PlannerClass &graph_;
    };

};

#endif // PRM_H