#include "global_body_planner/prm.h"
#include <vector>
#include <cstdlib> // For random number generation

PRM::PRM() {}

PRM::~PRM() {}

using namespace planning_utils;

void PRM::buildRoadmap(PRM_PlannerClass &G, const PlannerConfig &planner_config,const int &num_samples,const double &epsilon) {
    StateActionResult result;
    State goal = G.getVertex(1);
    for (int i = 0; i < num_samples; ++i) {
        State random_state = samplePoint(G, planner_config);
        
        if ((!isValidState(random_state, planner_config, LEAP_STANCE)) || (IsInGraph(random_state, G, planner_config))) {
            continue;
        }
        int s_new_index = G.getNumVertices();
        G.addVertex(s_new_index, random_state);
        updateDistHeuristic(G, planner_config, random_state, s_new_index, goal);

        std::vector<int> neighbors = G.neighborhoodDist(random_state, epsilon); //ToDo put dist in PlannerConfig
        if (neighbors.empty()) {
            continue;
        }

        for (int neighbor : neighbors) {
            State neighbor_state = G.getVertex(neighbor);
            if (GetAction(neighbor_state, random_state, planner_config)) {
                G.addEdge(s_new_index, neighbor, 0.0); //do not want edge cost for now edge len result.distance
                G.actions[{neighbor, s_new_index}] = result.a_new;
                //G.actions[{s_new_index, neighbor}] = flipDirection(a)
            }

            if (GetAction(random_state, neighbor_state, planner_config)) {
                G.addEdge(neighbor, s_new_index, 0.0);
                G.actions[{s_new_index, neighbor}] = result.a_new;
                //G.actions[{neighbor, s_new_index}] = flipDirection(a)
            }
        }
    }
    //connectNodes(G, planner_config);
}

bool PRM::GetAction(State start, State s, const PlannerConfig &planner_config) {
    // Implement validity checking logic
    StateActionResult result;
    if (calculateDirectAction(start, s, result, planner_config)) {
        return true;
    } else if (sampleLeapAction(start, s, result, planner_config)) {
        return true;
    }
    return false;
}

std::vector<int> PRM::findPath(int start, int goal) {
    // Implement pathfinding algorithm (e.g., A* or Dijkstra's)
    std::vector<int> path;
    // Placeholder implementation
    path.push_back(start);
    path.push_back(goal);
    return path;
}

State PRM::samplePoint(PRM_PlannerClass &G, const PlannerConfig &planner_config) {
    return G.randomState(planner_config);
}

bool PRM::calculateDirectAction(const State &s_start, const State &s_goal, StateActionResult &result, const PlannerConfig &planner_config) {
    // Calculate the stance time based on the distance and velocities
    double t_s = 6.0 * poseDistance(s_start, s_goal) /
                 ((s_start.vel + s_goal.vel).norm() + 4.0 * planner_config.v_nom);

    // Compute accelerations at start and end of the behavior
    Eigen::Vector3d acc_0 =
        -(2.0 * (3.0 * s_start.pos - 3.0 * s_goal.pos + 2.0 * s_start.vel * t_s +
                 s_goal.vel * t_s)) /
        (t_s * t_s);
    Eigen::Vector3d acc_f = (2.0 * (3.0 * s_start.pos - 3.0 * s_goal.pos +
                                    s_start.vel * t_s + 2.0 * s_goal.vel * t_s)) /
                            (t_s * t_s);

    // Transform from accelerations to body weight ground reaction forces (GRFs)
    result.a_new.grf_0 = (acc_0 - planner_config.g_vec) / planner_config.g;
    result.a_new.grf_f = (acc_f - planner_config.g_vec) / planner_config.g;

    // Set the vertical component of GRFs to contain height above terrain
    result.a_new.grf_0[2] = s_start.pos[2] - getTerrainZFilteredFromState(s_start, planner_config);
    result.a_new.grf_f[2] = s_goal.pos[2] - getTerrainZFilteredFromState(s_goal, planner_config);

    // Set the action parameters
    result.a_new.t_s_leap = t_s;
    result.a_new.t_f = 0;
    result.a_new.t_s_land = 0;
    result.a_new.dz_0 = getDzFromState(s_start, planner_config);
    result.a_new.dz_f = getDzFromState(s_goal, planner_config);

    // Check if the action is valid
    if (isValidAction(result.a_new, planner_config)) {
        // Validate the state-action pair
        if (isValidStateActionPair(s_start, result.a_new, result, planner_config)) {
            return true;
        }
    }

    return false;
}

bool PRM::sampleLeapAction(const State &s_start, const State &s_goal, StateActionResult &result, const PlannerConfig &planner_config) {
    Eigen::Vector3d surf_norm = getSurfaceNormalFiltered(s_start, planner_config);

    int num_total_actions = 0;
    int num_valid_actions = 0;

    for (int i = 0; i < planner_config.num_leap_samples; ++i) {
        bool valid_state_found = false;
        Action a_test;
        bool is_valid_initial = getRandomLeapAction(s_start, surf_norm, a_test, planner_config);
        num_total_actions++;
        if (!is_valid_initial) {
            continue;
        }

        for (int j = 0; j < planner_config.num_leap_samples; ++j) {
            bool is_valid = isValidStateActionPair(s_start, a_test, result, planner_config);
            if (is_valid) {
                valid_state_found = true;
                num_valid_actions++;
                break;
            } else {
                is_valid_initial = getRandomLeapAction(s_start, surf_norm, a_test, planner_config);
                num_total_actions++;
            }
        }

        if (valid_state_found) {
            return true;
        }
    }

    return false;
}

bool PRM::IsInGraph(const State &s, PRM_PlannerClass &G, const PlannerConfig &planner_config) {
    if(G.getNumVertices() == 0) {
        return false;
    }
    /* for(int i = 0; i < G.getNumVertices(); i++) {
        if(stateDistance(s, G.getVertex(i)) < planner_config.epsilon) {
            return true;
        }
    } */

    int n = G.getNearestNeighbor(s);
    State neighbor = G.getVertex(n);
    double delta = stateDistance(s, neighbor);
    double tres = 0.1;
    if(delta < tres) {
        return true;
    }

    return false;
}

void PRM::updateDistHeuristic(PRM_PlannerClass &G, const PlannerConfig &planner_config, const State &s, const int &s_index, const State &goal) {
    double delta = stateDistance(s, goal);
    G.h_dist[s_index] = delta;
}

/* void PRM::connectNodes(PRM_PlannerClass &G, const PlannerConfig &planner_config) {
    // Implement logic to connect nodes in the graph
    // Placeholder implementation
} */