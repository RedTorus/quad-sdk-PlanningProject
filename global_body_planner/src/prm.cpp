#include "global_body_planner/prm.h"
#include <vector>
#include <cstdlib> // For random number generation
#include <queue>
#include <unordered_set>
#include <algorithm> 
#include <chrono>

PRM::PRM() {}

PRM::~PRM() {}

using namespace planning_utils;

void PRM::buildRoadmap(PRM_PlannerClass &G, const PlannerConfig &planner_config, const int &num_samples, const double &epsilon)
{   
    auto start_time = std::chrono::steady_clock::now(); // Start timing
    StateActionResult result;
    bool check=false;
    State goal = G.getVertex(1);
    G.updateGValue(1, std::numeric_limits<double>::max()-10);
    G.updateGValue(0, 0);
    for (int i = 0; i < num_samples; ++i)
    {
        State random_state = samplePoint(G, planner_config);

        if ((!isValidState2(random_state, planner_config, LEAP_STANCE, check)) || (IsInGraph(random_state, G, planner_config)))
        {
            continue;
        }
        int s_new_index = G.getNumVertices();
        G.addVertex(s_new_index, random_state);
        updateDistHeuristic(G, planner_config, random_state, s_new_index, goal);
        G.updateGValue(s_new_index, std::numeric_limits<double>::max()-10);
        std::vector<int> neighbors = G.neighborhoodDist(random_state, epsilon); // ToDo put dist in PlannerConfig
        if (neighbors.empty())
        {
            continue;
        }

        if (i%1000 == 0)
        {
            ROS_INFO("Sampled %d states", i);
        }	

        for (int neighbor : neighbors)
        {
            State neighbor_state = G.getVertex(neighbor);
            if (calculateDirectAction(neighbor_state, random_state, result, planner_config, check))
            {
                G.addEdge(s_new_index, neighbor, 0.0); // do not want edge cost for now edge len result.distance
                G.actions[{neighbor, s_new_index}] = result.a_new;
                // G.actions[{s_new_index, neighbor}] = flipDirection(a)
            }

            if (calculateDirectAction(random_state, neighbor_state, result, planner_config, check))
            {
                G.addEdge(neighbor, s_new_index, 0.0);
                G.actions[{s_new_index, neighbor}] = result.a_new;
                // G.actions[{neighbor, s_new_index}] = flipDirection(a)
            }
        }
    }
    auto end_time = std::chrono::steady_clock::now(); // End timing
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    ROS_INFO("BuildRoadmap completed in %f seconds", elapsed_seconds.count());
    ROS_INFO("Number of nodes in PRM: %d", G.getNumVertices());
}

void PRM::buildRoadmap2(PRM_PlannerClass &G, const PlannerConfig &planner_config, const int &start, const int &goal, const int &num_samples, const double &epsilon)
{   
    auto start_time = std::chrono::steady_clock::now(); // Start timing
    StateActionResult result;
    bool check=false;
    State goalS = G.getVertex(goal);
    G.updateGValue(goal, std::numeric_limits<double>::max()-30);
    G.updateGValue(start, 0);
    for (int i = 0; i < num_samples; ++i)
    {
        State random_state = samplePoint(G, planner_config);

        if ((!isValidState2(random_state, planner_config, LEAP_STANCE, check)) || (IsInGraph(random_state, G, planner_config)))
        {
            continue;
        }
        int s_new_index = G.getNumVertices();
        G.addVertex(s_new_index, random_state);
        updateDistHeuristic(G, planner_config, random_state, s_new_index, goalS);
        G.updateGValue(s_new_index, std::numeric_limits<double>::max()-30);
        std::vector<int> neighbors = G.neighborhoodDist(random_state, epsilon); // ToDo put dist in PlannerConfig
        if (neighbors.empty())
        {
            continue;
        }

        if (i%1000 == 0)
        {
            ROS_INFO("Sampled %d states", i);
        }	

        for (int neighbor : neighbors)
        {
            State neighbor_state = G.getVertex(neighbor);
            if (calculateDirectAction(neighbor_state, random_state, result, planner_config, check))
            {
                G.addEdge(s_new_index, neighbor, 0.0); // do not want edge cost for now edge len result.distance
                G.actions[{neighbor, s_new_index}] = result.a_new;
                // G.actions[{s_new_index, neighbor}] = flipDirection(a)
            }

            if (calculateDirectAction(random_state, neighbor_state, result, planner_config, check))
            {
                G.addEdge(neighbor, s_new_index, 0.0);
                G.actions[{s_new_index, neighbor}] = result.a_new;
                // G.actions[{neighbor, s_new_index}] = flipDirection(a)
            }
        }
    }
    auto end_time = std::chrono::steady_clock::now(); // End timing
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    ROS_INFO("BuildRoadmap completed in %f seconds", elapsed_seconds.count());
    ROS_INFO("Number of nodes in PRM: %d", G.getNumVertices());
}

bool PRM::GetAction(State start, State s, StateActionResult &result, const PlannerConfig &planner_config, bool &check)
{
    // Implement validity checking logic
    
    if (calculateDirectAction(start, s, result, planner_config, check))
    {
        return true;
    }
    else if (sampleLeapAction(start, s, result, planner_config, check))
    {
        return true;
    }
    return false;
}

std::vector<int> PRM::findPath(int start, int goal)
{
    // Implement pathfinding algorithm (e.g., A* or Dijkstra's)
    std::vector<int> path;
    // Placeholder implementation
    path.push_back(start);
    path.push_back(goal);

    return path;
}

State PRM::samplePoint(PRM_PlannerClass &G, const PlannerConfig &planner_config)
{
    return G.randomState(planner_config);
}

bool PRM::calculateDirectAction(const State &s_start, const State &s_goal, StateActionResult &result, const PlannerConfig &planner_config, bool &check)
{
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
    if (isValidAction(result.a_new, planner_config))
    {
        // Validate the state-action pair
        if (isValidStateActionPair2(s_start, result.a_new, result, planner_config, check))
        {
            return true;
        }
    }

    return false;
}

bool PRM::sampleLeapAction(const State &s_start, const State &s_goal, StateActionResult &result, const PlannerConfig &planner_config, bool &check)
{
    Eigen::Vector3d surf_norm = getSurfaceNormalFiltered(s_start, planner_config);

    int num_total_actions = 0;
    int num_valid_actions = 0;

    for (int i = 0; i < planner_config.num_leap_samples; ++i)
    {
        bool valid_state_found = false;
        Action a_test;
        bool is_valid_initial = getRandomLeapAction(s_start, surf_norm, a_test, planner_config);
        num_total_actions++;
        if (!is_valid_initial)
        {
            continue;
        }

        for (int j = 0; j < planner_config.num_leap_samples; ++j)
        {
            bool is_valid = isValidStateActionPair2(s_start, a_test, result, planner_config, check);
            if (is_valid)
            {
                valid_state_found = true;
                num_valid_actions++;
                break;
            }
            else
            {
                is_valid_initial = getRandomLeapAction(s_start, surf_norm, a_test, planner_config);
                num_total_actions++;
            }
        }

        if (valid_state_found)
        {
            return true;
        }
    }

    return false;
}

bool PRM::IsInGraph(const State &s, PRM_PlannerClass &G, const PlannerConfig &planner_config)
{
    if (G.getNumVertices() == 0)
    {
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
    if (delta < tres)
    {
        return true;
    }

    return false;
}

void PRM::updateDistHeuristic(PRM_PlannerClass &G, const PlannerConfig &planner_config, const State &s, const int &s_index, const State &goal)
{
    double delta = stateDistance(s, goal);
    G.h_dist[s_index] = delta;
}

std::vector<int> PRM::GetNeighbors(PRM_PlannerClass &G, const State &state, const double &epsilon) {

    return G.neighborhoodDist(state, epsilon); 
}

std::vector<int> PRM::Astar(PRM_PlannerClass &G, const int &start, const int &goal, const double &epsilon, const PlannerConfig &planner_config) {
    auto start_time = std::chrono::steady_clock::now();
    std::vector<int> path;
    auto compare = [&G](int lhs, int rhs) {
        return G.g_values[lhs] + G.h_dist[lhs] > G.g_values[rhs] + G.h_dist[rhs];
    };
    std::priority_queue<int, std::vector<int>, decltype(compare)> open(compare);
    std::unordered_set<int> open_set;
    std::unordered_set<int> closed;
    bool check=true;
    //int start = 0;
    G.g_values[start] = 0;
    open.push(start);
    open_set.insert(start);
    int current= start;
    while(!open.empty()){

        current  = open.top();
        open.pop();
        open_set.erase(current);
        closed.insert(current);

        if(current == goal){
            ROS_INFO("--------Goal reached");
            break;
        }

        std::vector<int> neighbors = GetNeighbors(G, G.getVertex(current), epsilon);
        for(int neighbor : neighbors){
            bool a = checkConnection(G, current, neighbor);
            /* auto it = G.actions.find({current, neighbor});
            if (it != G.actions.end()) {
                continue;
            } */
            if( neighbor == goal){
                //ROS_INFO("--------Goal found");
                //break;
            }

            if (!isValidState2(G.getVertex(neighbor), planner_config, LEAP_STANCE, check)) {
                continue;
            }

            if ((G.g_values[neighbor] > G.g_values[current] + stateDistance(G.getVertex(current), G.getVertex(neighbor)))&& a){
                G.g_values[neighbor] = G.g_values[current] + stateDistance(G.getVertex(current), G.getVertex(neighbor));
                G.parents[neighbor] = current;
                if(open_set.find(neighbor) == open_set.end()){
                    open.push(neighbor);
                    open_set.insert(neighbor);
                }
            }

        }
    }
    if (current != goal){
        ROS_ERROR("Goal not found");
        throw std::runtime_error("Goal not found");
    }
    while(current != start){
        path.push_back(current);
        current = G.parents[current];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    ROS_INFO("---------Path found");

    auto end_time = std::chrono::steady_clock::now(); // End timing
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    ROS_INFO("Astar completed in %f seconds", elapsed_seconds.count());
    ROS_INFO("Cost of Astar (g value of goal): %f", G.g_values[1]);
    ROS_INFO("Number of nodes in closed list: %d", closed.size());
    return path;

}

std::vector<int> PRM::WAstar(PRM_PlannerClass &G, const int &start, const int &goal, const double &epsilon, const double &w, const PlannerConfig &planner_config) {
    auto start_time = std::chrono::steady_clock::now();
    std::vector<int> path;
    auto compare = [&G, w](int lhs, int rhs) {
        return G.g_values[lhs] + w*G.h_dist[lhs] > G.g_values[rhs] + w*G.h_dist[rhs];
    };
    std::priority_queue<int, std::vector<int>, decltype(compare)> open(compare);
    std::unordered_set<int> open_set;
    std::unordered_set<int> closed;
    bool check=true;
    //int start = 0;
    G.g_values[start] = 0;
    open.push(start);
    open_set.insert(start);
    int current= start;
    while(!open.empty()){

        current  = open.top();
        open.pop();
        open_set.erase(current);
        closed.insert(current);

        if(current == goal){
            ROS_INFO("--------Goal reached");
            break;
        }

        std::vector<int> neighbors = GetNeighbors(G, G.getVertex(current), epsilon);
        for(int neighbor : neighbors){
            bool a = checkConnection(G, current, neighbor);
            /* auto it = G.actions.find({current, neighbor});
            if (it != G.actions.end()) {
                continue;
            } */
            if( neighbor == goal){
                //ROS_INFO("--------Goal found");
                //break;
            }

            if (!isValidState2(G.getVertex(neighbor), planner_config, LEAP_STANCE, check)) {
                continue;
            }

            if ((G.g_values[neighbor] > G.g_values[current] + stateDistance(G.getVertex(current), G.getVertex(neighbor)))&& a){
                G.g_values[neighbor] = G.g_values[current] + stateDistance(G.getVertex(current), G.getVertex(neighbor));
                G.parents[neighbor] = current;
                if(open_set.find(neighbor) == open_set.end()){
                    open.push(neighbor);
                    open_set.insert(neighbor);
                }
            }

        }
    }
    if (current != goal){
        ROS_ERROR("Goal not found");
        throw std::runtime_error("Goal not found");
    }
    while(current != start){
        path.push_back(current);
        current = G.parents[current];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    ROS_INFO("---------Path found");

    auto end_time = std::chrono::steady_clock::now(); // End timing
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    ROS_INFO("Astar completed in %f seconds", elapsed_seconds.count());
    ROS_INFO("Cost of Astar (g value of goal): %f", G.g_values[1]);
    ROS_INFO("Number of nodes in closed list: %d", closed.size());
    return path;

}

bool PRM::checkConnection(PRM_PlannerClass &G, int& s, int& neighbor) {
    bool is_s_parent = std::find(G.edges[neighbor].begin(), G.edges[neighbor].end(), s) == G.edges[neighbor].end();
    bool is_neighbor_child = std::find(G.successors[s].begin(), G.successors[s].end(), neighbor) == G.successors[s].end();
    bool is_action_there = G.actions.find({s, neighbor}) != G.actions.end();

    return is_s_parent && is_neighbor_child && is_action_there;
}

void PRM::reset_gValues(PRM_PlannerClass &G) {
    for (int i = 0; i < G.getNumVertices(); i++) {
        G.g_values[i] = std::numeric_limits<double>::max()-30;
    }
}




/* void PRM::connectNodes(PRM_PlannerClass &G, const PlannerConfig &planner_config) {
    // Implement logic to connect nodes in the graph
    // Placeholder implementation
} */