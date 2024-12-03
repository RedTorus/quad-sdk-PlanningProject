// #include "global_body_planner/prm.h"
// #include <vector>
// #include <cstdlib> // For random number generation

// PRM::PRM() {}

// PRM::~PRM() {}

// using namespace planning_utils;
// std::vector<State> prm_graph;
// // void PRM::findPathUsingAStar(PRM_PlannerClass &G, int start, int goal) {
// //     PRMGraphTraits::Graph graph;

// //     // Add vertices
// //     for (int i = 0; i < G.getNumVertices(); ++i) {
// //         boost::add_vertex(graph);
// //     }

// //     // Add edges
// //     for (const auto &action : G.actions) {
// //         auto [src, dest] = action.first;
// //         double cost = stateDistance(G.getVertex(src), G.getVertex(dest)); // Example cost
// //         boost::add_edge(src, dest, cost, graph);
// //     }

// //     std::vector<PRMGraphTraits::Vertex> predecessors(boost::num_vertices(graph));
// //     std::vector<double> distances(boost::num_vertices(graph));
// //     try {
// //         boost::astar_search(
// //             graph, start, PRMHeuristic(G, goal),
// //             boost::predecessor_map(&predecessors[0]).distance_map(&distances[0]));
// //     } catch (boost::exception &) {
// //         std::cerr << "Path found to goal!" << std::endl;
// //     }

// //     // Reconstruct the path
// //     std::vector<int> path;
// //     for (int v = goal; v != start; v = predecessors[v]) {
// //         path.push_back(v);
// //     }
// //     path.push_back(start);
// //     std::reverse(path.begin(), path.end());

// //     std::cout << "Path found: ";
// //     for (int v : path) {
// //         std::cout << v << " ";
// //     }
// //     std::cout << std::endl;
// // }
// // bool PRM::GetAction(State start, State s, const PlannerConfig &planner_config)
// // {
// //     // Implement validity checking logic
// //     StateActionResult result;
// //     if (calculateDirectAction(start, s, result, planner_config))
// //     {
// //         return true;
// //     }
// //     else if (sampleLeapAction(start, s, result, planner_config))
// //     {
// //         return true;
// //     }
// //     return false;
// // }

// Action PRM::getAction(int from_node, int to_node) {
//     auto it = actions.find({from_node, to_node});
//     if (it != actions.end()) {
//         return it->second;
//     } else {
//         throw std::runtime_error("Action not found between nodes.");
//     }
// }



// std::vector<int> PRM::findPathUsingAStar(PRM_PlannerClass &G, int start, int goal) {
//     std::unordered_map<int, double> g_cost; // Cost from start to node
//     std::unordered_map<int, double> f_cost; // Cost from start to goal via node
//     std::unordered_map<int, int> came_from; // Node connections
//     std::set<std::pair<double, int>> open_set; // Nodes to explore (f_cost, node)

//     g_cost[start] = 0.0;
//     f_cost[start] = stateDistance(G.getVertex(start), G.getVertex(goal));
//     open_set.insert({f_cost[start], start});

//     while (!open_set.empty()) {
//         // Get node with the lowest f_cost
//         int current = open_set.begin()->second;
//         open_set.erase(open_set.begin());

//         if (current == goal) {
//             // Reconstruct path
//             std::vector<int> path;
//             while (current != start) {
//                 path.push_back(current);
//                 current = came_from[current];
//             }
//             path.push_back(start);
//             std::reverse(path.begin(), path.end());
//             return path;
//         }

//         // Explore neighbors
//         for (int neighbor : G.getNeighbors(current)) {
//             double tentative_g = g_cost[current] + G.getEdgeCost(current, neighbor);

//             if (g_cost.find(neighbor) == g_cost.end() || tentative_g < g_cost[neighbor]) {
//                 // Update costs and path
//                 g_cost[neighbor] = tentative_g;
//                 f_cost[neighbor] = tentative_g + stateDistance(G.getVertex(neighbor), G.getVertex(goal));
//                 came_from[neighbor] = current;

//                 // Add to open set
//                 open_set.insert({f_cost[neighbor], neighbor});
//             }
//         }
//     }

//     // Return empty path if no solution is found
//     return {};
// }




// void PRM::buildRoadmap(PRM_PlannerClass &G, const PlannerConfig &planner_config, const int &num_samples, const double &epsilon)
// {
//     StateActionResult result;
//     State goal = G.getVertex(1);
//     for (int i = 0; i < num_samples; ++i)
//     {
//         State random_state = samplePoint(G, planner_config);

//         if ((!isValidState(random_state, planner_config, LEAP_STANCE)) || (IsInGraph(random_state, G, planner_config)))
//         {
//             continue;
//         }
//         int s_new_index = G.getNumVertices();
//         G.addVertex(s_new_index, random_state);
//         updateDistHeuristic(G, planner_config, random_state, s_new_index, goal);

//         std::vector<int> neighbors = G.neighborhoodDist(random_state, epsilon); // ToDo put dist in PlannerConfig
//         if (neighbors.empty())
//         {
//             continue;
//         }

//         for (int neighbor : neighbors)
//         {
//             State neighbor_state = G.getVertex(neighbor);
//             if (GetAction(neighbor_state, random_state, planner_config))
//             {
//                 G.addEdge(s_new_index, neighbor, 0.0); // do not want edge cost for now edge len result.distance
//                 G.actions[{neighbor, s_new_index}] = result.a_new;
//                 // G.actions[{s_new_index, neighbor}] = flipDirection(a)
//             }

//             if (GetAction(random_state, neighbor_state, planner_config))
//             {
//                 G.addEdge(neighbor, s_new_index, 0.0);
//                 G.actions[{s_new_index, neighbor}] = result.a_new;
//                 // G.actions[{neighbor, s_new_index}] = flipDirection(a)
//             }
//         }
//     }
//     // connectNodes(G, planner_config);
// }



// std::vector<int> PRM::findPath(int start, int goal)
// {
//     // Implement pathfinding algorithm (e.g., A* or Dijkstra's)
//     std::vector<int> path;
//     // Placeholder implementation
//     path.push_back(start);
//     path.push_back(goal);

//     return path;
// }

#include "global_body_planner/prm.h"
#include <vector>
#include <cstdlib>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/exception.hpp>
#include <boost/property_map/property_map.hpp>
#include <cmath>
#include <stdexcept>
#include <vector>

State PRM::samplePoint(PRM_PlannerClass &G, const PlannerConfig &planner_config)
{
    return G.randomState(planner_config);
}




PRM::PRM() {}

PRM::~PRM() {}


bool PRM::calculateDirectAction(const State &s_start, const State &s_goal, StateActionResult &result, const PlannerConfig &planner_config)
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
        if (isValidStateActionPair(s_start, result.a_new, result, planner_config))
        {
            return true;
        }
    }

    return false;
}

bool PRM::sampleLeapAction(const State &s_start, const State &s_goal, StateActionResult &result, const PlannerConfig &planner_config)
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
            bool is_valid = isValidStateActionPair(s_start, a_test, result, planner_config);
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

// /* void PRM::connectNodes(PRM_PlannerClass &G, const PlannerConfig &planner_config) {
//     // Implement logic to connect nodes in the graph
//     // Placeholder implementation
// } */


// Action PRM::getAction(int from_node, int to_node) {
//     auto it = actions.find({from_node, to_node});
//     if (it != actions.end()) {
//         return it->second;
//     } else {
//         throw std::runtime_error("Action not found between nodes.");
//     }
// }
// State PRM::getState(int node_index) {
//     if (node_index >= 0 && node_index < prm_graph.size()) {
//         return prm_graph[node_index];
//     } else {
//         throw std::out_of_range("Node index is out of range.");
//     }
// }


// std::vector<int> PRM::findPathUsingAStar(PRM_PlannerClass &G, int start, int goal) {
//     std::unordered_map<int, double> g_cost;
//     std::unordered_map<int, double> f_cost;
//     std::unordered_map<int, int> came_from;
//     std::set<std::pair<double, int>> open_set;

//     g_cost[start] = 0.0;
//     f_cost[start] = stateDistance(G.getVertex(start), G.getVertex(goal));
//     open_set.insert({f_cost[start], start});

//     while (!open_set.empty()) {
//         int current = open_set.begin()->second;
//         open_set.erase(open_set.begin());

//         if (current == goal) {
//             std::vector<int> path;
//             while (current != start) {
//                 path.push_back(current);
//                 current = came_from[current];
//             }
//             path.push_back(start);
//             std::reverse(path.begin(), path.end());
//             return path;
//         }

//         for (int neighbor : G.getNeighbors(current)) {
//             double tentative_g = g_cost[current] + G.getEdgeCost(current, neighbor);

//             if (g_cost.find(neighbor) == g_cost.end() || tentative_g < g_cost[neighbor]) {
//                 g_cost[neighbor] = tentative_g;
//                 f_cost[neighbor] = tentative_g + stateDistance(G.getVertex(neighbor), G.getVertex(goal));
//                 came_from[neighbor] = current;
//                 open_set.insert({f_cost[neighbor], neighbor});
//             }
//         }
//     }
//     return {};
// }


// Exception for goal reached



// Exception for goal reached
class AStarGoalReached : public std::exception {
public:
    const char *what() const noexcept override {
        return "Goal Reached";
    }
};

// Heuristic function
struct AStarHeuristic {
    PRM_PlannerClass &G;
    int goal;

    AStarHeuristic(PRM_PlannerClass &graph, int goal_node)
        : G(graph), goal(goal_node) {}

    double operator()(int v) const {
        return stateDistance(G.getVertex(v), G.getVertex(goal));
    }
};

// Custom visitor
class AStarVisitor : public boost::default_astar_visitor {
public:
    int goal;

    explicit AStarVisitor(int goal_node) : goal(goal_node) {}

    template <class Vertex, class Graph>
    void examine_vertex(Vertex u, Graph &) {
        if (u == goal) {
            throw AStarGoalReached();
        }
    }
};

// A* implementation
std::vector<int> PRM::findPathUsingAStar(PRM_PlannerClass &G, int start, int goal) {
    using namespace boost;

    // Define Boost graph type
    typedef adjacency_list<listS, vecS, directedS, no_property, property<edge_weight_t, double>> Graph;
    typedef graph_traits<Graph>::vertex_descriptor Vertex;

    // Create Boost graph
    Graph graph(G.getNumVertices());

    // Add edges
    for (int from = 0; from < G.getNumVertices(); ++from) {
        for (int to : G.getNeighbors(from)) {
            double weight = G.getEdgeCost(from, to);
            add_edge(from, to, weight, graph);
        }
    }

    // Predecessor and distance maps
    std::vector<Vertex> predecessors(num_vertices(graph));
    std::vector<double> distances(num_vertices(graph));

    try {
        // Run A* search
        astar_search(
            graph,
            start,
            AStarHeuristic(G, goal),
            predecessor_map(&predecessors[0])
                .distance_map(&distances[0])
                .visitor(AStarVisitor(goal)));
    } catch (AStarGoalReached &) {
        // Reconstruct the path from the predecessors
        std::vector<int> path;
        for (Vertex v = goal; v != start; v = predecessors[v]) {
            path.push_back(v);
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }

    // If no path is found, return an empty vector
    return {};
}


void PRM::buildRoadmap(PRM_PlannerClass &G, const PlannerConfig &planner_config, const int &num_samples, const double &epsilon) {
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

        std::vector<int> neighbors = G.neighborhoodDist(random_state, epsilon);
        if (neighbors.empty()) continue;

        for (int neighbor : neighbors) {
            State neighbor_state = G.getVertex(neighbor);
            if (calculateDirectAction(neighbor_state, random_state, result, planner_config)) {

                G.addEdge(s_new_index, neighbor, 0.0);
                G.actions[{neighbor, s_new_index}] = result.a_new;
            }
            if (calculateDirectAction(random_state, neighbor_state, result, planner_config)) {

                G.addEdge(neighbor, s_new_index, 0.0);
                G.actions[{s_new_index, neighbor}] = result.a_new;
            }
        }
    }
}

Action PRM::findAction(PRM_PlannerClass &G, int from_node, int to_node) {
    auto it = G.actions.find({from_node, to_node});
    if (it != G.actions.end()) {
        return it->second;
    } else {
        throw std::runtime_error("Action not found between nodes.");
    }
}


State PRM::findState(PRM_PlannerClass &G, int node_index) {
    if (node_index >= 0 && node_index < G.getNumVertices()) {
        return G.getVertex(node_index);
    } else {
        throw std::out_of_range("Node index is out of range.");
    }
}

