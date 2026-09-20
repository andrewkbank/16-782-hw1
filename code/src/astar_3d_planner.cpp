/*=================================================================
 *
 * astar_3d_planner.cpp
 *
 * Fast Hybrid Space-Time Planner with Trajectory Intersection Checks,
 * Outpost Wait Offloading, and Time-Budgeted Search.
 *
 *=================================================================*/
#include "../include/astar_3d_planner.h"
#include <iostream>
#include <vector>
#include <queue>
#include <chrono>
#include <cmath>
#include <climits>
#include <algorithm>
#include <unordered_map>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (((Y)-1)*(XSIZE) + ((X)-1))
#define NUMOFDIRS 8

struct AStarNode {
    int x;
    int y;
    int t;              // Absolute time step robot arrives at (x, y)
    int g_cost;         // Path cost to reach (x, y) at time t
    int f_cost;         // g_cost + heuristic
    int parent_idx;
};

struct PathWaypoint {
    int x;
    int y;
};

static std::vector<PathWaypoint> cached_path;
static size_t cached_step_idx = 0;

// Strictly admissible & consistent heuristic for 8-connected maps with non-uniform costs
static inline int calculate_heuristic(
    int x, int y, int t, 
    int target_steps, const int* target_traj, 
    int min_map_cost)
{
    int min_h = INT_MAX;

    // Evaluate all future target positions
    for (int tau = t; tau < target_steps; tau++)
    {
        int tx = target_traj[tau];
        int ty = target_traj[tau + target_steps];

        int spatial_dist = std::max(std::abs(x - tx), std::abs(y - ty));
        int time_available = tau - t;

        // The robot can physically reach this target point in time
        if (spatial_dist <= time_available)
        {
            int lower_bound_cost = spatial_dist * min_map_cost;
            if (lower_bound_cost < min_h)
            {
                min_h = lower_bound_cost;
            }
        }
    }

    // If min_h is INT_MAX, the target is physically unreachable from (x,y,t) 
    // before the simulation ends. Do NOT return 0 (which breaks consistency).
    // Return a sufficiently large number to prune this dead-end safely.
    // (We use 999999 to avoid integer overflow when added to g_cost)
    return (min_h == INT_MAX) ? 999999 : min_h;
}

// Outpost evaluation: check if waiting at a nearby cheap tile yields a lower overall cost
static bool evaluate_outpost_detour(
    int startX, int startY,
    int slack_steps, int current_best_cost,
    int* map, int collision_thresh,
    int x_size, int y_size,
    PathWaypoint& out_outpost_pos)
{
    if (slack_steps <= 0) return false;

    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    int start_cell_cost = map[GETMAPINDEX(startX, startY, x_size, y_size)];
    int best_saving = 0;
    bool found_detour = false;

    for (int dir = 0; dir < NUMOFDIRS; dir++)
    {
        int candidateX = startX + dX[dir];
        int candidateY = startY + dY[dir];

        if (candidateX >= 1 && candidateX <= x_size && candidateY >= 1 && candidateY <= y_size)
        {
            int c_idx = GETMAPINDEX(candidateX, candidateY, x_size, y_size);
            int candidate_cost = map[c_idx];

            if (candidate_cost >= 0 && candidate_cost < collision_thresh)
            {
                int cost_diff = start_cell_cost - candidate_cost;
                if (cost_diff > 0)
                {
                    int total_saving = (cost_diff * slack_steps) - (2 * candidate_cost);
                    if (total_saving > best_saving)
                    {
                        best_saving = total_saving;
                        out_outpost_pos = {candidateX, candidateY};
                        found_detour = true;
                    }
                }
            }
        }
    }
    return found_detour;
}

void astar_3d_planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )
{
    if (robotposeX == targetposeX && robotposeY == targetposeY)
    {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        cached_path.clear();
        cached_step_idx = 0;
        return;
    }

    if (curr_time == 0)
    {
        cached_path.clear();
        cached_step_idx = 0;
    }

    // Follow cached trajectory if valid
    if (!cached_path.empty() && cached_step_idx < cached_path.size())
    {
        PathWaypoint next_wp = cached_path[cached_step_idx];
        int dx = std::abs(next_wp.x - robotposeX);
        int dy = std::abs(next_wp.y - robotposeY);

        if (dx <= 1 && dy <= 1)
        {
            int next_map_idx = GETMAPINDEX(next_wp.x, next_wp.y, x_size, y_size);
            if (map[next_map_idx] >= 0 && map[next_map_idx] < collision_thresh)
            {
                action_ptr[0] = next_wp.x;
                action_ptr[1] = next_wp.y;
                cached_step_idx++;
                return;
            }
        }
        cached_path.clear();
        cached_step_idx = 0;
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    const double TIME_BUDGET_MS = 900.0;

    int min_map_cost = INT_MAX;
    int total_cells = x_size * y_size;
    for (int i = 0; i < total_cells; i++)
    {
        if (map[i] >= 0 && map[i] < collision_thresh)
        {
            if (map[i] < min_map_cost) min_map_cost = map[i];
        }
    }
    if (min_map_cost == INT_MAX || min_map_cost == 0) min_map_cost = 1;

    // --- PRE-COMPUTE TARGET TRAJECTORY INTERSECTIONS ---
    // Maps cell index -> vector of time steps tau when the target visits this cell
    std::vector<std::vector<int>> target_visit_times(total_cells);
    for (int tau = 0; tau < target_steps; tau++)
    {
        int tx = target_traj[tau];
        int ty = target_traj[tau + target_steps];
        if (tx >= 1 && tx <= x_size && ty >= 1 && ty <= y_size)
        {
            int t_idx = GETMAPINDEX(tx, ty, x_size, y_size);
            target_visit_times[t_idx].push_back(tau);
        }
    }

    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    typedef std::pair<int, int> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;

    std::vector<AStarNode> node_pool;
    node_pool.reserve(50000);

    std::vector<int> min_cost_2d(total_cells, INT_MAX);

    int start_map_idx = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
    int start_h = calculate_heuristic(robotposeX, robotposeY, curr_time, target_steps, target_traj, min_map_cost);

    node_pool.push_back({robotposeX, robotposeY, curr_time, 0, start_h, -1});
    pq.push({start_h, 0});
    min_cost_2d[start_map_idx] = 0;

    int best_intercept_node_idx = -1;
    int best_intercept_tau = -1;
    int best_total_cost = INT_MAX;
    int nodes_expanded = 0;

    while (!pq.empty())
    {
        // Enforce time budget limit once at least one candidate interception is found
        if (best_intercept_node_idx != -1)
        {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - start_time).count();
            if (elapsed >= TIME_BUDGET_MS) break;
        }

        PQElement top = pq.top();
        pq.pop();

        int curr_idx = top.second;
        AStarNode curr_node = node_pool[curr_idx];
        int curr_map_idx = GETMAPINDEX(curr_node.x, curr_node.y, x_size, y_size);

        if (curr_node.g_cost > min_cost_2d[curr_map_idx]) continue;
        nodes_expanded++;

        // --- TRAJECTORY INTERSECTION INTERCEPTION CHECK ---
        // Check if the target ever passes through (curr_node.x, curr_node.y) at tau >= curr_node.t
        const auto& visits = target_visit_times[curr_map_idx];
        for (int tau : visits)
        {
            if (tau >= curr_node.t)
            {
                int wait_duration = tau - curr_node.t;
                int wait_cost = wait_duration * map[curr_map_idx];
                int total_candidate_cost = curr_node.g_cost + wait_cost;

                if (total_candidate_cost < best_total_cost)
                {
                    best_total_cost = total_candidate_cost;
                    best_intercept_node_idx = curr_idx;
                    best_intercept_tau = tau;
                }
            }
        }

        if (curr_node.t + 1 >= target_steps) continue;
        int next_t = curr_node.t + 1;

        // Expand spatial 2D neighbors
        for (int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int nextX = curr_node.x + dX[dir];
            int nextY = curr_node.y + dY[dir];

            if (nextX >= 1 && nextX <= x_size && nextY >= 1 && nextY <= y_size)
            {
                int next_map_idx = GETMAPINDEX(nextX, nextY, x_size, y_size);
                int cell_cost = map[next_map_idx];

                if (cell_cost >= 0 && cell_cost < collision_thresh)
                {
                    int next_g = curr_node.g_cost + cell_cost;

                    if (next_g < min_cost_2d[next_map_idx])
                    {
                        min_cost_2d[next_map_idx] = next_g;
                        int h = calculate_heuristic(nextX, nextY, next_t, target_steps, target_traj, min_map_cost);
                        int next_f = next_g + h;

                        int new_node_idx = static_cast<int>(node_pool.size());
                        node_pool.push_back({nextX, nextY, next_t, next_g, next_f, curr_idx});
                        pq.push({next_f, new_node_idx});
                    }
                }
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    bool hit_time_limit = (duration_ms >= TIME_BUDGET_MS);

    // --- RECONSTRUCT PATH AND APPLY OUTPOST OPTIMIZATION ---
    if (best_intercept_node_idx != -1)
    {
        std::vector<PathWaypoint> travel_path;
        int trace_idx = best_intercept_node_idx;

        while (trace_idx != 0 && trace_idx != -1)
        {
            travel_path.push_back({node_pool[trace_idx].x, node_pool[trace_idx].y});
            trace_idx = node_pool[trace_idx].parent_idx;
        }
        std::reverse(travel_path.begin(), travel_path.end());

        int arrival_time = node_pool[best_intercept_node_idx].t;
        int slack_steps = best_intercept_tau - arrival_time;

        PathWaypoint outpost_pos;
        if (slack_steps > 0 && evaluate_outpost_detour(
                robotposeX, robotposeY,
                slack_steps, best_total_cost, map, collision_thresh, x_size, y_size, outpost_pos))
        {
            travel_path.insert(travel_path.begin(), outpost_pos);
        }
        else if (slack_steps > 0)
        {
            // NEW: Pad wait time at the INTERCEPT cell, not the start cell
            PathWaypoint intercept_pos = travel_path.back();
            for (int w = 0; w < slack_steps; w++)
            {
                travel_path.push_back(intercept_pos);
            }
        }

        action_ptr[0] = travel_path[0].x;
        action_ptr[1] = travel_path[0].y;

        // NEW: If we hit the time budget, do not cache. Force replanning next step.
        if (hit_time_limit)
        {
            cached_path.clear();
            cached_step_idx = 0;
        }
        else
        {
            cached_path = std::move(travel_path);
            cached_step_idx = 1;
        }

        std::cout << "[hybrid_astar_3d_planner] t=" << curr_time 
                  << " | HYBRID A* expanded=" << nodes_expanded 
                  << " | search_time=" << duration_ms << "ms"
                  << (hit_time_limit ? " (TIME LIMIT)" : "")
                  << " | intercept_pt=(" << node_pool[best_intercept_node_idx].x << "," << node_pool[best_intercept_node_idx].y << ")"
                  << " | intercept_tau=" << best_intercept_tau
                  << " | next_action=(" << action_ptr[0] << "," << action_ptr[1] << ")"
                  << std::endl;
    }
    else
    {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        cached_path.clear();
        cached_step_idx = 0;

        std::cout << "[hybrid_astar_3d_planner] t=" << curr_time 
                  << " | HYBRID A* expanded=" << nodes_expanded 
                  << " | search_time=" << duration_ms << "ms"
                  << " | NO PATH FOUND -> staying in place"
                  << std::endl;
    }
}