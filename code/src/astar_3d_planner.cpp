/*=================================================================
 *
 * hybrid_astar_3d_planner.cpp
 *
 * Fast Space-Time Planner with Pareto Outpost Tracking, 
 * Branch-and-Bound Interception, and Time-Budget Handling.
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
    int t;              // Absolute time step robot arrives
    int g_cost;         // Path cost to reach (x, y)
    int min_path_cost;  // The cheapest cell cost encountered on this path so far
    int parent_idx;
};

// Priority Queue Element
struct PQElement {
    int f;
    int g;
    int idx;
};

struct ComparePQ {
    bool operator()(const PQElement& a, const PQElement& b) const {
        if (a.f == b.f) return a.g < b.g; // Tie-breaker: prefer higher g-cost
        return a.f > b.f;
    }
};

struct PathWaypoint {
    int x;
    int y;
};

// Pareto Frontier tracking for 2D map exploration
struct ParetoFront {
    int g1, m1;
    int g2, m2;
};

static std::vector<PathWaypoint> cached_path;
static size_t cached_step_idx = 0;

static inline int calculate_heuristic(
    int x, int y, int t, 
    int target_steps, const int* target_traj, 
    int min_map_cost)
{
    int min_h = INT_MAX;
    for (int tau = t; tau < target_steps; tau++)
    {
        int tx = target_traj[tau];
        int ty = target_traj[tau + target_steps];
        int spatial_dist = std::max(std::abs(x - tx), std::abs(y - ty));
        int time_available = tau - t;

        if (spatial_dist <= time_available)
        {
            int lower_bound_cost = spatial_dist * min_map_cost;
            if (lower_bound_cost < min_h) min_h = lower_bound_cost;
        }
    }
    // Penalize physically unreachable target states to maintain A* consistency
    return (min_h == INT_MAX) ? 999999 : min_h;
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
    const double TIME_BUDGET_MS = 950.0;

    int min_map_cost = INT_MAX;
    int total_cells = x_size * y_size;
    for (int i = 0; i < total_cells; i++)
    {
        if (map[i] >= 0 && map[i] < collision_thresh)
        {
            if (map[i] < min_map_cost) min_map_cost = map[i];
        }
    }
    if (min_map_cost == INT_MAX) min_map_cost = 0;

    // Fast Trajectory Mapping
    std::vector<std::vector<int>> target_visit_times(total_cells);
    for (int tau = 0; tau < target_steps; tau++)
    {
        int tx = target_traj[tau];
        int ty = target_traj[tau + target_steps];
        if (tx >= 1 && tx <= x_size && ty >= 1 && ty <= y_size)
        {
            target_visit_times[GETMAPINDEX(tx, ty, x_size, y_size)].push_back(tau);
        }
    }

    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    std::priority_queue<PQElement, std::vector<PQElement>, ComparePQ> pq;
    std::vector<AStarNode> node_pool;
    node_pool.reserve(50000);

    // Pareto frontier tracking to allow both Direct Paths and Detour Paths to coexist
    std::vector<ParetoFront> visited(total_cells, {INT_MAX, INT_MAX, INT_MAX, INT_MAX});

    int start_map_idx = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
    int start_cell_cost = std::max(1, map[start_map_idx]);
    int start_h = calculate_heuristic(robotposeX, robotposeY, curr_time, target_steps, target_traj, min_map_cost);

    node_pool.push_back({robotposeX, robotposeY, curr_time, 0, start_cell_cost, -1});
    pq.push({start_h, 0, 0});
    
    visited[start_map_idx].g1 = 0;
    visited[start_map_idx].m1 = start_cell_cost;

    int best_intercept_node_idx = -1;
    int best_intercept_tau = -1;
    int best_total_cost = INT_MAX;
    int nodes_expanded = 0;
    bool hit_time_limit = false;

    while (!pq.empty())
    {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start_time).count();
        
        if (elapsed >= TIME_BUDGET_MS && best_intercept_node_idx != -1)
        {
            hit_time_limit = true;
            break;
        }

        PQElement top = pq.top();
        pq.pop();

        // --- BRANCH AND BOUND OPTIMALITY EXIT ---
        if (best_intercept_node_idx != -1 && top.f >= best_total_cost)
        {
            break;
        }

        int curr_idx = top.idx;
        AStarNode curr_node = node_pool[curr_idx];
        int curr_map_idx = GETMAPINDEX(curr_node.x, curr_node.y, x_size, y_size);

        nodes_expanded++;

        // --- TRAJECTORY INTERCEPTION CHECK ---
        const auto& visits = target_visit_times[curr_map_idx];
        for (int tau : visits)
        {
            if (tau >= curr_node.t)
            {
                int wait_duration = tau - curr_node.t;
                // Wait cost relies on the cheapest cell encountered along this path
                int wait_cost = wait_duration * curr_node.min_path_cost;
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
                    int next_min_path = std::min(curr_node.min_path_cost, cell_cost);

                    // Pareto Dominance Check
                    bool dominated = false;
                    auto& p = visited[next_map_idx];
                    
                    if (p.g1 <= next_g && p.m1 <= next_min_path) dominated = true;
                    else if (p.g2 <= next_g && p.m2 <= next_min_path) dominated = true;

                    if (!dominated)
                    {
                        // Replace strictly worse states, empty slots, or the highest g_cost slot
                        if (next_g <= p.g1 && next_min_path <= p.m1) { p.g1 = next_g; p.m1 = next_min_path; }
                        else if (next_g <= p.g2 && next_min_path <= p.m2) { p.g2 = next_g; p.m2 = next_min_path; }
                        else if (p.g1 == INT_MAX) { p.g1 = next_g; p.m1 = next_min_path; }
                        else if (p.g2 == INT_MAX) { p.g2 = next_g; p.m2 = next_min_path; }
                        else {
                            if (p.g1 > p.g2) { p.g1 = next_g; p.m1 = next_min_path; }
                            else { p.g2 = next_g; p.m2 = next_min_path; }
                        }

                        int h = calculate_heuristic(nextX, nextY, next_t, target_steps, target_traj, min_map_cost);
                        int next_f = next_g + h;

                        int new_node_idx = static_cast<int>(node_pool.size());
                        node_pool.push_back({nextX, nextY, next_t, next_g, next_min_path, curr_idx});
                        pq.push({next_f, next_g, new_node_idx});
                    }
                }
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    // --- RECONSTRUCT PATH ---
    if (best_intercept_node_idx != -1)
    {
        std::vector<PathWaypoint> travel_path;
        int trace_idx = best_intercept_node_idx;

        while (trace_idx != -1)
        {
            travel_path.push_back({node_pool[trace_idx].x, node_pool[trace_idx].y});
            trace_idx = node_pool[trace_idx].parent_idx;
        }
        std::reverse(travel_path.begin(), travel_path.end());

        int arrival_time = node_pool[best_intercept_node_idx].t;
        int slack_steps = best_intercept_tau - arrival_time;
        int target_min_cost = node_pool[best_intercept_node_idx].min_path_cost;

        // INJECT WAITING STEPS AT THE TRUE OUTPOST
        if (slack_steps > 0)
        {
            auto it = travel_path.begin();
            // Scan path to find the exact cell that granted the min_path_cost
            for (; it != travel_path.end(); ++it)
            {
                if (map[GETMAPINDEX(it->x, it->y, x_size, y_size)] == target_min_cost) break;
            }
            if (it == travel_path.end()) it = travel_path.end() - 1; 

            // Pad wait time natively at the outpost
            PathWaypoint wait_pos = *it;
            travel_path.insert(it, slack_steps, wait_pos);
        }

        action_ptr[0] = travel_path[1].x; // index 1 is next step (index 0 is current pose)
        action_ptr[1] = travel_path[1].y;

        // Invalidate cache if time budget triggered to force continued exploration next frame
        if (hit_time_limit)
        {
            cached_path.clear();
            cached_step_idx = 0;
        }
        else
        {
            cached_path = std::move(travel_path);
            cached_step_idx = 2;
        }

        std::cout << "[hybrid_astar_3d_planner] t=" << curr_time 
                  << " | OPTIMIZED HYBRID expanded=" << nodes_expanded 
                  << " | search_time=" << duration_ms << "ms" << (hit_time_limit ? " (TIME LIMIT)" : "")
                  << " | outpost_cost=" << target_min_cost
                  << " | intercept_tau=" << best_intercept_tau
                  << " | total_cost=" << best_total_cost 
                  << " | next_action=(" << action_ptr[0] << "," << action_ptr[1] << ")"
                  << std::endl;
    }
    else
    {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        cached_path.clear();
        cached_step_idx = 0;
    }
}