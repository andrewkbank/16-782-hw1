/*=================================================================
 *
 * hybrid_astar_3d_planner.cpp
 *
 * Anytime Space-Time Planner:
 * 1. O(1) Precomputed Heuristic Map
 * 2. Fast 2D-Bounded A* (Finds fast upper-bound cost)
 * 3. Thorough Pareto 3D A* (Optimizes using outposts & branch/bound)
 *
 *=================================================================*/
#include "../include/astar_3d_planner.h"
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <chrono>
#include <cmath>
#include <climits>
#include <algorithm>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (((Y)-1)*(XSIZE) + ((X)-1))
#define NUMOFDIRS 8

// --- DATA STRUCTURES ---
struct FastNode {
    int x, y, cost, time_steps, parent_idx;
};

struct ThoroughNode {
    int x, y, t, g_cost, min_path_cost, parent_idx;
};

struct PQElement {
    int f, g, idx;
};

struct ComparePQ {
    bool operator()(const PQElement& a, const PQElement& b) const {
        if (a.f == b.f) return a.g < b.g; 
        return a.f > b.f;
    }
};

struct PathWaypoint {
    int x, y;
};

struct ParetoFront {
    int g1, m1;
    int g2, m2;
};

// --- GLOBALS & CACHES ---
static std::vector<PathWaypoint> cached_path;
static size_t cached_step_idx = 0;
static std::vector<int> h_grid;
static bool h_grid_initialized = false;
const double TIME_BUDGET_MS = 100.0; // Max time allowed per turn

void astar_3d_planner(
    int* map, int collision_thresh, int x_size, int y_size,
    int robotposeX, int robotposeY, int target_steps,
    int* target_traj, int targetposeX, int targetposeY,
    int curr_time, int* action_ptr)
{
    // 0. Immediate Intercept & Caching Logic
    if (robotposeX == targetposeX && robotposeY == targetposeY) {
        action_ptr[0] = robotposeX; action_ptr[1] = robotposeY;
        cached_path.clear(); cached_step_idx = 0;
        return;
    }

    if (curr_time == 0) {
        cached_path.clear(); cached_step_idx = 0;
    }

    // Follow Cached Path if Valid
    if (!cached_path.empty() && cached_step_idx < cached_path.size()) {
        PathWaypoint next_wp = cached_path[cached_step_idx];
        // Ensure next step is adjacent and valid
        if (std::abs(next_wp.x - robotposeX) <= 1 && std::abs(next_wp.y - robotposeY) <= 1) {
            int next_map_idx = GETMAPINDEX(next_wp.x, next_wp.y, x_size, y_size);
            if (map[next_map_idx] >= 0 && map[next_map_idx] < collision_thresh) {
                action_ptr[0] = next_wp.x; action_ptr[1] = next_wp.y;
                cached_step_idx++;
                return;
            }
        }
        // Cache invalidated (e.g., obstacle appeared), clear it
        cached_path.clear(); cached_step_idx = 0;
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    int total_cells = x_size * y_size;

    // --- 1. O(1) HEURISTIC PRECOMPUTATION ---
    if (curr_time == 0 || !h_grid_initialized || h_grid.size() != (size_t)total_cells) {
        h_grid.assign(total_cells, 999999);
        std::vector<int> bfs_queue;
        bfs_queue.reserve(total_cells);
        
        for (int tau = 0; tau < target_steps; tau++) {
            int tx = target_traj[tau], ty = target_traj[tau + target_steps];
            if (tx >= 1 && tx <= x_size && ty >= 1 && ty <= y_size) {
                int idx = GETMAPINDEX(tx, ty, x_size, y_size);
                if (h_grid[idx] != 0) {
                    h_grid[idx] = 0; bfs_queue.push_back(idx);
                }
            }
        }

        int head = 0;
        int dX8[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
        int dY8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
        
        while (head < (int)bfs_queue.size()) {
            int curr = bfs_queue[head++];
            int cx = (curr % x_size) + 1, cy = (curr / x_size) + 1;
            int current_dist = h_grid[curr];

            for (int dir = 0; dir < 8; dir++) {
                int nx = cx + dX8[dir], ny = cy + dY8[dir];
                if (nx >= 1 && nx <= x_size && ny >= 1 && ny <= y_size) {
                    int nidx = GETMAPINDEX(nx, ny, x_size, y_size);
                    if (h_grid[nidx] > current_dist + 1) {
                        h_grid[nidx] = current_dist + 1; bfs_queue.push_back(nidx);
                    }
                }
            }
        }
        h_grid_initialized = true;
    }

    int min_map_cost = INT_MAX;
    for (int i = 0; i < total_cells; i++) {
        if (map[i] >= 0 && map[i] < collision_thresh && map[i] < min_map_cost) min_map_cost = map[i];
    }
    if (min_map_cost == INT_MAX) min_map_cost = 0;

    // Fast lookups for target trajectory
    std::unordered_map<int, int> target_map_at_time;
    std::vector<std::vector<int>> target_visit_times(total_cells);
    target_map_at_time.reserve(target_steps - curr_time);
    
    for (int t = 0; t < target_steps; t++) {
        int tx = target_traj[t], ty = target_traj[t + target_steps];
        if (tx >= 1 && tx <= x_size && ty >= 1 && ty <= y_size) {
            int idx = GETMAPINDEX(tx, ty, x_size, y_size);
            if (t >= curr_time) target_map_at_time[t] = idx;
            target_visit_times[idx].push_back(t);
        }
    }

    int dX[NUMOFDIRS] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dY[NUMOFDIRS] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int start_idx = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
    int start_cell_cost = std::max(1, map[start_idx]);
    int start_spatial_dist = h_grid[start_idx];
    int start_max_time_avail = (target_steps - 1) - curr_time;
    int start_h = (start_spatial_dist <= start_max_time_avail) ? (start_spatial_dist * min_map_cost) : 999999;

    // --- 2. PHASE 1: FAST 2D-BOUNDED A* ---
    std::priority_queue<PQElement, std::vector<PQElement>, ComparePQ> fast_pq;
    std::vector<FastNode> fast_pool;
    fast_pool.reserve(50000);
    std::vector<int> min_cost(total_cells, INT_MAX);
    std::vector<int> min_time(total_cells, INT_MAX);

    fast_pool.push_back({robotposeX, robotposeY, 0, 0, -1});
    fast_pq.push({start_h, 0, 0});
    min_cost[start_idx] = 0; min_time[start_idx] = 0;

    int fast_goal_idx = -1;
    int best_total_cost = INT_MAX; // Upper bound
    int fast_nodes = 0;

    while (!fast_pq.empty()) {
        PQElement top = fast_pq.top(); fast_pq.pop();
        int curr_cost = top.g, curr_idx = top.idx;
        FastNode curr_node = fast_pool[curr_idx];
        int map_idx = GETMAPINDEX(curr_node.x, curr_node.y, x_size, y_size);

        if (curr_cost > min_cost[map_idx] && curr_node.time_steps > min_time[map_idx]) continue;
        fast_nodes++;
        int arrival_time = curr_time + curr_node.time_steps;

        if (arrival_time < target_steps) {
            auto it = target_map_at_time.find(arrival_time);
            if (it != target_map_at_time.end() && it->second == map_idx) {
                fast_goal_idx = curr_idx;
                best_total_cost = curr_cost;
                break;
            }
        }

        if (arrival_time + 1 >= target_steps) continue;

        for (int dir = 0; dir < NUMOFDIRS; dir++) {
            int nextX = curr_node.x + dX[dir], nextY = curr_node.y + dY[dir];
            if (nextX >= 1 && nextX <= x_size && nextY >= 1 && nextY <= y_size) {
                int nidx = GETMAPINDEX(nextX, nextY, x_size, y_size);
                int cell_cost = map[nidx];
                if (cell_cost >= 0 && cell_cost < collision_thresh) {
                    int next_cost = curr_cost + cell_cost, next_time = curr_node.time_steps + 1;
                    if (next_cost >= min_cost[nidx] && next_time >= min_time[nidx]) continue;
                    
                    if (next_cost < min_cost[nidx]) min_cost[nidx] = next_cost;
                    if (next_time < min_time[nidx]) min_time[nidx] = next_time;

                    int sd = h_grid[nidx];
                    int h = (sd <= (target_steps - 1) - (curr_time + next_time)) ? (sd * min_map_cost) : 999999;
                    fast_pool.push_back({nextX, nextY, next_cost, next_time, curr_idx});
                    fast_pq.push({next_cost + h, next_cost, (int)fast_pool.size() - 1});
                }
            }
        }
    }

    // --- 3. PHASE 2: THOROUGH PARETO 3D A* ---
    std::priority_queue<PQElement, std::vector<PQElement>, ComparePQ> thorough_pq;
    std::vector<ThoroughNode> thorough_pool;
    thorough_pool.reserve(100000);
    std::vector<ParetoFront> visited(total_cells, {INT_MAX, INT_MAX, INT_MAX, INT_MAX});

    thorough_pool.push_back({robotposeX, robotposeY, curr_time, 0, start_cell_cost, -1});
    thorough_pq.push({start_h, 0, 0});
    visited[start_idx].g1 = 0; visited[start_idx].m1 = start_cell_cost;

    int thorough_goal_idx = -1;
    int best_intercept_tau = -1;
    int thorough_nodes = 0;
    bool hit_time_limit = false;

    // Only run Thorough A* if Fast A* succeeded. (If Fast failed, no valid path exists anyway)
    if (fast_goal_idx != -1) {
        while (!thorough_pq.empty()) {
            // Check time every 1024 nodes
            if ((thorough_nodes & 1023) == 0) { 
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::high_resolution_clock::now() - start_time).count();
                if (elapsed >= TIME_BUDGET_MS) { hit_time_limit = true; break; }
            }

            PQElement top = thorough_pq.top(); thorough_pq.pop();
            if (top.f >= best_total_cost) break; // Branch and Bound prune!

            int curr_idx = top.idx;
            ThoroughNode curr_node = thorough_pool[curr_idx];
            int map_idx = GETMAPINDEX(curr_node.x, curr_node.y, x_size, y_size);
            thorough_nodes++;

            const auto& visits = target_visit_times[map_idx];
            for (int tau : visits) {
                if (tau >= curr_node.t) {
                    int wait_cost = (tau - curr_node.t) * curr_node.min_path_cost;
                    int total_cand_cost = curr_node.g_cost + wait_cost;
                    if (total_cand_cost < best_total_cost) {
                        best_total_cost = total_cand_cost;
                        thorough_goal_idx = curr_idx;
                        best_intercept_tau = tau;
                    }
                }
            }

            if (curr_node.t + 1 >= target_steps) continue;
            int next_t = curr_node.t + 1;

            for (int dir = 0; dir < NUMOFDIRS; dir++) {
                int nextX = curr_node.x + dX[dir], nextY = curr_node.y + dY[dir];
                if (nextX >= 1 && nextX <= x_size && nextY >= 1 && nextY <= y_size) {
                    int nidx = GETMAPINDEX(nextX, nextY, x_size, y_size);
                    int cell_cost = map[nidx];

                    if (cell_cost >= 0 && cell_cost < collision_thresh) {
                        int next_g = curr_node.g_cost + cell_cost;
                        int next_min_path = std::min(curr_node.min_path_cost, cell_cost);

                        int sd = h_grid[nidx];
                        int h = (sd <= (target_steps - 1) - next_t) ? (sd * min_map_cost) : 999999;
                        int next_f = next_g + h;
                        
                        if (next_f >= best_total_cost) continue;

                        bool dominated = false;
                        auto& p = visited[nidx];
                        if ((p.g1 <= next_g && p.m1 <= next_min_path) || 
                            (p.g2 <= next_g && p.m2 <= next_min_path)) dominated = true;

                        if (!dominated) {
                            if (next_g <= p.g1 && next_min_path <= p.m1) { p.g1 = next_g; p.m1 = next_min_path; }
                            else if (next_g <= p.g2 && next_min_path <= p.m2) { p.g2 = next_g; p.m2 = next_min_path; }
                            else if (p.g1 == INT_MAX) { p.g1 = next_g; p.m1 = next_min_path; }
                            else if (p.g2 == INT_MAX) { p.g2 = next_g; p.m2 = next_min_path; }
                            else {
                                if (p.g1 > p.g2) { p.g1 = next_g; p.m1 = next_min_path; }
                                else { p.g2 = next_g; p.m2 = next_min_path; }
                            }

                            thorough_pool.push_back({nextX, nextY, next_t, next_g, next_min_path, curr_idx});
                            thorough_pq.push({next_f, next_g, (int)thorough_pool.size() - 1});
                        }
                    }
                }
            }
        }
    }

    // --- 4. RECONSTRUCT BEST PATH ---
    std::vector<PathWaypoint> travel_path;

    if (thorough_goal_idx != -1) {
        // Optimized Thorough Path
        int trace_idx = thorough_goal_idx;
        while (trace_idx != 0 && trace_idx != -1) {  // Stop before start node (idx 0)
            travel_path.push_back({thorough_pool[trace_idx].x, thorough_pool[trace_idx].y});
            trace_idx = thorough_pool[trace_idx].parent_idx;
        }
        std::reverse(travel_path.begin(), travel_path.end());

        int arrival_time = thorough_pool[thorough_goal_idx].t;
        int slack = best_intercept_tau - arrival_time;
        int t_min_cost = thorough_pool[thorough_goal_idx].min_path_cost;

        if (slack > 0 && !travel_path.empty()) {
            auto it = travel_path.begin();
            for (; it != travel_path.end(); ++it) {
                if (map[GETMAPINDEX(it->x, it->y, x_size, y_size)] == t_min_cost) break;
            }
            if (it == travel_path.end()) it = travel_path.end() - 1; 
            travel_path.insert(it, slack, *it);
        }
    } 
    else if (fast_goal_idx != -1) {
        // Time limit hit or no better path found. Use Fast A* Fallback.
        int trace_idx = fast_goal_idx;
        while (trace_idx != 0 && trace_idx != -1) {  // Stop before start node (idx 0)
            travel_path.push_back({fast_pool[trace_idx].x, fast_pool[trace_idx].y});
            trace_idx = fast_pool[trace_idx].parent_idx;
        }
        std::reverse(travel_path.begin(), travel_path.end());
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    if (!travel_path.empty()) {
        // Since we stop before trace_idx == 0, travel_path[0] is the very first step to take!
        action_ptr[0] = travel_path[0].x; 
        action_ptr[1] = travel_path[0].y;
        
        // ALWAYS Cache the path so the robot can move!
        cached_path = std::move(travel_path);
        cached_step_idx = 1; // Since we just took step 0

        std::cout << "[anytime_astar] t=" << curr_time 
                  << " | time=" << duration_ms << "ms" << (hit_time_limit ? " [LIMIT]" : "")
                  << " | fast_nodes=" << fast_nodes << " | thorough_nodes=" << thorough_nodes
                  << " | final_cost=" << best_total_cost 
                  << " | route=" << (thorough_goal_idx != -1 ? "THOROUGH" : "FAST_FALLBACK")
                  << std::endl;
    } else {
        action_ptr[0] = robotposeX; action_ptr[1] = robotposeY;
        cached_path.clear(); cached_step_idx = 0;
    }
}