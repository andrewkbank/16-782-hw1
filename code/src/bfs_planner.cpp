#include "../include/bfs_planner.h"
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

struct BFSNode {
    int x;
    int y;
    int cost;
    int time_steps;
    int parent_idx;
};

struct PathWaypoint {
    int x;
    int y;
};

// Static cache to store planned trajectory across time steps
static std::vector<PathWaypoint> cached_path;
static size_t cached_step_idx = 0;

void bfs_planner(
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
    // If robot is already at the target position at current time
    if (robotposeX == targetposeX && robotposeY == targetposeY)
    {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        cached_path.clear();
        cached_step_idx = 0;
        return;
    }

    // Reset cache if starting a new simulation run
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

        // Verify valid 8-connected transition or wait (dx <= 1, dy <= 1) and collision-free
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

        // Cache was invalidated -> clear and re-plan
        cached_path.clear();
        cached_step_idx = 0;
    }

    // --- FULL BFS SEARCH ---
    auto start_time = std::chrono::high_resolution_clock::now();

    // 1. Build lookup for target position at each discrete time step: time -> map_index
    std::unordered_map<int, int> target_map_at_time;
    target_map_at_time.reserve(target_steps - curr_time);
    for (int t = curr_time; t < target_steps; t++)
    {
        int tx = target_traj[t];
        int ty = target_traj[t + target_steps];
        target_map_at_time[t] = GETMAPINDEX(tx, ty, x_size, y_size);
    }

    // 8-connected grid offsets
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    // Min-priority queue ordered by cumulative path cost
    // Pair: <cumulative_cost, node_index_in_pool>
    typedef std::pair<int, int> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;

    // Node pool storing all generated search nodes
    std::vector<BFSNode> node_pool;
    node_pool.reserve(100000);

    // Track best cost and best time seen per cell for pruning
    std::vector<int> min_cost(x_size * y_size, INT_MAX);
    std::vector<int> min_time(x_size * y_size, INT_MAX);

    // Push starting node (cost = 0, time_steps = 0)
    node_pool.push_back({robotposeX, robotposeY, 0, 0, -1});
    pq.push({0, 0});
    int start_idx = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
    min_cost[start_idx] = 0;
    min_time[start_idx] = 0;

    int found_goal_idx = -1;
    int target_intercept_time = -1;
    int nodes_expanded = 0;

    while (!pq.empty())
    {
        PQElement top = pq.top();
        pq.pop();

        int curr_cost = top.first;
        int curr_idx = top.second;
        BFSNode curr_node = node_pool[curr_idx];

        int map_idx = GETMAPINDEX(curr_node.x, curr_node.y, x_size, y_size);

        // Prune if this state is strictly dominated
        if (curr_cost > min_cost[map_idx] && curr_node.time_steps > min_time[map_idx])
        {
            continue;
        }

        nodes_expanded++;

        // Termination Check: does this node reach the EXACT position of the target at the EXACT same time?
        int arrival_time = curr_time + curr_node.time_steps;
        if (arrival_time < target_steps)
        {
            auto it = target_map_at_time.find(arrival_time);
            if (it != target_map_at_time.end() && it->second == map_idx)
            {
                // Exact space-time match found!
                target_intercept_time = arrival_time;
                found_goal_idx = curr_idx;
                break;
            }
        }

        // Don't expand beyond max simulation steps
        if (arrival_time + 1 >= target_steps)
        {
            continue;
        }

        // Expand 8-connected neighbors
        for (int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int nextX = curr_node.x + dX[dir];
            int nextY = curr_node.y + dY[dir];

            // Bounds check (1-indexed)
            if (nextX >= 1 && nextX <= x_size && nextY >= 1 && nextY <= y_size)
            {
                int next_map_idx = GETMAPINDEX(nextX, nextY, x_size, y_size);
                int cell_cost = map[next_map_idx];

                // Collision check
                if (cell_cost >= 0 && cell_cost < collision_thresh)
                {
                    int next_cost = curr_cost + cell_cost;
                    int next_time = curr_node.time_steps + 1;

                    // Prune if strictly dominated in both cost and time
                    if (next_cost >= min_cost[next_map_idx] && next_time >= min_time[next_map_idx])
                    {
                        continue;
                    }

                    // Update best seen metrics
                    if (next_cost < min_cost[next_map_idx]) min_cost[next_map_idx] = next_cost;
                    if (next_time < min_time[next_map_idx]) min_time[next_map_idx] = next_time;

                    // Add new node to pool
                    int new_node_idx = static_cast<int>(node_pool.size());
                    node_pool.push_back({nextX, nextY, next_cost, next_time, curr_idx});
                    pq.push({next_cost, new_node_idx});
                }
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    // Reconstruct full path
    if (found_goal_idx != -1)
    {
        std::vector<PathWaypoint> travel_path;
        int trace_idx = found_goal_idx;
        while (trace_idx != 0 && trace_idx != -1)
        {
            travel_path.push_back({node_pool[trace_idx].x, node_pool[trace_idx].y});
            trace_idx = node_pool[trace_idx].parent_idx;
        }
        std::reverse(travel_path.begin(), travel_path.end());

        int robot_arrival_time = curr_time + node_pool[found_goal_idx].time_steps;
        PathWaypoint intercept_pos = {node_pool[found_goal_idx].x, node_pool[found_goal_idx].y};

        if (travel_path.empty())
        {
            // Robot is already at the target position at current time
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            cached_path.clear();
            cached_step_idx = 0;
        }
        else
        {
            // Execute first step and cache the rest
            action_ptr[0] = travel_path[0].x;
            action_ptr[1] = travel_path[0].y;

            cached_path = std::move(travel_path);
            cached_step_idx = 1;
        }

        std::cout << "[bfs_planner] t=" << curr_time 
                  << " | FULL SEARCH expanded=" << nodes_expanded 
                  << " | search_time=" << duration_ms << "ms"
                  << " | intercept_pt=(" << intercept_pos.x << "," << intercept_pos.y << ")"
                  << " | intercept_time=" << target_intercept_time
                  << " | travel_cost=" << node_pool[found_goal_idx].cost 
                  << " | total_plan_length=" << cached_path.size()
                  << " | next_action=(" << action_ptr[0] << "," << action_ptr[1] << ")"
                  << std::endl;
    }
    else
    {
        // Fallback: stay in place if no interception path found
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        cached_path.clear();
        cached_step_idx = 0;

        std::cout << "[bfs_planner] t=" << curr_time 
                  << " | FULL SEARCH expanded=" << nodes_expanded 
                  << " | search_time=" << duration_ms << "ms"
                  << " | NO PATH FOUND -> staying in place"
                  << std::endl;
    }
}