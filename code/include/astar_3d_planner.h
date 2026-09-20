#ifndef ASTAR_3D_PLANNER_H
#define ASTAR_3D_PLANNER_H

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
);

#endif // ASTAR_3D_PLANNER_H
