#ifndef GREEDY_PLANNER_H
#define GREEDY_PLANNER_H

void greedy_planner(
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

#endif // GREEDY_PLANNER_H
