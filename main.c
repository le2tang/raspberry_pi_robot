#include <math.h>

#include "config.h"
#include "map.h"
#include "path.h"

pose_t robot_pose;
const unicycle_t control_limits = {0.1, 0.1};
map_t interest_map;

static pose_t default_path[4] = {
  {0.1, 0.1, M_PI/4.0},
  {0.2, 0.2, -M_PI},
  {0.1, 0.1, -3.0*M_PI/4.0},
  {0, 0, 0}
};
const size_t num_waypoints = 4;
waypoint_t path[4];

waypoint_t *curr_waypoint;

static void init() {
  interest_map_init(&interest_map, INTEREST_MAP_NROWS, INTEREST_MAP_NCOLS, INTEREST_MAP_DX, INTEREST_MAP_DY);

  for (size_t i = 0; i < num_waypoints; ++i) {
    waypoint_init(&path[i], default_path[i], 0, 0);
  }

  for (size_t i = 0; i < num_waypoints-1; ++i) {
    path[i].next = &path[i+1];
  }
  for (size_t i = 1; i < num_waypoints; ++i) {
    path[i].prev = &path[i-1];
  }
  path[num_waypoints-1].next = &path[0];
  path[0].prev = &path[num_waypoints-1];

  curr_waypoint = &path[0];
}

int main(void) {
  init();

  while (1) {
    if (curr_waypoint) {
      // go to next waypoint

      // calculate [v, omega] from pose to waypoint
      unicycle_t controls = get_controls(robot_pose, path->pose, control_limits);

      // calculate [vr, vl], send to motor controller
      set_motors(controls, ROBOT_BODY_WIDTH, ROBOT_WHEEL_RADIUS);

      // update state estimate
      update_state(&robot_pose, controls, TIMESTEP);
      
      // update occupancy map

      // update interest map
      interest_map_update(&interest_map, robot_pose, INTEREST_MAP_MAX, INTEREST_MAP_DECAY);

      // check if near a waypoint, update waypoints
      if (pose_near(robot_pose, path->pose, WAYPOINT_NEAR_POS_TOL, WAYPOINT_NEAR_ANG_TOL)) {
        curr_waypoint = curr_waypoint->next;
      }
    }
    else {
      // Pick pose based on interest in the map
    }
  }
}