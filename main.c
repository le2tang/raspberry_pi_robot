#include "config.h"
#include "map.h"
#include "path.h"

waypoint_t *path;
pose_t robot_pose;
const unicycle_t control_limits = {0.1, 0.1};
map_t interest_map;

void init() {
  init_interest_map(&interest_map, INTEREST_MAP_NROWS, INTEREST_MAP_NCOLS, INTEREST_MAP_DX, INTEREST_MAP_DY);
}

int main(void) {
  init();

  while (1) {
    if (path) {
      // go to next waypoint

      // calculate [v, omega] from pose to waypoint
      unicycle_t controls = get_controls(robot_pose, path->pose, control_limits);

      // calculate [vr, vl], send to motor controller
      set_motors(controls, ROBOT_BODY_WIDTH, ROBOT_WHEEL_RADIUS);

      // update state estimate
      update_state(&robot_pose, controls, TIMESTEP);
      
      // update occupancy map


      // update interest map
      update_interest_map(&interest_map, robot_pose, INTEREST_MAP_DECAY);

      // check if near a waypoint, update waypoints
      if (pose_near(robot_pose, path->pose, NEAR_WAYPOINT_TOL)) {
        next_waypoint(path);
      }
    }
    else {
      // Pick pose based on interest in the map
      ;
      set_waypoint(path);
    }
  }
}