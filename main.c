#include <math.h>
#include <stdio.h>

#include "config.h"
#include "hw.h"
#include "map.h"
#include "path.h"

pose robot_pose = {0};
const unicycle control_limits = {0.1, 0.1};
map interest_map;
static int t = 0;
static const int t_lim = 2000;

static pose default_path[4] = {
  {0.1, 0.1, M_PI/4.0},
  {0.2, 0.2, -M_PI},
  {0.3, 0.1, -3.0*M_PI/4.0},
  {0, 0, 0}
};
const size_t num_waypoints = 4;
waypoint *path[4];

waypoint *curr_waypoint;

static void init() {
  interest_map_init(&interest_map, INTEREST_MAP_NROWS, INTEREST_MAP_NCOLS, INTEREST_MAP_DX, INTEREST_MAP_DY);
  // init_hw();

  for (size_t i = 0; i < num_waypoints; ++i) {
    path[i] = waypoint_init(default_path[i], 0, 0);
  }

  for (size_t i = 0; i < num_waypoints-1; ++i) {
    path[i]->next = path[i+1];
  }
  for (size_t i = 1; i < num_waypoints; ++i) {
    path[i]->prev = path[i-1];
  }
  path[num_waypoints-1]->next = path[0];
  path[0]->prev = path[num_waypoints-1];

  for (size_t i = 0; i < num_waypoints; ++i) {
    printf("Waypoint %d: (%.3f, %3.f)\n", i, path[i]->target.x, path[i]->target.y);
  }

  curr_waypoint = path[0];
}

int main(void) {
  init();

  while (t < t_lim) {
    if (curr_waypoint) {
      // go to next waypoint

      // calculate [v, omega] from pose to waypoint
      unicycle controls = get_controls(robot_pose, curr_waypoint->target, control_limits);

      // calculate [vr, vl], send to motor controller
      // set_motors(controls, ROBOT_BODY_WIDTH, ROBOT_WHEEL_RADIUS);

      // update state estimate
      robot_pose = update_state(robot_pose, controls, TIMESTEP);
      if (t % 100 == 0) {
        printf("t=%d, v=%.3f, w = %.3f, pose: (%.3f, %.3f) @ %.3f\n", t, controls.v, controls.w, robot_pose.x, robot_pose.y, robot_pose.theta);
      }

      // update occupancy map

      // update interest map
      interest_map_update(&interest_map, robot_pose, INTEREST_MAP_MAX, INTEREST_MAP_DECAY);
      // map_print(&interest_map);

      // check if near a waypoint, update waypoints
      if (position_near(robot_pose, curr_waypoint->target, WAYPOINT_NEAR_POS_TOL)) {
        pose curr_pose = curr_waypoint->target;
        curr_waypoint = curr_waypoint->next;       
        printf("Reached (%3f, %3f) @ %3f. (%3f, %3f) @ %3f\nHeading to (%3f, %3f) @ %3f\n",
          curr_pose.x,
          curr_pose.y,
          curr_pose.theta,
          robot_pose.x,
          robot_pose.y,
          robot_pose.theta,
          curr_waypoint->target.x,
          curr_waypoint->target.y,
          curr_waypoint->target.theta);
      }
    }
    else {
      // Pick pose based on interest in the map
    }

    ++t;
    // t += TIMESTEP;
  }
}