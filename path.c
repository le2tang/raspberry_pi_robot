#include "path.h"

#include <math.h>

#include "config.h"

bool map_inbounds(map_t *map, size_t row, size_t col) {
  return (row < map->nrows && col < map->ncols);
}

float map_get(map_t *map, size_t row, size_t col) {
  return map->data[row * map->ncols + col];
}

void map_set(map_t *map, size_t row, size_t col, float data) {
  map->data[row * map->ncols + col] = data;
}

pose_t rotate(pose_t pose, float angle) {
  pose_t rotated_pose = {
    cos(angle) * pose.x - sin(angle) * pose.y,
    sin(angle) * pose.x + cos(angle) * pose.y,
    pose.theta
  };
  return rotated_pose;
}

pose_t pose2map_indices(pose_t pose, map_t *map) {
  pose_t map_pose = {
    pose.x - map->pose.x,
    pose.y - map->pose.y,
    0
  };

  float rot_angle = map->pose.theta - pose.theta;
  map_pose = rotate(map_pose, rot_angle);
  
  map_pose.x /= map->xdelta;
  map_pose.y /= map->ydelta;
  return map_pose;
}

void init_interest_map(map_t *map, size_t nrows, size_t ncols, float xdelta, float ydelta) {
  map->data = malloc(sizeof(float) * nrows * ncols);
  map->nrows = nrows;
  map->ncols = ncols;
  map->xdelta = xdelta;
  map->ydelta = ydelta;
  map->pose.x = 0;
  map->pose.y = 0;
  map->pose.theta = 0;
}

void update_interest_map(map_t *map, pose_t robot_pose, float decay) {
  pose_t map_indices = pose2map_indices(robot_pose, map);

  for (int i = -1; i < 2; ++i) {
    for (int j = -1; j < 2; ++j) {
      if (map_inbounds(map, map_indices.x + i, map_indices.y + j)) {
        float current_interest = map_get(map, map_indices.x + i, map_indices.y + j);
        float new_interest = max(0, current_interest - decay);
        map_set(map, map_indices.x + i, map_indices.y + j, new_interest);
      }
    }
  }
}

float pose_distance(pose_t pose1, pose_t pose2) {
  float dx = pose1.x - pose2.x;
  float dy = pose1.y - pose2.y;
  return sqrt(dx * dx + dy * dy);
}

bool pose_near(pose_t pose1, pose_t pose2, float position_tol, float angle_tol) {
  return (abs(pose1.theta - pose2.theta) < angle_tol) && position_near(pose1, pose2, position_tol);
}

bool position_near(pose_t pose1, pose_t pose2, float position_tol) {
  return pose_distance(pose1, pose2) < position_tol;
}

unicycle_t get_controls(pose_t current, pose_t target, unicycle_t limits) {
  float distance = pose_distance(target, current);
  float ang_distance = abs(target.theta - current.theta);
  int ang_direction = sign(target.theta - current.theta);

  unicycle_t controls = {
    min(distance, limits.v),
    ang_direction * min(ang_distance, limits.w)
  };
  return controls;
}

void set_motors(unicycle_t controls, float body_width, float wheel_radius) {
  float left_wheel = (controls.v + 0.5 * body_width * controls.w) / wheel_radius;
  float right_wheel = (controls.v - 0.5 * body_width * controls.w) / wheel_radius;
  
  lmtr_pwr(left_wheel);
  rmtr_pwr(right_wheel);
}

void update_state(pose_t *state, unicycle_t controls, float dt) {
  pose_t derivative = {
    controls.v * cos(state->theta),
    controls.v * sin(state->theta),
    controls.w
  };
  
  state->x += derivative.x * dt;
  state->y += derivative.y * dt;
  state->theta += derivative.theta * dt;
}

void set_waypoint(waypoint_t *path, pose_t target) {
  waypoint_t *new_waypoint = malloc(sizeof(waypoint_t));
  new_waypoint->pose = target;
  new_waypoint->prev = path;
  new_waypoint->next = 0;

  path->next = new_waypoint;
  path = new_waypoint;
}