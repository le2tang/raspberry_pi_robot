#include "path.h"

#include <math.h>

#include "config.h"
#include "hw.h"

pose_t rotate(pose_t pose, float angle) {
  pose_t rotated_pose = {
    cos(angle) * pose.x - sin(angle) * pose.y,
    sin(angle) * pose.x + cos(angle) * pose.y,
    pose.theta
  };
  return rotated_pose;
}

float pose_distance(pose_t pose1, pose_t pose2) {
  float dx = pose1.x - pose2.x;
  float dy = pose1.y - pose2.y;
  return sqrt(dx * dx + dy * dy);
}

bool pose_near(pose_t pose1, pose_t pose2, float position_tol, float angle_tol) {
  return (abs(pose1.theta - pose2.theta) <= angle_tol) && position_near(pose1, pose2, position_tol);
}

bool position_near(pose_t pose1, pose_t pose2, float position_tol) {
  return pose_distance(pose1, pose2) <= position_tol;
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
  float left_wheel = (controls.v + 0.5 * body_width * controls.w) / wheel_radius / controls.v;
  float right_wheel = (controls.v - 0.5 * body_width * controls.w) / wheel_radius / controls.v;
  
  motors_set(left_wheel, right_wheel);
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

void waypoint_init(waypoint_t *waypoint, pose_t pose, waypoint_t *prev, waypoint_t *next) {
  if (waypoint) {
    free(waypoint);
  }

  waypoint = malloc(sizeof(waypoint_t));
  waypoint->pose = pose;
  waypoint->prev = prev;
  waypoint->next = next;
}

void waypoint_set_prev(waypoint_t *waypoint, pose_t pose) {
  waypoint_t new_waypoint;
  waypoint_init(&new_waypoint, pose, waypoint->prev, waypoint);

  waypoint->prev = &new_waypoint;
}

void waypoint_set_next(waypoint_t * waypoint, pose_t pose) {
  waypoint_t new_waypoint;
  waypoint_init(&new_waypoint, pose, waypoint, waypoint->next);

  waypoint->next = &new_waypoint;
}
