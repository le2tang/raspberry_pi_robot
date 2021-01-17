#include "path.h"

#include <math.h>
#include <stdlib.h>

#include "config.h"
#include "hw.h"

pose rotate(pose p, float angle) {
  pose rotated_pose = {
    cos(angle) * p.x - sin(angle) * p.y,
    sin(angle) * p.x + cos(angle) * p.y,
    p.theta
  };
  return rotated_pose;
}

float pose_distance(pose p1, pose p2) {
  float dx = p1.x - p2.x;
  float dy = p1.y - p2.y;
  return sqrt(dx * dx + dy * dy);
}

bool pose_near(pose p1, pose p2, float position_tol, float angle_tol) {
  return (abs(p1.theta - p2.theta) <= angle_tol) && position_near(p1, p2, position_tol);
}

bool position_near(pose p1, pose p2, float position_tol) {
  return pose_distance(p1, p2) <= position_tol;
}

unicycle get_controls(pose curr, pose target, unicycle limits) {
  float distance = pose_distance(target, curr);
  float ang_distance = target.theta - curr.theta;
  int ang_direction = 0;
  if (ang_distance > 0)
    ang_direction = 1;
  else if (ang_distance < 0)
    ang_direction = -1;

  unicycle controls = {
    (abs(distance) < limits.v) ? abs(distance) : limits.v,
    ang_direction * ((ang_distance < limits.w) ? ang_distance : limits.w)
  };
  return controls;
}

void set_motors(unicycle controls, float body_width, float wheel_radius) {
  float left_wheel = (controls.v + 0.5 * body_width * controls.w) / wheel_radius / controls.v;
  float right_wheel = (controls.v - 0.5 * body_width * controls.w) / wheel_radius / controls.v;
  
  motors_set(left_wheel, right_wheel);
}

void update_state(pose *state, unicycle controls, float dt) {
  pose derivative = {
    controls.v * cos(state->theta),
    controls.v * sin(state->theta),
    controls.w
  };

  state->x += derivative.x * dt;
  state->y += derivative.y * dt;
  state->theta += derivative.theta * dt;
}

waypoint *waypoint_init(pose target, waypoint *prev, waypoint *next) {
  waypoint *new_waypoint = malloc(sizeof(waypoint));
  new_waypoint->target.x = target.x;
  new_waypoint->target.y = target.y;
  new_waypoint->target.theta = target.theta;
  new_waypoint->prev = prev;
  new_waypoint->next = next;

  return new_waypoint;
}

void waypoint_set_prev(waypoint *curr, pose target) {
  waypoint new_waypoint;
  waypoint_init(&new_waypoint, target, curr->prev, curr);

  curr->prev = &new_waypoint;
}

void waypoint_set_next(waypoint *curr, pose target) {
  waypoint new_waypoint;
  waypoint_init(&new_waypoint, target, curr, curr->next);

  curr->next = &new_waypoint;
}
