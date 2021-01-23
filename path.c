#include "path.h"

#include <math.h>
#include <stdlib.h>

#include "config.h"
#include "hw.h"

float phase_principle(float t) {
  while (t > M_PI) {
    t -= 2 * M_PI;
  }
  while (t < -M_PI) {
    t += 2 * M_PI;
  }
  return t;
}

float phase_diff_unwrapped(float tfinal, float tinit) {
  while (tfinal - tinit > M_PI) {
    tfinal -= 2 * M_PI;
  }
  while (tinit - tfinal > M_PI) {
    tfinal += 2 * M_PI;
  }
  return tfinal - tinit;
}

pose rotate(pose p, float angle) {
  pose rotated_pose = {
    cos(angle) * p.x - sin(angle) * p.y,
    sin(angle) * p.x + cos(angle) * p.y,
    p.theta
  };
  return rotated_pose;
}

pose pose_difference(pose pfinal, pose pinit) {
  pose difference = {
    pfinal.x - pinit.x,
    pfinal.y - pinit.y,
    phase_diff_unwrapped(pfinal.theta, pinit.theta)
  };
  return difference;
}

float pose_distance(pose p1, pose p2) {
  float dx = p1.x - p2.x;
  float dy = p1.y - p2.y;
  return sqrt(dx * dx + dy * dy);
}

bool pose_near(pose p1, pose p2, float position_tol, float angle_tol) {
  return (fabs(phase_diff_unwrapped(p2.theta, p1.theta)) <= angle_tol) && position_near(p1, p2, position_tol);
}

bool position_near(pose p1, pose p2, float position_tol) {
  return pose_distance(p1, p2) <= position_tol;
}

unicycle get_controls(pose curr, pose target, unicycle limits) {
  pose goto_pose_error = pose_difference(target, curr);
  goto_pose_error = rotate(goto_pose_error, goto_pose_error.theta);

  float goto_distance = pose_distance(target, curr) + 1E-9;
  float error_sq = goto_distance / GOTO_SLOWDOWN_DISTANCE;
  float goto_gain = limits.v / goto_distance * (1 - exp(-error_sq * error_sq));

  goto_pose_error.x *= goto_gain;
  goto_pose_error.y *= goto_gain;

  float speed = cos(curr.theta) * goto_pose_error.x + sin(curr.theta) * goto_pose_error.y;
  float ang_vel = (-sin(curr.theta) * goto_pose_error.x + cos(curr.theta) * goto_pose_error.y) / 0.01;
  
  unicycle controls = {
    speed,
    ang_vel
  };
  return controls;
}

void set_motors(unicycle controls, float body_width, float wheel_radius) {
  float left_wheel = (controls.v + 0.5 * body_width * controls.w) / wheel_radius;
  float right_wheel = (controls.v - 0.5 * body_width * controls.w) / wheel_radius;
  
  motors_set(left_wheel, right_wheel);
}

pose update_state(pose state, unicycle controls, float dt) {
  pose derivative = {
    controls.v * cos(state.theta),
    controls.v * sin(state.theta),
    controls.w
  };

  pose new_state = {
    state.x + derivative.x * dt,
    state.y + derivative.y * dt,
    phase_principle(state.theta + derivative.theta * dt)
  };
  return new_state;
}

waypoint *waypoint_init(pose target, waypoint *prev, waypoint *next) {
  waypoint *new_waypoint = malloc(sizeof(waypoint));
  new_waypoint->target = target;
  new_waypoint->prev = prev;
  new_waypoint->next = next;

  return new_waypoint;
}

void waypoint_set_prev(waypoint *curr, pose target) {  
  waypoint *new_waypoint = waypoint_init(target, curr->prev, curr);
  curr->prev = new_waypoint;
}

void waypoint_set_next(waypoint *curr, pose target) {
  waypoint *new_waypoint = waypoint_init(target, curr, curr->next);
  curr->next = new_waypoint;
}
