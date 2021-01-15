#ifndef PATH_H_
#define PATH_H_

#include <stdbool.h>

typedef struct {
  float x;
  float y;
  float theta;
} pose_t;

typedef struct {
  float v;
  float w;
} unicycle_t;

typedef struct {
  pose_t pose;
  waypoint_t *prev;
  waypoint_t *next;
} waypoint_t;

int sign(float x) {
  if (x < 0) {
    return -1;
  }
  else if (x > 0) {
    return 1;
  }
  return 0;
}

pose_t rotate(pose_t pose, float angle);
float pose_distance(pose_t pose1, pose_t pose2);
bool pose_near(pose_t pose1, pose_t pose2, float position_tol, float angle_tol);
bool position_near(pose_t pose1, pose_t pose2, float position_tol);

unicycle_t get_controls(pose_t current, pose_t target, unicycle_t limits);
void set_motors(unicycle_t controls, float body_width, float wheel_radius);
void update_state(pose_t *state, unicycle_t controls, float dt);
void set_waypoint(waypoint_t *path, pose_t target);

#endif
