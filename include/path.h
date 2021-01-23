#ifndef PATH_H_
#define PATH_H_

#include <stdbool.h>

typedef struct pose {
  float x;
  float y;
  float theta;
} pose;

typedef struct unicycle {
  float v;
  float w;
} unicycle;

typedef struct waypoint {
  pose target;
  struct waypoint *prev;
  struct waypoint *next;
} waypoint;

pose rotate(pose p, float angle);
pose pose_difference(pose pfinal, pose pinit);
float pose_distance(pose p1, pose p2);
bool pose_near(pose p1, pose p2, float position_tol, float angle_tol);
bool position_near(pose p1, pose p2, float position_tol);

unicycle get_controls(pose curr, pose target, unicycle limits);
void set_motors(unicycle controls, float body_width, float wheel_radius);
pose update_state(pose state, unicycle controls, float dt);

waypoint *waypoint_init(pose target, waypoint *prev, waypoint *next);
void waypoint_set_prev(waypoint *curr, pose target);
void waypoint_set_next(waypoint *curr, pose target);

#endif
