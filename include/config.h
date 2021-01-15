#ifndef CONFIG_H_
#define CONFIG_H_

#define TIMESTEP 0.01

#define LMTR_FWD 0 // Left motor forwards pin
#define LMTR_BCK 1 // Left motor backwards pin
#define RMTR_FWD 2 // Right motor forwards pin
#define RMTR_BCK 3 // Right motor backwards pin

#define LMTR_PWM 4 // Left motor PWM pin
#define RMTR_PWM 5 // Right motor PWM pin

#define NEAR_WAYPOINT_TOL 0.05 // m

#define ROBOT_BODY_WIDTH 0.1 // m
#define ROBOT_WHEEL_RADIUS 0.04 // m

#define INTEREST_MAP_NROWS 10
#define INTEREST_MAP_NCOLS 10
#define INTEREST_MAP_DX 0.05 // m
#define INTEREST_MAP_DY 0.05 // m
#define INTEREST_MAP_DECAY 0.1

#endif
