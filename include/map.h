#ifndef MAP_H_
#define MAP_H_

#include <stddef.h>

#include "path.h"

typedef struct {
  float *data;
  size_t nrows;
  size_t ncols;
  float xdelta;
  float ydelta;
  pose_t pose;
} map_t;

extern map_t interest_map;

bool map_inbounds(map_t *map, size_t row, size_t col);
float map_get(map_t *map, size_t row, size_t col);
float mat_set(map_t *map, size_t row, size_t col, float data);

void init_interest_map(map_t *map, size_t nrows, size_t ncols, float xdelta, float ydelta);
void update_interest_map(map_t *map, pose_t robot_pose, float decay);

#endif