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

extern inline bool map_inbounds(map_t *map, size_t row, size_t col);
extern inline float map_get(map_t *map, size_t row, size_t col);
extern inline float mat_set(map_t *map, size_t row, size_t col, float data);
extern inline size_t map_get_index(map_t *map, size_t row, size_t col);

void interest_map_init(map_t *map, size_t nrows, size_t ncols, float xdelta, float ydelta);
void interest_map_update(map_t *map, pose_t robot_pose, float max_interest, float decay);

#endif