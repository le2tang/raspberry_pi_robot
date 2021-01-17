#ifndef MAP_H_
#define MAP_H_

#include <stddef.h>

#include "path.h"

typedef struct map {
  float *data;
  size_t nrows;
  size_t ncols;
  float xdelta;
  float ydelta;
  pose origin;
} map;

bool map_inbounds(map *m, size_t row, size_t col);
float map_get(map *m, size_t row, size_t col);
void map_set(map *m, size_t row, size_t col, float data);
size_t map_get_index(map *m, size_t row, size_t col);

void interest_map_init(map *interest_map, size_t nrows, size_t ncols, float xdelta, float ydelta);
void interest_map_update(map *interest_map, pose robot_pose, float max_interest, float decay);

#endif