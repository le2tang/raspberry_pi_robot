#include "map.h"

#include <stdio.h>
#include <stdlib.h>

#include "path.h"

bool map_inbounds(map *m, size_t row, size_t col) {
  return (row < m->nrows && col < m->ncols);
}

float map_get(map *m, size_t row, size_t col) {
  return m->data[row * m->ncols + col];
}

void map_set(map *m, size_t row, size_t col, float data) {
  m->data[row * m->ncols + col] = data;
}

size_t map_get_index(map *m, size_t row, size_t col) {
  return row * m->ncols + col;
}

pose pose2map_indices(pose p, map *m) {
  pose map_pose = {
    p.x - m->origin.x,
    p.y - m->origin.y,
    0
  };

  float rot_angle = m->origin.theta - p.theta;
  map_pose = rotate(map_pose, rot_angle);
  
  map_pose.x /= m->xdelta;
  map_pose.y /= m->ydelta;
  return map_pose;
}

void interest_map_init(map *interest_map, size_t nrows, size_t ncols, float xdelta, float ydelta) {
  interest_map->data = malloc(sizeof(float) * nrows * ncols);
  interest_map->nrows = nrows;
  interest_map->ncols = ncols;
  interest_map->xdelta = xdelta;
  interest_map->ydelta = ydelta;
  interest_map->origin.x = 0;
  interest_map->origin.y = 0;
  interest_map->origin.theta = 0;
}

void interest_map_update(map *interest_map, pose robot_pose, float max_interest, float decay) {
  pose map_indices = pose2map_indices(robot_pose, interest_map);

  for (int i = -1; i < 2; ++i) {
    for (int j = -1; j < 2; ++j) {
      if (map_inbounds(interest_map, map_indices.x + i, map_indices.y + j)) {
        map_set(interest_map, map_indices.x + i, map_indices.y + j, max_interest);
      }
    }
  }

  for (size_t i = 0; i < interest_map->nrows; ++i) {
    for (size_t j = 0; j < interest_map->ncols; ++j) {
        float current_interest = map_get(interest_map, map_indices.x + i, map_indices.y + j);
        float new_interest = (current_interest - decay > 0) ? (current_interest - decay) : 0;
        map_set(interest_map, i, j, new_interest);
    }
  }
}

void map_print(map *m) {
  printf("Map %dx%d: (%3f, %3f) @ %3f\n", m->nrows, m->ncols, m->origin.x, m->origin.y, m->origin.theta);
  for (size_t i = 0; i < m->nrows; ++i) {
    for (size_t j = 0; j < m->ncols; ++j) {
      float val = map_get(m, i, j);
      printf("%.1f", val);
      if (j < (m->ncols - 1)) {
        printf(",");
      }
    }
    printf("\n");
  }
}
