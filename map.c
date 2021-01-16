#include "map.h"

#include "path.h"

bool map_inbounds(map_t *map, size_t row, size_t col) {
  return (row < map->nrows && col < map->ncols);
}

float map_get(map_t *map, size_t row, size_t col) {
  return map->data[row * map->ncols + col];
}

void map_set(map_t *map, size_t row, size_t col, float data) {
  map->data[row * map->ncols + col] = data;
}

size_t map_get_index(map_t *map, size_t row, size_t col) {
  return row * map->ncols + col;
}

pose_t pose2map_indices(pose_t pose, map_t *map) {
  pose_t map_pose = {
    pose.x - map->pose.x,
    pose.y - map->pose.y,
    0
  };

  float rot_angle = map->pose.theta - pose.theta;
  map_pose = rotate(map_pose, rot_angle);
  
  map_pose.x /= map->xdelta;
  map_pose.y /= map->ydelta;
  return map_pose;
}

void interest_map_init(map_t *map, size_t nrows, size_t ncols, float xdelta, float ydelta) {
  map->data = malloc(sizeof(float) * nrows * ncols);
  map->nrows = nrows;
  map->ncols = ncols;
  map->xdelta = xdelta;
  map->ydelta = ydelta;
  map->pose.x = 0;
  map->pose.y = 0;
  map->pose.theta = 0;
}

void interest_map_update(map_t *map, pose_t robot_pose, float max_interest, float decay) {
  pose_t map_indices = pose2map_indices(robot_pose, map);

  for (int i = -1; i < 2; ++i) {
    for (int j = -1; j < 2; ++j) {
      if (map_inbounds(map, map_indices.x + i, map_indices.y + j)) {
        map_set(map, map_indices.x + i, map_indices.y + j, max_interest);
      }
    }
  }

  for (size_t i = 0; i < map->nrows; ++i) {
    for (size_t j = 0; j < map->ncols; ++j) {
        float current_interest = map_get(map, map_indices.x + i, map_indices.y + j);
        float new_interest = max(0, current_interest - decay);
        map_set(map, i, j, new_interest);
    }
  }
}
