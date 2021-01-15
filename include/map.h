#ifndef MAP_H_
#define MAP_H_

typedef struct {
  float *data;
  size_t nrows;
  size_t ncols;
  float xdelta;
  float ydelta;
  pose_t pose;
} map_t;

#endif