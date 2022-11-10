#include <cstdio>
#include "kalman_filter.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  KalmanFilter::LinearKalmanFilter lkf(0.01);
  printf("current position: %f\n\n", lkf.get_position());
  for(int i=0; i<11; i++)
    lkf.predict(0.01, 0.2, 0.01);
  return 0;
}
