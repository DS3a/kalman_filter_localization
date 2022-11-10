#include <cstdio>
#include <eigen3/Eigen/Core>

namespace KalmanFilter {
  class LinearKalmanFilter {
    // control inputs from wheel encoders
    // imu input for acceleration (needs to be filtered to remove gravity using orientation)
    // gps to give position
    public:
    KalmanFilter() {
    }

    void predict(double dt, double control_signal, double cov) {
      // dt = time delay
      // control signal is the velocity we are getting from the wheel encoder
      // cov = process noise covariance
    }

    void measure_acceleration(double dt, double acceleration, double cov) {
      // function to update the state belief based on acceleration data
      // dt = time delay
      // acceleration = acceleration measurement
      // cov = acceleration covariance
    }

    void measure_position(double dt, double position, double cov) {
      // function to update state belief based on position data
      // dt = time
      // position = position measurement
      // cov = position covariance
    }

    void update() {
    }


    private:
    Eigen::Matrix<double, > state;
  };
}
