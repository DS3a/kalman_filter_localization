#include <cstdio>
#include <eigen3/Eigen/Core>
#include <iostream>

namespace KalmanFilter {
  class LinearKalmanFilter {
    // control inputs from wheel encoders
    // imu input for acceleration (needs to be filtered to remove gravity using orientation)
    // gps to give position
    public:
    LinearKalmanFilter(double input_variance) {
      input_variance = input_variance;
      // input is velocity from encoders

      state.setZero();
      previous_state = state;

      state_covariance.setIdentity();
      previous_state_covariance = state_covariance;
    }

    void predict(double dt, double control_signal, double cov) {
      // dt = time delay
      // control signal is the velocity we are getting from the wheel encoder
      // cov = process noise covariance

      A << 1, dt, 0.5*dt*dt, 0, 0, dt, 0, 0, 1;
      // A is the transition matrix

      U << 0, control_signal, 0;
      // U is the input vector

      // Q is the process noise covariance

      predicted_state = (A * previous_state) + U;
      std::cout << predicted_state << std::endl;

      // debugging steps
      state = predicted_state;
      previous_state = state;
    }

    void measure_acceleration(double dt, double acceleration, double cov) {
      // function to update the state belief based on acceleration data
      // dt = time delay
      // acceleration = acceleration measurement
      // cov = acceleration measurement covariance
    }

    void measure_velocity(double dt, double velocity, double cov) {
      // function to update the state of the belief based on velocity measurement
      // dt = time delay
      // velocity = velocity measurement
      // cov = velocity measurement covariance
    }

    void measure_position(double dt, double position, double cov) {
      // function to update state belief based on position data
      // dt = time
      // position = position measurement
      // cov = position measurement covariance
    }

    void update() {
      // function to update the current state
      previous_state = state;

      // modify state based on sensor measurements
    }

    void modify_input_variance(double variance) {
      input_variance = variance;
    }

    double get_position() const {
      return state[0];
    }

    double get_velocity() const {
      return state[1];
    }

    double get_acceleration() const {
      return state[2];
    }

    private:
    Eigen::Matrix<double, 3, 3> A;
    // A is the transition matrix

    Eigen::Matrix<double, 3, 1> U;
    // U is the input matrix input is the velocity from encoders

    Eigen::Matrix<double, 3, 3> Q;
    // Q is the process noise covariance matrix

    Eigen::Matrix<double, 3, 1> previous_state;
    Eigen::Matrix<double, 3, 1> predicted_state;
    Eigen::Matrix<double, 3, 1> state;
    // state consists of x, x_dot, and x_dot_dot
    // i.e. position, velocity, and acceleration in one dimension

    Eigen::Matrix<double, 3, 3> previous_state_covariance;
    Eigen::Matrix<double, 3, 3> predicted_state_covariance;
    Eigen::Matrix<double, 3, 3> state_covariance;
    // 3x3 matrix containing the state covariance

    double input_variance;
    // velocity from encoders
  };
}
