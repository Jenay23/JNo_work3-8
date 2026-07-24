#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <Eigen/Dense>
#include "extended_kalman_filter/extended_kalman_filter.hpp"

std::vector<double> read_yaw_data(const std::string & path)
{
  std::vector<double> data;
  std::ifstream file(path);
  if (!file.is_open()) {
    return data;
  }

  std::string line;
  std::getline(file, line);

  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    std::stringstream ss(line);
    std::string token;
    for (int i = 0; i < 4; i++) {
      std::getline(ss, token, ',');
    }
    double yaw = std::stod(token);
    data.push_back(yaw);
  }
  return data;
}

int main()
{
  std::vector<double> yaw_obs = read_yaw_data("armor_data.csv");
  if (yaw_obs.empty()) {
    return -1;
  }

  double dt = 1.0;

  Eigen::MatrixXd F(2, 2);
  F(0, 0) = 1.0;
  F(0, 1) = dt;
  F(1, 0) = 0.0;
  F(1, 1) = 1.0;

  Eigen::MatrixXd H(1, 2);
  H(0, 0) = 1.0;
  H(0, 1) = 0.0;

  Eigen::MatrixXd Q(2, 2);
  Q(0, 0) = 0.0001;
  Q(0, 1) = 0.0;
  Q(1, 0) = 0.0;
  Q(1, 1) = 0.00001;

  Eigen::MatrixXd R(1, 1);
  R(0, 0) = 0.004;

  Eigen::VectorXd x0(2);
  x0(0) = yaw_obs[0];
  x0(1) = 0.0;

  Eigen::MatrixXd P0(2, 2);
  P0(0, 0) = 10.0;
  P0(0, 1) = 0.0;
  P0(1, 0) = 0.0;
  P0(1, 1) = 10.0;

  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    Eigen::VectorXd result(2);
    result(0) = a(0) + b(0);
    result(1) = a(1) + b(1);
    result(0) = std::atan2(std::sin(result(0)), std::cos(result(0)));
    return result;
  };

  tools::ExtendedKalmanFilter ekf(x0, P0, x_add);

  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    Eigen::VectorXd result(1);
    result(0) = a(0) - b(0);
    result(0) = std::atan2(std::sin(result(0)), std::cos(result(0)));
    return result;
  };

  std::vector<double> filtered;
  filtered.reserve(yaw_obs.size());

  for (int i = 0; i < static_cast<int>(yaw_obs.size()); i++) {
    double z = yaw_obs[i];
    auto f = [&](const Eigen::VectorXd & x) {
      Eigen::VectorXd x_pred(2);
      x_pred(0) = F(0, 0) * x(0) + F(0, 1) * x(1);
      x_pred(1) = F(1, 0) * x(0) + F(1, 1) * x(1);
      x_pred(0) = std::atan2(std::sin(x_pred(0)), std::cos(x_pred(0)));
      return x_pred;
    };
    ekf.predict(F, Q, f);

    Eigen::VectorXd z_vec(1);
    z_vec(0) = z;

    double yaw_res = z - ekf.x(0);
    if (std::abs(yaw_res) > M_PI_2) {
      if (yaw_res > 0) {
        z_vec(0) = z_vec(0) - M_PI;
      } else {
        z_vec(0) = z_vec(0) + M_PI;
      }
      z_vec(0) = std::atan2(std::sin(z_vec(0)), std::cos(z_vec(0)));
    }

    ekf.update(z_vec, H, R, z_subtract);
    filtered.push_back(ekf.x(0));
  }

  std::ofstream out("yaw_filtered_1d.csv");
  out << "raw_yaw,filt_yaw\n";
  for (int i = 0; i < static_cast<int>(yaw_obs.size()); i++) {
    out << yaw_obs[i] << ',' << filtered[i] << '\n';
  }

  return 0;
}