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
    for (int i = 0; i < 3; i++) {
      std::getline(ss, token, ',');
    }
    std::getline(ss, token, ',');
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
  Q.setZero();
  Q(0, 0) = 0.0002;
  Q(1, 1) = 0.00001;

  Eigen::MatrixXd R(1, 1);
  R.setZero();
  R(0, 0) = 0.004;

  Eigen::VectorXd x0(2);
  x0(0) = yaw_obs[0];
  x0(1) = 0.0;

  Eigen::MatrixXd P0(2, 2);
  P0.setIdentity();
  P0 = P0 * 10.0;

  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    Eigen::VectorXd sum = a + b;
    sum(0) = std::atan2(std::sin(sum(0)), std::cos(sum(0)));
    return sum;
  };

  tools::ExtendedKalmanFilter ekf(x0, P0, x_add);

  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    const double pi = std::acos(-1.0);
    Eigen::VectorXd result(1);
    double d = a(0) - b(0);
    double best = std::atan2(std::sin(d), std::cos(d));
    double min_abs = std::abs(best);
    double cand = std::atan2(std::sin(d + pi), std::cos(d + pi));
    if (std::abs(cand) < min_abs) { best = cand; min_abs = std::abs(cand); }
    cand = std::atan2(std::sin(d - pi), std::cos(d - pi));
    if (std::abs(cand) < min_abs) { best = cand; }
    result(0) = best;
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

    ekf.update(z_vec, H, R, z_subtract);

    filtered.push_back(ekf.x(0));
  }

  std::ofstream out("yaw_1d_ekf.csv");
  out << "raw_yaw,filt_yaw\n";
  for (int i = 0; i < static_cast<int>(yaw_obs.size()); i++) {
    out << yaw_obs[i] << ',' << filtered[i] << '\n';
  }

  return 0;
}
