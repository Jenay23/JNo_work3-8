#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <Eigen/Dense>
#include "extended_kalman_filter/extended_kalman_filter.hpp"

std::vector<Eigen::VectorXd> read_armor_data(const std::string & path)
{
  std::vector<Eigen::VectorXd> data;
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
    Eigen::VectorXd z(4);
    for (int i = 0; i < 4; i++) {
      std::getline(ss, token, ',');
      z(i) = std::stod(token);
    }
    data.push_back(z);
  }
  return data;
}

int main()
{
  auto obs = read_armor_data("armor_data.csv");
  if (obs.empty()) {
    return -1;
  }

  double dt = 1.0;

  Eigen::MatrixXd F(8, 8);
  F.setIdentity();
  F(0, 4) = dt;
  F(1, 5) = dt;
  F(2, 6) = dt;
  F(3, 7) = dt;

  Eigen::MatrixXd H(4, 8);
  H.setZero();
  H(0, 0) = 1;
  H(1, 1) = 1;
  H(2, 2) = 1;
  H(3, 3) = 1;

  Eigen::MatrixXd Q(8, 8);
  Q.setZero();
  Q(0, 0) = 0.002;
  Q(1, 1) = 0.002;
  Q(2, 2) = 0.0001;
  Q(3, 3) = 0.0002;
  Q(4, 4) = 0.005;
  Q(5, 5) = 0.005;
  Q(6, 6) = 0.005;
  Q(7, 7) = 0.00001;

  Eigen::MatrixXd R(4, 4);
  R.setZero();
  R(0, 0) = 0.003;
  R(1, 1) = 0.003;
  R(2, 2) = 0.0001;
  R(3, 3) = 0.004;

  Eigen::VectorXd x0(8);
  x0 << obs[0](0), obs[0](1), obs[0](2), obs[0](3), 0, 0, 0, 0;

  Eigen::MatrixXd P0(8, 8);
  P0.setIdentity();
  P0 = P0 * 10.0;

  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    Eigen::VectorXd sum = a + b;
    sum(3) = std::atan2(std::sin(sum(3)), std::cos(sum(3)));
    return sum;
  };

  tools::ExtendedKalmanFilter ekf(x0, P0, x_add);

  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    const double pi = std::acos(-1.0);
    Eigen::VectorXd diff = a - b;
    double d = diff(3);
    double best = std::atan2(std::sin(d), std::cos(d));
    double min_abs = std::abs(best);
    double cand = std::atan2(std::sin(d + pi), std::cos(d + pi));
    if (std::abs(cand) < min_abs) { best = cand; min_abs = std::abs(cand); }
    cand = std::atan2(std::sin(d - pi), std::cos(d - pi));
    if (std::abs(cand) < min_abs) { best = cand; }
    diff(3) = best;
    return diff;
  };

  std::vector<Eigen::VectorXd> filtered;
  filtered.reserve(obs.size());

  for (int i = 0; i < static_cast<int>(obs.size()); i++) {
    Eigen::VectorXd z = obs[i];

    auto f = [&](const Eigen::VectorXd & x) {
      Eigen::VectorXd x_pred = F * x;
      x_pred(3) = std::atan2(std::sin(x_pred(3)), std::cos(x_pred(3)));
      return x_pred;
    };
    ekf.predict(F, Q, f);

    ekf.update(z, H, R, z_subtract);
    filtered.push_back(ekf.x);
  }

  std::ofstream out("work6.csv");
  out << "raw_x,raw_y,raw_z,raw_yaw,filt_x,filt_y,filt_z,filt_yaw\n";
  for (int i = 0; i < static_cast<int>(obs.size()); i++) {
    out << obs[i](0) << ',' << obs[i](1) << ',' << obs[i](2) << ',' << obs[i](3) << ','
        << filtered[i](0) << ',' << filtered[i](1) << ',' << filtered[i](2) << ',' << filtered[i](3) << '\n';
  }

  return 0;
}
