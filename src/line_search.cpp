#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code

  double sigma = max_step;

  Eigen::VectorXd d_z = z - sigma * dz;
  proj_z(d_z);

  while (f(d_z) > f(z)) {
      sigma = sigma / 2;
      d_z = z - sigma * dz;
      proj_z(d_z);
  }

  return sigma;
  /////////////////////////////////////////////////////////////////////////////
}
