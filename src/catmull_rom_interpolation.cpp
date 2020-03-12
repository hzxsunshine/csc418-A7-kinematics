#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>

Eigen::Vector3d catmull_rom_interpolation(
        const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
        double t)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  // href: https://en.wikipedia.org/wiki/Centripetal_Catmull–Rom_spline
  // c(t) = a t^3 + b t ^2 + c t + d
  // mk = theta_k+1 - theta_k / t_k+1 - t_k
  Eigen::Vector3d c_t(0, 0, 0);
  if (keyframes.empty()) {
      return c_t;
  }

  int size = keyframes.size();
  double local_t = std::fmod(t, keyframes.back().first);


  double t_0, t_1, t_2, t_3;
  Eigen::Vector3d m_0, m_2;
  Eigen::Vector3d theta_0, theta_1, theta_2, theta_3;

  int index = 0;
  for (int i = 0; i < size - 1; i++){
      if ((keyframes[i].first <= local_t) && (local_t < keyframes[i + 1].first))
          index = i;
  }

  int offset = (index == 0) ? 0 : 1;
  int offset_2 = (index + 2 == size) ? 1 : 0;

  t_0 = keyframes[index - offset].first;
  t_1 = keyframes[index].first;
  t_2 = keyframes[index + 1].first;
  t_3 = keyframes[index + 2 - offset_2].first;

  theta_0 = keyframes[index - offset].second;
  theta_1 = keyframes[index].second;
  theta_2 = keyframes[index + 1].second;
  theta_3 = keyframes[index + 2 - offset_2].second;

  m_0 = (theta_2 - theta_0) / (t_2 - t_0);
  m_2 = (theta_3 - theta_1) / (t_3 - t_1);

  double time = (local_t - t_1) / (t_2 - t_1);

  c_t = (2 * pow(time, 3) - 3 * pow(time, 2) + 1) * theta_1 + \
        (pow(time, 3) - 2 * pow(time, 2) + time) * m_0 + \
        (-2 * pow(time, 3) + 3 * pow(time, 2)) * theta_2 + \
        (pow(time, 3) - pow(time, 2))  * m_2;
  return c_t;
  /////////////////////////////////////////////////////////////////////////////
}
