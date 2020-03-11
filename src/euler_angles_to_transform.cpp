#include "euler_angles_to_transform.h"

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  double x1 = xzx.x() * M_PI / 180;
  double z = xzx.y() * M_PI / 180;
  double x2 = xzx.z() * M_PI / 180;

  Eigen::Matrix4d rx1, rz, rx2;
  rx1 <<
  1,    0,          0,          0,
  0,    cos(x1),    -sin(x1),   0,
  0,    sin(x1),    cos(x1),    0,
  0,    0,          0,          1;

  rz<<
  cos(z),   -sin(z),0,  0,
  sin(z),   cos(z), 0,  0,
  0,        0,      1,  0,
  0,        0,      0,  1;

  rx2 <<
  1,    0,          0,          0,
  0,    cos(x2),   sin(x2),    0,
  0,    sin(x2),   cos(x2),    0,
  0,    0,          0,          1;


  Eigen::Affine3d A;
  A.matrix() << rx2 * rz * rx1;

  return A;
  /////////////////////////////////////////////////////////////////////////////
}
