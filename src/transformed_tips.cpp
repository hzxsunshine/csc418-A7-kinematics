#include "transformed_tips.h"
#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton, 
  const Eigen::VectorXi & b)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  Eigen::VectorXd res;
  res.resize(3*b.size());

  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> T;
  forward_kinematics(skeleton, T);

  for (int i = 0; i < b.size(); ++i) {
      int b_i = b[i];
      Bone bone = skeleton[b_i];

      Eigen::Affine3d trans = T[b_i];
      Eigen::Affine3d rest = bone.rest_T;
      Eigen::Vector4d tip(skeleton[b[i]].length, 0, 0, 1);

      tip = trans * rest * tip;

      res[3 * i] = tip.x();
      res[3 * i + 1] = tip.y();
      res[3 * i + 2] = tip.z();
  }
  return res;
  /////////////////////////////////////////////////////////////////////////////
}
