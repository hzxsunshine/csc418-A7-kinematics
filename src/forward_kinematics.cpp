#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function

void forward_kinematics(
  const Skeleton & skeleton,
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  // lambda function
  std::function<void(int, const Skeleton &, std::vector<Eigen::Affine3d,
          Eigen::aligned_allocator<Eigen::Affine3d>> &)> forward_recursive = [&](int index, const Skeleton & skeleton,
                                       std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T){
      if (skeleton[index].parent_index == -1) {
          T[index] = Eigen::Affine3d::Identity();
      } else{
          forward_recursive(skeleton[index].parent_index, skeleton, T);

          Eigen::Affine3d inverse = skeleton[index].rest_T.inverse();
          Eigen::Vector3d local_xzx = skeleton[index].xzx;
          Eigen::Affine3d Rotation = euler_angles_to_transform(local_xzx);

          T[index] = T[skeleton[index].parent_index] * skeleton[index].rest_T * Rotation * inverse;
      }
  };

  T.resize(skeleton.size(),Eigen::Affine3d::Identity());
  for (int i = 0; i < skeleton.size(); ++i) {
      forward_recursive(i, skeleton, T);
  }
  /////////////////////////////////////////////////////////////////////////////
}
