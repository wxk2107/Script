#include "collision_checking.hpp"

int main(int argc, char *argv[]) {

  /*
  transformMsgToEigen (const geometry_msgs::Transform &m, Eigen::Affine3d &e)
  http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html#a4323ed345fb25c89de3a297fdc7401a4
  */


  std::string gripper_model_path = "/home/damao/Documents/ROS/Goal-directed-Manipulation/src/grasp_pose_checker/data/gripper/gripper/gripper_local.txt";
  pcl::PointCloud<pcl::PointXYZ>::Ptr gripper_local_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  ReadGripperLocalPose(gripper_model_path, gripper_local_cloud);

  Eigen::Affine3f transform_gripper2world_affine(Eigen::Affine3f::Identity());

  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("pcd/" + std::string(argv[1]), *scene);

  if (gripperPoseChecker(transform_gripper2world_affine, scene, gripper_local_cloud)) {
    std::cout << "No Collision!" << std::endl;
  }
  else {
    std::cout << "WARNING: Collision!" << std::endl;
  }
  return 0;
}
