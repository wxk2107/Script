#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/io/pcd_io.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>      /* printf */
#include <math.h>       /* sin */
#include <vector>
#include <string>

bool gripperPoseChecker (Eigen::Affine3f& transform_gripper2world_affine, pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr gripper_local_cloud);

void ConstructVectorsForGripperPart(std::vector<tf::Vector3>& gripper_part, int start_index, pcl::PointCloud<pcl::PointXYZ>::Ptr gripper_pose_world);

bool PointToGripperCollisionChecking(tf::Vector3 point, std::vector<tf::Vector3> palm_vectors, std::vector<tf::Vector3> left_finger_vectors, std::vector<tf::Vector3> right_finger_vectors);

void ReadGripperLocalPose(std::string pose_file, pcl::PointCloud<pcl::PointXYZ>::Ptr gripper_local_cloud);
