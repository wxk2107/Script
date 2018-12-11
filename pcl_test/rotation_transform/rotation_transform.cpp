#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
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
// #include <tf_conversns/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

int main (int argc, char** argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }
    pcl::PCDWriter writer;
    int sign = 1;
    Eigen::Quaternionf q_place_local_based_on_grasp;
    Eigen::Affine3f transform_place_local_based_on_grasp_affine;
    q_place_local_based_on_grasp = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(sign * M_PI/2, Eigen::Vector3f::UnitY()) *
        // Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
    transform_place_local_based_on_grasp_affine = Eigen::Translation3f(0,0,0)* q_place_local_based_on_grasp ;
    Eigen::Matrix4f transform_place_local_based_on_grasp = transform_place_local_based_on_grasp_affine.matrix();

    pcl::PointCloud<pcl::PointXYZ>::Ptr object_point_cloud_debug3(new pcl::PointCloud<pcl::PointXYZ> () );
    pcl::transformPointCloud(*cloud, *object_point_cloud_debug3, transform_place_local_based_on_grasp);
    writer.write<pcl::PointXYZ> ("after_rotation.pcd", *object_point_cloud_debug3, false);


    return 0;
}
