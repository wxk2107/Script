#include <iostream>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// for crop_box
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <tf_conversions/tf_eigen.h>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data

  cloud.is_dense = false;

  int length = 100;
  int width = 100;
  int height = 100;
  int SCALE_ = 100;
  // BOTTOM
  for (int i = -length/2; i < length/2; i = i + 5)
  {
    for (int j = -width/2; j < width/2; j = j + 5)
    {
      for (int k = -height/2; k < height/2; k = k + 5) {
        pcl::PointXYZ point;
        point.x = (float)i/SCALE_;
        point.y = (float)j/SCALE_;
        point.z = (float)k/SCALE_;
        cloud.points.push_back(point);
      }
    }
  }

  cloud.width    = cloud.points.size();
  cloud.height   = 1;
  pcl::io::savePCDFileASCII ("box.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to box.pcd." << std::endl;

  tf::Vector3 x_axis(1, 1, 0);
  tf::Vector3 z_axis(1, -1, 0);
  x_axis = x_axis.normalize();
  z_axis = z_axis.normalize();

  tf::Vector3 y_axis = z_axis.cross(x_axis);

  Eigen::Matrix4f transform;
  transform << x_axis.x(), y_axis.x(), z_axis.x(), 0.,
                             x_axis.y(), y_axis.y(), z_axis.y(), 0.,
                             x_axis.z(), y_axis.z(), z_axis.z(), 0.,
                             0 				 , 0         , 0         , 1;
  // transform << 1, 0, 0, 0.,
  //                         0, 1, 0, 0.,
  //                         0, 0, 1, 0.,
  //                           0 				 , 0         , 0         , 1;
  Eigen::Affine3f transform_affine;
  transform_affine.matrix() = transform;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ> () );
  cloud_ptr = cloud.makeShared();
  pcl::CropBox<pcl::PointXYZ> cropFilter(true);
  cropFilter.setInputCloud(cloud_ptr);
  cropFilter.setMax(Eigen::Vector4f(0.1, 0.20, 0.30, 1.0f));
  cropFilter.setMin(Eigen::Vector4f(-0.10, -0.20, -0.30, 1.0f));

  Eigen::Matrix3f m;
  m <<  transform_affine.matrix()(0,0), transform_affine.matrix()(0,1), transform_affine.matrix()(0,2),
          transform_affine.matrix()(1,0), transform_affine.matrix()(1,1), transform_affine.matrix()(1,2),
          transform_affine.matrix()(2,0), transform_affine.matrix()(2,1), transform_affine.matrix()(2,2);
  Eigen::Vector3f boxTranslation(transform_affine.matrix()(0,3), transform_affine.matrix()(1,3), transform_affine.matrix()(2,3));
  Eigen::Vector3f boxXYZRotation = m.eulerAngles(2,1,0);

  // cropFilter.setTranslation(boxTranslation);
  // cropFilter.setRotation(Eigen::Vector3f(boxXYZRotation[2], boxXYZRotation[1], boxXYZRotation[0])); // set Rotation in order of X Y Z.
  cropFilter.setTransform(transform_affine);
  // std::cout << cropFilter.getTransform().matrix() << std::endl;
  cropFilter.setNegative(false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ> () );
  cropFilter.filter(*output_cloud);
  std::cout << output_cloud->points.size() << std::endl;
  pcl::io::savePCDFileASCII ("output_cloud.pcd", *output_cloud);



  return (0);
}
