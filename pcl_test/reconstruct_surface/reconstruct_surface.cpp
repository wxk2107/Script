#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>


#include <fstream>

using namespace Eigen;
using namespace pcl;
using namespace std;

#define RESOLUTION 0.01

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  std::string name = "test_pcd";
  reader.read (name+".pcd", *cloud_blob);
  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (RESOLUTION, RESOLUTION, RESOLUTION);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (name + "_lms400_downsampled.pcd", *cloud_filtered, false);

  /***************CROP BOX****************/
  // float l = 1.0, w = 1.0, h = 1.0;
  float l = 0.6, w = 0.6, h = 1.0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_crop (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropBox<pcl::PointXYZ> cropFilter(true);
	cropFilter.setInputCloud(cloud_filtered);
	cropFilter.setMax(Eigen::Vector4f(l/2, w/2, h/2, 1.0f));
	cropFilter.setMin(Eigen::Vector4f(-l/2, -w/2, -h/2, 1.0f));

  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(euler.x, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(euler.y, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(euler.z, Eigen::Vector3f::UnitZ());
  Eigen::Affine3f transform_box2world = Eigen::Translation3f(position.x, position.y, position.z) * q;
  /***************/
  cropFilter.SetTransform(transform_box2world);
  // cropFilter.setTranslation(Eigen::Vector3f(0.7f, 0.7f, h/2));
	// cropFilter.setRotation(Eigen::Vector3f(0f, 0f, 0f)); // set Rotation in order of X Y Z.
	cropFilter.setNegative(false);
	cropFilter.filter(*cloud_filtered_crop);

  writer.write<pcl::PointXYZ> (name + "_crop.pcd", *cloud_filtered_crop, false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box_frame(new pcl::PointCloud<pcl::PointXYZ> () );
  pcl::transformPointCloud(*cloud_filtered_crop, *cloud_box_frame, transform_box2world);
  writer.write<pcl::PointXYZ> (name + "_crop_box_frame.pcd", *cloud_box_frame, false);

  /***************DISCRETIZATION****************/
  int m_l = (int)(l/RESOLUTION), m_w = (int)(w/RESOLUTION);
  MatrixXd m = MatrixXd::Constant(m_l, m_w, -1);
  for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_filtered_crop->begin(); it != cloud_filtered_crop->end(); it++){
    // cout << it->x << ", " << it->y << ", " << it->z << endl;
    m((int)((it->x)/RESOLUTION), (int)((it->y)/RESOLUTION)) = max(m((int)(it->x/RESOLUTION), (int)(it->y/RESOLUTION)), round(it->z/RESOLUTION));
  }
  ofstream myfile;
  myfile.open ("matrix.txt");
  myfile << m;
  myfile.close();

  /***************SUPPLEMENT****************/
  for (int i = 0; i < m.rows(); i++) {
    if (m(i, 0) < 9) m(i, 0) = 9;
    for (int j = 0; j < m.cols(); j++) {
      if (m(i, j) < 9) m(i, j) = m(i, j-1);
    }
  }
  myfile.open ("matrix_supplement.txt");
  myfile << m;
  myfile.close();
  // for (int i = 0; i < m_l; i++) {
  //   for (int j = 0; j <= i; j++) {
  //     if
  //   }
  // }


  /***************VISUALIZATION****************/

  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // // Create the segmentation object
  // pcl::SACSegmentation<pcl::PointXYZ> seg;
  // // Optional
  // seg.setOptimizeCoefficients (true);
  // // Mandatory
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setMaxIterations (1000);
  // seg.setDistanceThreshold (0.1);
  //
  // // Create the filtering object
  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  //
  // int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // // While 30% of the original cloud is still there
  // while (cloud_filtered->points.size () > 0.3 * nr_points)
  // {
  //   // Segment the largest planar component from the remaining cloud
  //   seg.setInputCloud (cloud_filtered);
  //   seg.segment (*inliers, *coefficients);
  //   if (inliers->indices.size () == 0)
  //   {
  //     std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
  //     break;
  //   }
  //
  //   // Extract the inliers
  //   extract.setInputCloud (cloud_filtered);
  //   extract.setIndices (inliers);
  //   extract.setNegative (false);
  //   extract.filter (*cloud_p);
  //   std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
  //
  //   std::stringstream ss;
  //   ss << "table_scene_lms400_plane_" << i << ".pcd";
  //   writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
  //
  //   // Create the filtering object
  //   extract.setNegative (true);
  //   extract.filter (*cloud_f);
  //   cloud_filtered.swap (cloud_f);
  //   i++;
  // }

  return (0);
}
