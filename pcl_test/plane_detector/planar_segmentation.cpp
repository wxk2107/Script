#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

int
 main (int argc, char** argv)
{
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }
  // Fill in the cloud data
  // cloud->width  = 15;
  // cloud->height = 1;
  // cloud->points.resize (cloud->width * cloud->height);
  //
  // // Generate the data
  // for (size_t i = 0; i < cloud->points.size (); ++i)
  // {
  //   cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
  //   cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
  //   cloud->points[i].z = 1.0;
  // }
  //
  // // Set a few outliers
  // cloud->points[0].z = 2.0;
  // cloud->points[3].z = -2.0;
  // cloud->points[6].z = 4.0;
  //
  // std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
  // for (size_t i = 0; i < cloud->points.size (); ++i)
  //   std::cerr << "    " << cloud->points[i].x << " "
  //                       << cloud->points[i].y << " "
  //                       << cloud->points[i].z << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;

  // std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  // for (size_t i = 0; i < inliers->indices.size (); ++i)
  //   std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
  //                                              << cloud->points[inliers->indices[i]].y << " "
  //                                              << cloud->points[inliers->indices[i]].z << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = simpleVis(cloud);
  viewer->addPlane(*coefficients);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return (0);
}
