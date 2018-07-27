#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data

  cloud.is_dense = false;

  int length = 470;
  int width = 380;
  int height = 265;
  int SCALE_ = 1000;
  // BOTTOM
  // for (int i = -length/2; i < length/2; ++i)
  // {
  //   for (int j = -width/2; j < width/2; j++)
  //   {
  //     pcl::PointXYZ point;
  //     point.x = (float)i/SCALE_;
  //     point.y = (float)j/SCALE_;
  //     point.z = (float)0/SCALE_;
  //     cloud.points.push_back(point);
  //   }
  // }

  // LEFT, RIGHT, FRONT, BACK
  // for (int i = -length/2; i < length/2; ++i)
  // {
  //   for (int k = 0; k < height; k++) {
  //     pcl::PointXYZ point;
  //     point.x = (float)i/SCALE_;
  //     point.y = (float)width/2/SCALE_;
  //     point.z = (float)k/SCALE_;
  //     cloud.points.push_back(point);
  //     point.y = - (float)width/2/SCALE_;
  //     cloud.points.push_back(point);
  //   }
  // }

  // for (int j = -width/2; j < width/2; j++)
  // {
  //   for (int k = 0; k < height; k++) {
  //     pcl::PointXYZ point;
  //     point.x = (float)length/2/SCALE_;
  //     point.y = (float)j/SCALE_;
  //     point.z = (float)k/SCALE_;
  //     cloud.points.push_back(point);
  //     point.x = -(float)length/2/SCALE_;
  //     cloud.points.push_back(point);
  //   }
  // }

  pcl::PointXYZ point;
  point.x = (float)length/2/SCALE_;
  point.y = (float)width/2/SCALE_;
  point.z = (float)height/SCALE_;
  cloud.points.push_back(point);
  point.x = -(float)length/2/SCALE_;
  cloud.points.push_back(point);

  point.x = (float)length/2/SCALE_;
  point.y = -(float)width/2/SCALE_;
  point.z = (float)height/SCALE_;
  cloud.points.push_back(point);
  point.x = -(float)length/2/SCALE_;
  cloud.points.push_back(point);

  point.x = (float)length/2/SCALE_;
  point.y = (float)width/2/SCALE_;
  point.z = 0;
  cloud.points.push_back(point);
  point.x = -(float)length/2/SCALE_;
  cloud.points.push_back(point);

  point.x = (float)length/2/SCALE_;
  point.y = -(float)width/2/SCALE_;
  point.z = 0;
  cloud.points.push_back(point);
  point.x = -(float)length/2/SCALE_;
  cloud.points.push_back(point);



  cloud.width    = cloud.points.size();
  cloud.height   = 1;
  // cloud.points.resize (cloud.width * cloud.height);

  pcl::io::savePCDFileASCII ("box.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to box.pcd." << std::endl;


  return (0);
}
