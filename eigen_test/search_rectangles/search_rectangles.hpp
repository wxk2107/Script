#include <iostream>
#include <Eigen/Dense>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
// #include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/common/impl/angles.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

#include <string>


class SearchRectangles {

public:
  SearchRectangles();
  SearchRectangles(float resolution);
  SearchRectangles(float length, float width, float height, float rect_width, float rect_length, float resolution);

  void Run();

  void setResolution(float resolution) {
    resolution = resolution_;
  }
  void setLength(float length) {
    length_ = length;
    m_l = (int)(length_ / resolution_);
  }
  void setWidth(float width) {
    width_ = width;
    m_w = (int)(width_ / resolution_);
  }
  void setHeight(float height) {
    height_ = height;
    m_h = (int)(height_ / resolution_);
  }

  void setRectLength(float rect_length) {
    rect_length_ = rect_length;
  }

  void setRectWidth(float rect_width) {
    rect_width_ = rect_width;
  }

  void setSaveFilePath(std::string path) {
    save_file_path_ = path;
  }

  void LoadPointCloud(std::string name);

  void setThreshold(float threshold) {
    threshold_ = threshold;
  }

  void setThresholdWeight(float threshold_weight) {
    threshold_weight_ = threshold_weight;
  }
  
private:
  void filteredPointCloud();

  void initializeMatrix();

  void segmentPlanes();

  Eigen::MatrixXd m_box;
  float resolution_;
  float length_, width_, height_;
  float rect_length_, rect_width_;
  int m_l, m_w, m_h;

  pcl::PointCloud<pcl::PointXYZ>::Ptr box_point_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr box_point_cloud_filtered_;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;// (new pcl::PointCloud<pcl::Normal>);

  std::map<int, pcl::PointCloud<pcl::PointXYZ> > segment_planes_;
  pcl::ModelCoefficients::Ptr coefficients_;
  pcl::PointIndices::Ptr inliers_;
  float threshold_;
  float threshold_weight_;

  std::string save_file_path_;
};
