#include "search_rectangles.hpp"

SearchRectangles::SearchRectangles():
box_point_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
box_point_cloud_filtered_(new pcl::PointCloud<pcl::PointXYZ>),
cloud_normals_(new pcl::PointCloud<pcl::Normal>),
coefficients_(new pcl::ModelCoefficients),
inliers_(new pcl::PointIndices),
save_file_path_("/home/damao/Documents/ROS/Goal-directed-Manipulation/output/") {
}

SearchRectangles::SearchRectangles(float resolution = 0.01):
box_point_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
box_point_cloud_filtered_(new pcl::PointCloud<pcl::PointXYZ>),
cloud_normals_(new pcl::PointCloud<pcl::Normal>),
coefficients_(new pcl::ModelCoefficients),
inliers_(new pcl::PointIndices),
save_file_path_("/home/damao/Documents/ROS/Goal-directed-Manipulation/output/") {
  resolution_ = resolution;
}

SearchRectangles::SearchRectangles(float length, float width, float height, float rect_width, float rect_length, float resolution = 0.01):
box_point_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
box_point_cloud_filtered_(new pcl::PointCloud<pcl::PointXYZ>),
cloud_normals_(new pcl::PointCloud<pcl::Normal>),
coefficients_(new pcl::ModelCoefficients),
inliers_(new pcl::PointIndices),
save_file_path_("/home/damao/Documents/ROS/Goal-directed-Manipulation/output/") {
  resolution_ = resolution;
  length_ = length;
  m_l = (int)(length_ / resolution_);
  width_ = width;
  m_w = (int)(width_ / resolution_);
  height_ = height;
  m_h = (int)(height_ / resolution_);

  rect_length_ = rect_length;
  rect_width_ = rect_width;
}

void SearchRectangles::Run() {
  // downsample point cloud
  filteredPointCloud();
  // initialize box matrix based on point cloud.
  initializeMatrix();
  // search planes
  segmentPlanes();
  // search poses on planes
}

void SearchRectangles::LoadPointCloud(std::string name) {
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (name, *box_point_cloud_) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return;
  }
  std::cout << "Loaded "
            << box_point_cloud_->width * box_point_cloud_->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
}


void SearchRectangles::filteredPointCloud() {
  // Create the filtering object: downsample the dataset using a leaf size of 1cm

  std::cerr << "PointCloud before filtering: " << box_point_cloud_->width * box_point_cloud_->height << " data points." << std::endl;

  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);

  pcl::toPCLPointCloud2(*box_point_cloud_, *cloud_blob);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  pcl::fromPCLPointCloud2(*cloud_filtered_blob, *box_point_cloud_filtered_);

  std::cerr << "PointCloud after filtering: " << box_point_cloud_filtered_->width * box_point_cloud_filtered_->height << " data points." << std::endl;

}

void SearchRectangles::initializeMatrix() {
  m_box = Eigen::MatrixXd::Constant(m_l, m_w, -1);

  /***************DISCRETIZATION****************/
  std::cout << m_l << " " << m_w << std::endl;
  // std::cout << "size: " << cloud_box_frame->points.size() << std::endl;
  for(pcl::PointCloud<pcl::PointXYZ>::iterator it = box_point_cloud_->begin(); it != box_point_cloud_->end(); it++){
    int x_temp = (int)((it->x + length_/2.0)/resolution_);
    int y_temp = (int)((it->y + width_/2.0)/resolution_);
    double z_temp = round((it->z + height_/2.0)/resolution_);
    // std::cout << "x,y: " << it->x << " " << it->y << std::endl;
    // std::cout << "x_delta,y_delta: " << l/2 << " " << w/2 << std::endl;
    // std::cout << "x_int,y_int: " << x_temp << " " << y_temp << std::endl;
    m_box(x_temp, y_temp) = std::max(m_box(x_temp, y_temp), z_temp);
  }
  // std::ofstream myfile;
  // myfile.open (save_file_path_ + "matrix.txt");
  // myfile << m_box;
  // myfile.close();
  /***************SUPPLEMENT****************/
  int max_h = 0, min_h = INT_MAX;
  for (int i = 0; i < m_box.rows(); i++) {
    if (m_box(i, 0) < 2) m_box(i, 0) = 2;
    for (int j = 0; j < m_box.cols(); j++) {
      if (m_box(i, j) < 2) m_box(i, j) = m_box(i, j-1);
      if (m_box(i, j) > max_h) max_h = m_box(i, j);
      if (m_box(i, j) < min_h) min_h = m_box(i, j);
    }
  }
  // myfile.open (save_file_path_ + "matrix_supplement.txt");
  // myfile << m_box;
  // myfile.close();
}

void SearchRectangles::segmentPlanes() {
  pcl::PCDWriter writer;
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  // pcl::SACSegmentation<pcl::PointXYZ> seg;

  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  // seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
  // seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setMethodType (pcl::SAC_LMEDS);
  seg.setMaxIterations (1000);
  // float threshold = atof(argv[2]);
  // std::cout << threshold << std::endl;
  seg.setDistanceThreshold (threshold_);
  // seg.setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
  // seg.setEpsAngle (pcl::deg2rad (5.0));
  // float threshold_weight = atof(argv[3]);
  seg.setNormalDistanceWeight (threshold_weight_);


  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  int i = 0, nr_points = (int) box_point_cloud_filtered_->points.size ();
  // While 30% of the original cloud is still there
  while (box_point_cloud_filtered_->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (box_point_cloud_filtered_);
    // Estimate point normals
    cloud_normals_->clear();
    ne.setSearchMethod (tree);
    ne.setInputCloud (box_point_cloud_filtered_);
    ne.setKSearch (50);
    ne.compute (*cloud_normals_);
    seg.setInputNormals (cloud_normals_);
    seg.segment (*inliers_, *coefficients_);
    // std::cout << *coefficients << std::endl;

    if (inliers_->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (box_point_cloud_filtered_);
    extract.setIndices (inliers_);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    if (coefficients_->values[2] > 0.98) {
      std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
      std::stringstream ss;
      ss << save_file_path_ << "table_scene_lms400_plane_" << i << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
      segment_planes_[i] = *cloud_p;

      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud_p,centroid);
      std::cout << centroid << std::endl;
      i++;
    }
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    box_point_cloud_filtered_.swap (cloud_f);
  }
}

int main (int argc, char** argv)
{
  float resolution = 0.01;
  float length = 0.37, width = 0.47, height = 0.265;
  float rect_length = 0.1, rect_width = 0.06;

  SearchRectangles SR(length, width, height, rect_width, rect_length, resolution);
  std::string name = argv[1];
  SR.LoadPointCloud(name);
  float threshold = atof(argv[2]);
  float threshold_weight = atof(argv[3]);
  SR.setThreshold(threshold);
  SR.setThresholdWeight(threshold_weight);
  SR.Run();
  // pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // // Fill in the cloud data
  // pcl::PCDReader reader;
  // std::string name = argv[1];
  // reader.read (name, *cloud_blob);
  //
  // std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
  //
  // // Create the filtering object: downsample the dataset using a leaf size of 1cm
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloud_blob);
  // sor.setLeafSize (0.01f, 0.01f, 0.01f);
  // sor.filter (*cloud_filtered_blob);
  //
  // // Convert to the templated PointCloud
  // pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
  //
  // std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
  //
  //
  // // Write the downsampled version to disk
  // pcl::PCDWriter writer;
  // writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);
  //
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // // Create the segmentation object
  // pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  // // pcl::SACSegmentation<pcl::PointXYZ> seg;
  //
  // // Optional
  // seg.setOptimizeCoefficients (true);
  // // Mandatory
  // seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  // // seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
  // // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // // seg.setMethodType (pcl::SAC_LMEDS);
  // seg.setMaxIterations (1000);
  // float threshold = atof(argv[2]);
  // std::cout << threshold << std::endl;
  // seg.setDistanceThreshold (threshold);
  // // seg.setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
  // // seg.setEpsAngle (pcl::deg2rad (5.0));
  // float threshold_weight = atof(argv[3]);
  // seg.setNormalDistanceWeight (threshold_weight);
  //
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
  //   // Estimate point normals
  //   cloud_normals->clear();
  //   ne.setSearchMethod (tree);
  //   ne.setInputCloud (cloud_filtered);
  //   ne.setKSearch (50);
  //   ne.compute (*cloud_normals);
  //   seg.setInputNormals (cloud_normals);
  //   seg.segment (*inliers, *coefficients);
  //   // std::cout << *coefficients << std::endl;
  //
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
  //   if (coefficients->values[2] > 0.98) {
  //     std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
  //     std::stringstream ss;
  //     ss << "table_scene_lms400_plane_" << i << ".pcd";
  //     writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
  //
  //     Eigen::Vector4f centroid;
  //     pcl::compute3DCentroid(*cloud_p,centroid);
  //     std::cout << centroid << std::endl;
  //     i++;
  //   }
  //   // Create the filtering object
  //   extract.setNegative (true);
  //   extract.filter (*cloud_f);
  //   cloud_filtered.swap (cloud_f);
  //
  // }

  return (0);
}
