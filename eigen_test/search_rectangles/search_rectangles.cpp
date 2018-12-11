#include "search_rectangles.hpp"

SearchRectangles::SearchRectangles():
candidates_(std::vector<pcl::PointXYZ>()),
box_point_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
box_point_cloud_filtered_(new pcl::PointCloud<pcl::PointXYZ>),
cloud_normals_(new pcl::PointCloud<pcl::Normal>),
coefficients_(new pcl::ModelCoefficients),
inliers_(new pcl::PointIndices),
save_file_path_("/home/damao/Documents/ROS/Goal-directed-Manipulation/output/") {

}

SearchRectangles::SearchRectangles(float resolution):
candidates_(std::vector<pcl::PointXYZ>()),
box_point_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
box_point_cloud_filtered_(new pcl::PointCloud<pcl::PointXYZ>),
cloud_normals_(new pcl::PointCloud<pcl::Normal>),
coefficients_(new pcl::ModelCoefficients),
inliers_(new pcl::PointIndices),
save_file_path_("/home/damao/Documents/ROS/Goal-directed-Manipulation/output/") {
  resolution_ = resolution;
  // candidates_.clear();
}

SearchRectangles::SearchRectangles(float length, float width, float height, float rect_width, float rect_length, float resolution):
candidates_(std::vector<pcl::PointXYZ>()),
box_point_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
box_point_cloud_filtered_(new pcl::PointCloud<pcl::PointXYZ>),
cloud_normals_(new pcl::PointCloud<pcl::Normal>),
coefficients_(new pcl::ModelCoefficients),
inliers_(new pcl::PointIndices),
save_file_path_("/home/damao/Documents/ROS/Goal-directed-Manipulation/output/") {
  // candidates_.clear();
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
  std::cerr << "Filtered point cloud. " << std::endl;
  // initialize box matrix based on point cloud.
  initializeMatrix();
  std::cerr << "Generated matrix. " << std::endl;
  // search planes
  segmentPlanes();
  std::cerr << "Got planes. " << std::endl;
  // search poses on planes
  checkCollision ();
  std::cerr << "Done. " << std::endl;

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
    int x_temp, y_temp;
    double z_temp;
    pointsToMatrix(it->x, it->y, it->z, x_temp, y_temp, z_temp);
    // std::cout << "x,y: " << it->x << " " << it->y << std::endl;
    // std::cout << "x_delta,y_delta: " << l/2 << " " << w/2 << std::endl;
    // std::cout << "x_int,y_int: " << x_temp << " " << y_temp << std::endl;
    m_box(x_temp, y_temp) = std::max(m_box(x_temp, y_temp), z_temp);
  }
  std::ofstream myfile;
  std::string matrix_file_name = save_file_path_ + "matrix.txt";
  myfile.open (matrix_file_name.c_str());
  myfile << m_box;
  myfile.close();
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
  matrix_file_name = save_file_path_ + "matrix_supplement.txt";
  myfile.open (matrix_file_name.c_str());
  myfile << m_box;
  myfile.close();
}

void SearchRectangles::pointsToMatrix(float x, float y, float z, int & x_temp, int & y_temp, double & z_temp) {
  x_temp = (int)((x + length_/2.0)/resolution_);
  y_temp = (int)((y + width_/2.0)/resolution_);
  z_temp = round((z + height_/2.0)/resolution_);
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
  seg.setDistanceThreshold (threshold_);
  // seg.setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
  // seg.setEpsAngle (pcl::deg2rad (5.0));
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
    std::sort(cloud_p->points.begin(), cloud_p->points.end(), SearchRectangles());

    if (coefficients_->values[2] > 0.98) {
      std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
      std::stringstream ss;
      ss << save_file_path_ << "table_scene_lms400_plane_" << i << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
      segment_planes_[i] = *cloud_p;

      /*****************VISUALIZE*******************/
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (int j = 0; j < cloud_p->points.size(); j++) {
        pcl::PointXYZRGB pt;
        pt.x = cloud_p->points[j].x;
        pt.y = cloud_p->points[j].y;
        pt.z = cloud_p->points[j].z;
        pt.r = (int)((float)j / cloud_p->points.size() * 255);
        pt.g = 255;
        pt.b = 255;
        color_pc->points.push_back(pt);
      }
      color_pc->height = 1;
      color_pc->width = color_pc->points.size();
      std::stringstream sss;
      sss << save_file_path_ << "color_table_scene_lms400_plane_" << i << ".pcd";
      writer.write<pcl::PointXYZRGB> (sss.str (), *color_pc, false);
      /*****************VISUALIZE*******************/

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

// bool SearchRectangles::sortPointCloud(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
//   if (pcl::geometry::distance(p1, start_) < pcl::geometry::distance(p2, start_)) {
//     return true;
//   }
//   else {
//     return false;
//   }
// }


bool SearchRectangles::operator() (const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
  if (pcl::geometry::distance(p1, start_) < pcl::geometry::distance(p2, start_)) {
    return true;
  }
  else {
    return false;
  }
}

void SearchRectangles::checkCollision () {
  for (std::map<int, pcl::PointCloud<pcl::PointXYZ> >::iterator it = segment_planes_.begin(); it != segment_planes_.end(); it++) {
      for (pcl::PointCloud<pcl::PointXYZ>::iterator itt = it->second.begin(); itt != it->second.end(); itt++) {
        if (!checkCollisionForOnePoint(*itt)) candidates_.push_back(*itt);
      }
  }
  std::sort(candidates_.begin(), candidates_.end(), SearchRectangles());
}

bool SearchRectangles::checkCollisionForOnePoint(pcl::PointXYZ point) {
  int x_temp, y_temp;
  double z_temp;
  int type = 1;
  int collision_type;
  // bool flag = true;
  pointsToMatrix(point.x + rect_length_/2, point.y + rect_width_/2, point.z, x_temp, y_temp, z_temp);
  // if ( x_temp >= m_l || x_temp < 0 || y_temp >= m_w || y_temp < 0) return true;
  // if ( m_box(x_temp, y_temp) > z_temp + 0.01) return true;
  collision_type = checkCollisionForOnePointOfOnePoint(x_temp, y_temp, z_temp);
  if (collision_type == 1) return true;
  else if (collision_type == 0) type *= 2;

  pointsToMatrix(point.x + rect_length_/2, point.y - rect_width_/2, point.z, x_temp, y_temp, z_temp);
  collision_type = checkCollisionForOnePointOfOnePoint(x_temp, y_temp, z_temp);
  if (collision_type == 1) return true;
  else if (collision_type == 0) type *= 2;

  pointsToMatrix(point.x - rect_length_/2, point.y + rect_width_/2, point.z, x_temp, y_temp, z_temp);
  collision_type = checkCollisionForOnePointOfOnePoint(x_temp, y_temp, z_temp);
  if (collision_type == 1) return true;
  else if (collision_type == 0) type *= 2;

  pointsToMatrix(point.x - rect_length_/2, point.y - rect_width_/2, point.z, x_temp, y_temp, z_temp);
  collision_type = checkCollisionForOnePointOfOnePoint(x_temp, y_temp, z_temp);
  if (collision_type == 1) return true;
  else if (collision_type == 0) type *= 2;

  pointsToMatrix(point.x, point.y, point.z, x_temp, y_temp, z_temp);
  collision_type = checkCollisionForOnePointOfOnePoint(x_temp, y_temp, z_temp);
  if (collision_type == 1) return true;
  else if (collision_type == 0) type *= 3;

  if (type % 3 != 0 && (type / 2) < 3) return true;
  if (type % 3 == 0 && (type / 2) < 2) return true;

  return false;
}

int SearchRectangles::checkCollisionForOnePointOfOnePoint(int x_temp, int y_temp, double z_temp) {
  if ( x_temp >= m_l || x_temp < 0 || y_temp >= m_w || y_temp < 0) return 1;
  if ( m_box(x_temp, y_temp) > z_temp + 0.01) return 1;
  if ( m_box(x_temp, y_temp) + 0.01 > z_temp) return 0;
  return -1;
}


void SearchRectangles::test () {
  std::cout << segment_planes_.size() << std::endl;\
  std::cout << "start_ " << start_ << std::endl;
  for (std::map<int, pcl::PointCloud<pcl::PointXYZ> >::iterator it = segment_planes_.begin(); it != segment_planes_.end(); ++it) {
    std::cout << it->first << " " << it->second.points.size() << std::endl;
    std::cout << it->second.points[0] << " " << it->second.points[1] << std::endl;
  }

  pcl::PCDWriter writer;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr candidates_color(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "size::" << candidates_.size() << std::endl;
  for (int j = 0; j < candidates_.size(); j++) {
    pcl::PointXYZRGB pt;
    pt.x = candidates_[j].x;
    pt.y = candidates_[j].y;
    pt.z = candidates_[j].z;
    pt.r = (int)((float)j / candidates_.size() * 255);
    pt.g = 255;
    pt.b = 255;
    candidates_color->points.push_back(pt);
  }
  candidates_color->height = 1;
  candidates_color->width = candidates_color->points.size();
  std::stringstream sss;
  sss << save_file_path_ << "RESULT" << ".pcd";
  writer.write<pcl::PointXYZRGB> (sss.str (), *candidates_color, false);

}

int main (int argc, char** argv)
{
  float resolution = 0.01;
  float length = 0.37, width = 0.47, height = 0.265;
  float rect_length = 0.1, rect_width = 0.06;

  SearchRectangles SR(length, width, height, rect_width, rect_length, resolution);
  std::string name = argv[1];
  SR.LoadPointCloud(name);
  SR.setStartPoint(-length/2, 0, -height/2);
  float threshold = atof(argv[2]);
  float threshold_weight = atof(argv[3]);
  SR.setThreshold(threshold);
  SR.setThresholdWeight(threshold_weight);
  SR.Run();
  SR.test();
  return (0);
}
