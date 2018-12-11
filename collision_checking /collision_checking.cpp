#include "collision_checking.hpp"


std::vector<std::string> split(std::string str, char delimiter) {
	std::vector<std::string> internal;
	std::stringstream ss(str); // Turn the string into a stream.
	std::string tok;

	while(getline(ss, tok, delimiter)) {
		internal.push_back(tok);
	}

	return internal;
}


bool gripperPoseChecker (Eigen::Affine3f& transform_gripper2world_affine, pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr gripper_local_cloud) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr object_scene_cloud_(new pcl::PointCloud<pcl::PointXYZ> ());;

  pcl::CropBox<pcl::PointXYZ> cropFilter(true);
  cropFilter.setInputCloud(world_cloud);
  cropFilter.setMax(Eigen::Vector4f(0.05, 0.08, 0.05, 1.0f));
  cropFilter.setMin(Eigen::Vector4f(-0.18, -0.08, -0.05, 1.0f));

  Eigen::Matrix3f m;
  m <<  transform_gripper2world_affine.matrix()(0,0), transform_gripper2world_affine.matrix()(0,1), transform_gripper2world_affine.matrix()(0,2),
          transform_gripper2world_affine.matrix()(1,0), transform_gripper2world_affine.matrix()(1,1), transform_gripper2world_affine.matrix()(1,2),
          transform_gripper2world_affine.matrix()(2,0), transform_gripper2world_affine.matrix()(2,1), transform_gripper2world_affine.matrix()(2,2);
  Eigen::Vector3f boxTranslation(transform_gripper2world_affine.matrix()(0,3), transform_gripper2world_affine.matrix()(1,3), transform_gripper2world_affine.matrix()(2,3));
  Eigen::Vector3f boxXYZRotation = m.eulerAngles(2,1,0);

  cropFilter.setTranslation(boxTranslation);
  cropFilter.setRotation(Eigen::Vector3f(boxXYZRotation[2], boxXYZRotation[1], boxXYZRotation[0])); // set Rotation in order of X Y Z.
  // cropFilter.setTransform(transform_gripper2world_affine);
  // std::cout << cropFilter.getTransform().matrix() << std::endl;
  cropFilter.setNegative(false);
  cropFilter.filter(*object_scene_cloud_);
  std::string save_file_path = "/home/damao/Documents/ROS/Goal-directed-Manipulation/output/";
  // pcl::io::savePCDFileASCII (save_file_path +  "object_scene_cloud_.pcd", *object_scene_cloud_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr gripper_pose_world(new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud(*gripper_local_cloud, *gripper_pose_world, transform_gripper2world_affine.matrix());
  pcl::io::savePCDFileASCII (save_file_path + "gripper_pose_world.pcd", *gripper_pose_world);
  std::vector<tf::Vector3> palm_vectors, left_finger_vectors, right_finger_vectors;

  ConstructVectorsForGripperPart(palm_vectors, 0, gripper_pose_world);
  ConstructVectorsForGripperPart(left_finger_vectors, 8, gripper_pose_world);
  ConstructVectorsForGripperPart(right_finger_vectors, 16, gripper_pose_world);

  int count = 0;

  for(int i = 0; i < object_scene_cloud_->points.size(); i++)
  {
      if (isnan(object_scene_cloud_->points[i].x) || isnan(object_scene_cloud_->points[i].y) || isnan(object_scene_cloud_->points[i].z) )
      {
    //std::cout << "a weird point here " << std::endl;
          continue;
      }

      tf::Vector3 point(object_scene_cloud_->points[i].x, object_scene_cloud_->points[i].y, object_scene_cloud_->points[i].z);
      /******************************************************/
      if (!PointToGripperCollisionChecking(point, palm_vectors, left_finger_vectors, right_finger_vectors))
      {
          count ++;
      }
      // std::cout << "count: " << count << std::endl;
      if (count > 100)
      {
          //std::cout << "a collision pose " << std::endl;
          return false;
      }
  }
  // for(int i = 0; i < 4; i++)
  // {
  //     std::cout << palm_vectors[i].getX() << " " << palm_vectors[i].getY() << " " << palm_vectors[i].getZ() << std::endl;
  //     std::cout << left_finger_vectors[i].getX() << " " << left_finger_vectors[i].getY() << " " << left_finger_vectors[i].getZ() << std::endl;
  //     std::cout << right_finger_vectors[i].getX() << " " << right_finger_vectors[i].getY() << " " << right_finger_vectors[i].getZ() << std::endl;
  // }
  // std::cout << transform_gripper2world_affine.matrix() << std::endl;
  // std::cout << "world cloud points: " << world_cloud->points.size() <<  " points: " << object_scene_cloud_->points.size() << " count: " << count << std::endl;
  return true;
}

void ConstructVectorsForGripperPart(std::vector<tf::Vector3>& gripper_part, int start_index,
                                                      pcl::PointCloud<pcl::PointXYZ>::Ptr gripper_pose_world)
{
    tf::Vector3 v0;
    v0.setX(gripper_pose_world->points[start_index].x);
    v0.setY(gripper_pose_world->points[start_index].y);
    v0.setZ(gripper_pose_world->points[start_index].z);

    tf::Vector3 v1;
    v1.setX(gripper_pose_world->points[start_index + 1].x - gripper_pose_world->points[start_index].x);
    v1.setY(gripper_pose_world->points[start_index + 1].y - gripper_pose_world->points[start_index].y);
    v1.setZ(gripper_pose_world->points[start_index + 1].z - gripper_pose_world->points[start_index].z);

    tf::Vector3 v2;
    v2.setX(gripper_pose_world->points[start_index + 2].x - gripper_pose_world->points[start_index].x);
    v2.setY(gripper_pose_world->points[start_index + 2].y - gripper_pose_world->points[start_index].y);
    v2.setZ(gripper_pose_world->points[start_index + 2].z - gripper_pose_world->points[start_index].z);

    tf::Vector3 v3;
    v3.setX(gripper_pose_world->points[start_index + 3].x - gripper_pose_world->points[start_index].x);
    v3.setY(gripper_pose_world->points[start_index + 3].y - gripper_pose_world->points[start_index].y);
    v3.setZ(gripper_pose_world->points[start_index + 3].z - gripper_pose_world->points[start_index].z);

    gripper_part.push_back(v0);
    gripper_part.push_back(v1);
    gripper_part.push_back(v2);
    gripper_part.push_back(v3);
}

bool PointToGripperCollisionChecking(tf::Vector3 point, std::vector<tf::Vector3> palm_vectors,
                                                       std::vector<tf::Vector3> left_finger_vectors,
                                                       std::vector<tf::Vector3> right_finger_vectors)
{
    // Create the vector A from the origin to the point, project the vector A to each of the three vectors (B,C,D)
    // in the palm/left finger/right finger. Check if the projection is positive and within the length of the (B, C, D)

    // Test with palm
    bool collide_with_palm = true;
    tf::Vector3 vec = point - palm_vectors[0];
    for(int i = 1; i < 4; i++)
    {


        float dot_res = vec.dot(palm_vectors[i]);
        if ( dot_res < 0 || dot_res  > palm_vectors[i].length2() )
        {
            collide_with_palm = false;
            break;
        }
    }

    // Test with left finger
    bool collide_with_left_finger = true;
    vec = point - left_finger_vectors[0];
    for(int i = 1; i < 4; i++)
    {
        float dot_res = vec.dot(left_finger_vectors[i]);
        if ( dot_res < 0 || dot_res > left_finger_vectors[i].length2() )
        {
            collide_with_left_finger = false;
            break;
        }
    }

    // Test with right finger
    bool collide_with_right_finger = true;
    vec = point - right_finger_vectors[0];
    for(int i = 1; i < 4; i++)
    {
        float dot_res = vec.dot(right_finger_vectors[i]);
        if ( dot_res < 0 || dot_res > right_finger_vectors[i].length2() )
        {
            collide_with_right_finger = false;
            break;
        }
    }


    if ( collide_with_palm || collide_with_left_finger || collide_with_right_finger )
    {

        return false;
    }

    return true;
}


void ReadGripperLocalPose(std::string pose_file, pcl::PointCloud<pcl::PointXYZ>::Ptr gripper_local_cloud)
{
    // std::string pose_file = gripper_model_path_ + "gripper/gripper_local.txt";
	  std::cout << "Loading gripper pose file: " << pose_file.c_str() << std::endl;
    std::ifstream infile(pose_file.c_str(), std::ifstream::in);
    std::string line;

    std::vector<std::vector<std::string> > fetch_gripper;

    // Read palm pose
    std::getline(infile, line);

    for(int i = 0; i < 8; i++)
    {
        std::getline(infile, line);
        std::vector<std::string> tokens = split(line, ' ');

        fetch_gripper.push_back(tokens);
    }

    // Read left finger pose
    std::getline(infile, line);

    for(int i = 0; i < 8; i++)
    {
        std::getline(infile, line);
        std::vector<std::string> tokens = split(line, ' ');

        fetch_gripper.push_back(tokens);
    }


    // Read right finger pose
    std::getline(infile, line);

    for(int i = 0; i < 8; i++)
    {
        std::getline(infile, line);
        std::vector<std::string> tokens = split(line, ' ');

        fetch_gripper.push_back(tokens);
    }

    // Store them to a pointcloud
    for(int i = 0; i < 24; i++)
    {
        pcl::PointXYZ p;
        p.x = atof(fetch_gripper[i][0].c_str());
        p.y = atof(fetch_gripper[i][1].c_str());
        p.z = atof(fetch_gripper[i][2].c_str());

        gripper_local_cloud->points.push_back(p);
    }

    gripper_local_cloud->height = 1;
    gripper_local_cloud->width = gripper_local_cloud->points.size();
}
