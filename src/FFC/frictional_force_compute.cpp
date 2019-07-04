#include <iostream>
#include "ros/ros.h"
#include "gpd/GraspConfigList.h"
#include "geometry_msgs/PointStamped.h"
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <vector>  
#include <list> 

int top_n_grasp_used = 10;          // Number of grasp candidates use to compute FFC
float hand_outer_diameter = 0.1;    // Hand configuration
float hand_depth = 0.06;            // Hand configuration
float hand_height = 0.035;          // Hand configuration
float nt = 0.7;                     // Threshold of vertical normals
float h = 0.0025;                   // Threshold of Left and Right offsets

//geometry_msgs::PointStamped R1, R2, R3, R4;
ros::Publisher pub_r1_r4_pc, pub_FFC_pc, pub_FFC_GraspConfig, pub_original_candidate, pub_FFC_candidate;
tf::TransformListener *tf_listener;
geometry_msgs::PointStamped close_region_R1, close_region_R2, close_region_R3, close_region_R4;

// Convert GraspConfig msg to Marker in RVIZ
visualization_msgs::MarkerArray convertToVisualGraspMsg(const gpd::GraspConfig GraspConfig, const std::string& frame_id, const bool color)
{
  // Get parameters of GraspConfig
  int i = 1;
  double finger_width = 0.01;
  double width = hand_outer_diameter;
  double hw = 0.5 * width;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker left_finger, right_finger, base, approach;
  Eigen::Vector3d left_bottom, right_bottom, left_top, right_top, left_center, right_center, approach_center, base_center;

  Eigen::Vector3d g_bottom(GraspConfig.bottom.x, GraspConfig.bottom.y, GraspConfig.bottom.z);
  Eigen::Vector3d g_binormal(GraspConfig.binormal.x, GraspConfig.binormal.y, GraspConfig.binormal.z);
  Eigen::Vector3d g_approach(GraspConfig.approach.x, GraspConfig.approach.y, GraspConfig.approach.z);
  Eigen::Matrix3d g_frame;
  g_frame << GraspConfig.approach.x, GraspConfig.binormal.x, GraspConfig.axis.x,
             GraspConfig.approach.y, GraspConfig.binormal.y, GraspConfig.axis.y,
             GraspConfig.approach.z, GraspConfig.binormal.z, GraspConfig.axis.z;

  // Compute the position of each marker
  left_bottom = g_bottom - (hw - 0.5*finger_width) * g_binormal;
  right_bottom = g_bottom + (hw - 0.5*finger_width) * g_binormal;
  left_top = left_bottom + hand_depth * g_approach;
  right_top = right_bottom + hand_depth * g_approach;
  left_center = left_bottom + 0.5*(left_top - left_bottom);
  right_center = right_bottom + 0.5*(right_top - right_bottom);
  base_center = left_bottom + 0.5*(right_bottom - left_bottom) - 0.01*g_approach;
  approach_center = base_center - 0.04*g_approach;
  
  // Create grasp marker
  visualization_msgs::Marker createFingerMarker(const Eigen::Vector3d& center, const Eigen::Matrix3d& frame, double length, double width, double height, int id, const std::string& frame_id, const bool color);
  visualization_msgs::Marker createHandBaseMarker(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Matrix3d& frame, double length, double height, int id, const std::string& frame_id, const bool color);
  base = createHandBaseMarker(left_bottom, right_bottom, g_frame, 0.02, hand_height, i, frame_id, color);
  left_finger = createFingerMarker(left_center, g_frame, hand_depth, finger_width, hand_height, i*3, frame_id, color);
  right_finger = createFingerMarker(right_center, g_frame, hand_depth, finger_width, hand_height, i*3+1, frame_id, color);
  approach = createFingerMarker(approach_center, g_frame, 0.08, finger_width, hand_height, i*3+2, frame_id, color);

  // Input marker configuration to MarkerArray
  marker_array.markers.push_back(left_finger);
  marker_array.markers.push_back(right_finger);
  marker_array.markers.push_back(approach);
  marker_array.markers.push_back(base);

  return marker_array;
}

// Create the finger part of marker
visualization_msgs::Marker createFingerMarker(const Eigen::Vector3d& center, const Eigen::Matrix3d& frame, double length, double width, double height, int id, const std::string& frame_id, bool color)
{
  // Configuration of Marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "finger";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.lifetime = ros::Duration(20);

  // Use orientation of hand frame
  Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // These scales are relative to the hand frame (unit: meters)
  marker.scale.x = length; // forward direction
  marker.scale.y = width; // hand closing direction
  marker.scale.z = height; // hand vertical direction

  // Determine the color of grasp marker
  if(color == false)
  {
    // Green
    marker.color.a = 0.5;
    marker.color.g = 1.0;
  }
  else
  {
    // Red
    marker.color.a = 0.5;
    marker.color.r = 1.0;
  }
  
  return marker;
}

// Create the base part of marker
visualization_msgs::Marker createHandBaseMarker(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Matrix3d& frame, double length, double height, int id, const std::string& frame_id, bool color)
{
  Eigen::Vector3d center = start + 0.5 * (end - start);

  // Configuration of Marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "hand_base";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.lifetime = ros::Duration(20);

  // Use orientation of hand frame
  Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // These scales are relative to the hand frame (unit: meters)
  marker.scale.x = length; // forward direction
  marker.scale.y = (end - start).norm(); // hand closing direction
  marker.scale.z = height; // hand vertical direction

  // Determine the color of grasp marker
  if(color == false)
  {
    // Green
    marker.color.a = 0.5;
    marker.color.g = 1.0;
  }
  else
  {
    // Red
    marker.color.a = 0.5;
    marker.color.r = 1.0;
  }

  return marker;
}

// Compute the normal and number of points that in R1 - R4 region
int Compute_Normal_and_Number(sensor_msgs::PointCloud2 &R1_R4_region_pc)
{
	// Unit vector of y-axis
  Eigen::Vector3d v2(0, 1, 0);

  // Origin of P frame 
	float x0 = 0, y0 = 0, z0 = 0;

  // Convert ROS PointCloud2 data to PCL PointXYZ
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);

  pcl_conversions::toPCL(R1_R4_region_pc, *cloud);

  pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr(xyz_cloud);

  pcl::fromPCLPointCloud2(*cloudPtr,*xyzCloudPtr);

  // Get points normal
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(xyzCloudPtr);

  // Set up KdTree 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.03);
	ne.compute(*cloud_normals);

	std::vector<std::vector<float> > clound_xyz;
	std::vector<float> point_xyz;
  Eigen::Vector3d p_normal(0, 0, 0);

	float a, b, c, D;
	float cnv_v2 = 0.0;

  D = 0;

	a = v2[0], b = v2[1], c = v2[2];
	
  // Compute the inner product of point normal and vertical vector
  // If the inner product value within threshold and save it
	for (int i = 0; i < cloud_normals->points.size(); i++) 
  {
		p_normal[0] = cloud_normals->points[i].normal_x;
		p_normal[1] = cloud_normals->points[i].normal_y;
		p_normal[2] = cloud_normals->points[i].normal_z;
		cnv_v2 = v2.dot(p_normal);
		if (cnv_v2 > nt || cnv_v2 < -nt) {
			point_xyz.push_back(xyz_cloud->points[i].x);
			point_xyz.push_back(xyz_cloud->points[i].y);
			point_xyz.push_back(xyz_cloud->points[i].z);
			clound_xyz.push_back(point_xyz);
			point_xyz.clear();
		}
	} 

  //std::cout << "Number of Normals: " << clound_xyz.size() << std::endl;

  std::vector<float> distance;
	float x = 0, y = 0, z = 0;
	float max = 0, min = 0; 

  std::list<float> Left_list;
  std::list<float>::iterator Left_list_iter;

  std::list<float> Right_list;
  std::list<float>::iterator Right_list_iter;

  // Find out the point which is longest distance of Left side and Right side in R1 - R4 region
  // Collect the rest of point to Left or Right
	for (int i = 0; i < clound_xyz.size(); i++) 
  {
	  x = clound_xyz[i][0];
		y = clound_xyz[i][1];
		z = clound_xyz[i][2];
		D = a * (x - x0) + b * (y - y0) + c * (z - z0) / pow(a * a + b * b + c * c, 0.5);
		distance.push_back(D);
		
    if (D >= max - h) 
    {
			if (D > max) 
      {
        Right_list.clear();
				Right_list.push_back(i);
        max = D;
			}
			else 
      {
        Right_list.push_back(i);
			}
		}

		if (D <= min + h) 
    {
			if (D < min) 
      {
        Left_list.clear();
        Left_list.push_back(i);
				min = D;
			}
			else 
      {
        Left_list.push_back(i);
			}
		}
	}

	//std::cout << "Left: " << min << std::endl;
	//std::cout << "Right: " << max << std::endl;
	std::cout << "Left_number: " << Left_list.size() << std::endl;
	std::cout << "Right_number: " << Right_list.size() << std::endl;

  // Insert the Left and Right points to pointcloud
  pcl::PointCloud<pcl::PointXYZ> *result_cloud = new pcl::PointCloud<pcl::PointXYZ>;
  
  for(Left_list_iter = Left_list.begin() ; Left_list_iter != Left_list.end() ; Left_list_iter++)
    {
      pcl::PointXYZ point = {clound_xyz[*Left_list_iter][0], clound_xyz[*Left_list_iter][1], clound_xyz[*Left_list_iter][2]};
      result_cloud->push_back(point);
    }

  for(Right_list_iter = Right_list.begin() ; Right_list_iter != Right_list.end() ; Right_list_iter++)
    {
      pcl::PointXYZ point = {clound_xyz[*Right_list_iter][0], clound_xyz[*Right_list_iter][1], clound_xyz[*Right_list_iter][2]};
      result_cloud->push_back(point);
    }
  
  std::cout << "Result_cloud_points_number: " << result_cloud->width*result_cloud->height << std::endl;

  // Convert pointcloud to ROS PointCloud2 and show it in RVIZ
  pcl::PCLPointCloud2 PCL_Result_cloud;
  sensor_msgs::PointCloud2 ROS_Result_cloud;

  pcl::toPCLPointCloud2(*result_cloud ,PCL_Result_cloud);
  pcl_conversions::fromPCL(PCL_Result_cloud, ROS_Result_cloud);
  ROS_Result_cloud.header.stamp = ros::Time::now();
  ROS_Result_cloud.header.frame_id = "p_frame";

  pub_FFC_pc.publish(ROS_Result_cloud);

  // Get the score of FFC
  int FFC_score = Left_list.size()+Right_list.size();
  
  return FFC_score;
  
  // Visualize point normals
  // pcl::visualization::PCLVisualizer viwer("PCL");
  // viwer.setBackgroundColor(0.0, 0.0, 0.0);
  // viwer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(xyzCloudPtr, cloud_normals);

  // while(!viwer.wasStopped())
  // {
  //   viwer.spinOnce();
  // }
  
}

// PassThrough the original pointcloud from camera 
sensor_msgs::PointCloud2 Passthrough_R1_R4(sensor_msgs::PointCloud2 &t_pc)
{
  // Confugration of R1 - R4 region
  double R1_R4_x[4] = {close_region_R1.point.x, close_region_R2.point.x, close_region_R3.point.x, close_region_R4.point.x}; 
  double R1_R4_y[4] = {close_region_R1.point.y, close_region_R2.point.y, close_region_R3.point.y, close_region_R4.point.y};
  double x_max, x_min, y_max, y_min, z_max, z_min;
  x_max = *std::max_element(R1_R4_x, R1_R4_x + 4);
  x_min = *std::min_element(R1_R4_x, R1_R4_x + 4);
  y_max = *std::max_element(R1_R4_y, R1_R4_y + 4);
  y_min = *std::min_element(R1_R4_y, R1_R4_y + 4);
  z_max =  (hand_height/2);
  z_min = -(hand_height/2);

  //std::cout <<"x_min:"<< x_min <<", x_max:"<< x_max <<", y_min:"<< y_min <<", y_max:"<< y_max <<", z_min:"<< z_min <<", z_max:"<< z_max << std::endl;

  // Convert ROS PointCloud2 to PCL PointXYZ
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);

  pcl_conversions::toPCL(t_pc, *cloud);

  pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr(xyz_cloud);

  pcl::fromPCLPointCloud2(*cloudPtr,*xyzCloudPtr);

  // Use PassThrough filter
  // x-axis
  pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered =   new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrFiltered(xyz_cloud_filtered);

  pcl::PassThrough<pcl::PointXYZ> pass;    
  pass.setInputCloud(xyzCloudPtr);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(x_min, x_max);
  pass.filter(*xyzCloudPtrFiltered);

  // y-axis
  pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered2 =  new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrFiltered2(xyz_cloud_filtered2);

  pcl::PassThrough<pcl::PointXYZ> passy;
  passy.setInputCloud(xyzCloudPtrFiltered);
  passy.setFilterFieldName("y");
  passy.setFilterLimits(y_min, y_max);
  passy.filter(*xyzCloudPtrFiltered2);

  // z-axis
  pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered3 =  new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrFiltered3(xyz_cloud_filtered3);

  pcl::PassThrough<pcl::PointXYZ> passz;
  passz.setInputCloud(xyzCloudPtrFiltered2);
  passz.setFilterFieldName("z");
  passz.setFilterLimits(z_min, z_max);
  passz.filter(*xyzCloudPtrFiltered3);

  // Convert to ROS PointCloud2
  pcl::PCLPointCloud2 outputPCL;
  sensor_msgs::PointCloud2 R1_R4_pc;

  pcl::toPCLPointCloud2(*xyzCloudPtrFiltered3 ,outputPCL);
  pcl_conversions::fromPCL(outputPCL, R1_R4_pc);
  R1_R4_pc.header.stamp = ros::Time::now();
  R1_R4_pc.header.frame_id = "p_frame";

  return R1_R4_pc;
}

// Get R1, R2, R3, R4 points position and P frame
void GetR1toR4(gpd::GraspConfig grasps, sensor_msgs::PointCloud2 &t_pc)
{
  // Set up the translation and orientation of P frame from camera frame
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  tf2::Quaternion q;
  tf2::Matrix3x3 R_matrix;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "kinect2_link";
  transformStamped.child_frame_id = "p_frame";
  transformStamped.transform.translation.x = grasps.bottom.x;
  transformStamped.transform.translation.y = grasps.bottom.y;
  transformStamped.transform.translation.z = grasps.bottom.z;
 
  R_matrix.setValue(grasps.approach.x, grasps.binormal.x, grasps.axis.x,
                    grasps.approach.y, grasps.binormal.y, grasps.axis.y,
                    grasps.approach.z, grasps.binormal.z, grasps.axis.z);
  R_matrix.getRotation(q);

  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);

  // Get R1, R2, R3, R4 points position in P frame
  close_region_R1.header.stamp = ros::Time::now();
  close_region_R1.header.frame_id = "p_frame";
  close_region_R1.point.x = 0;
  close_region_R1.point.y = (hand_outer_diameter / 2);

  close_region_R2.header.stamp = ros::Time::now();
  close_region_R2.header.frame_id = "p_frame";
  close_region_R2.point.x = 0;
  close_region_R2.point.y = -(hand_outer_diameter / 2);
      
  close_region_R3.header.stamp = ros::Time::now();
  close_region_R3.header.frame_id = "p_frame";
  close_region_R3.point.x = hand_depth;
  close_region_R3.point.y = (hand_outer_diameter / 2);

  close_region_R4.header.stamp = ros::Time::now();
  close_region_R4.header.frame_id = "p_frame";
  close_region_R4.point.x = hand_depth;
  close_region_R4.point.y = -(hand_outer_diameter / 2);

  // Transform the pointcloud frame from camera to P frame 
  sensor_msgs::PointCloud2 pc;

  try
  {
   
    pc = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/passthrough_output",ros::Duration(3.0)));
    pc.header.frame_id = "kinect2_link";
    //std::cout << "Input pointcloud points number: "<< pc.width * pc.height << std::endl;
    pcl_ros::transformPointCloud("p_frame", pc, t_pc, *tf_listener);
  }
  catch ( ros::Exception &e )
  {
    ROS_ERROR("\nError occured: %s ", e.what());
  }

  // Convert R1_R4 to Kinect2_link frame
  // try
  // {
  //   tf2_ros::Buffer tfBuffer;
  //   tf2_ros::TransformListener tfListener(tfBuffer);

  //   geometry_msgs::TransformStamped p_frame_to_kinect2_link;
      
  //   p_frame_to_kinect2_link = tfBuffer.lookupTransform("kinect2_link", "p_frame", ros::Time::now(), ros::Duration(3.0));
  //   tf2::doTransform(close_region_R1, R1, p_frame_to_kinect2_link);
  //   tf2::doTransform(close_region_R2, R2, p_frame_to_kinect2_link);
  //   tf2::doTransform(close_region_R3, R3, p_frame_to_kinect2_link);
  //   tf2::doTransform(close_region_R4, R4, p_frame_to_kinect2_link);
  // }
  // catch ( ros::Exception &e )
  // {
  //   ROS_ERROR("Error occured: %s ", e.what());
  // }

  // ROS_INFO("r1: %f , %f , %f", R1.point.x, R1.point.y, R1.point.z);
  // ROS_INFO("r2: %f , %f , %f", R2.point.x, R2.point.y, R2.point.z);
  // ROS_INFO("r3: %f , %f , %f", R3.point.x, R3.point.y, R3.point.z);
  // ROS_INFO("r4: %f , %f , %f", R4.point.x, R4.point.y, R4.point.z);  
}

// GraspConfig subscriber callback function
void GraspConfigListCallback(const gpd::GraspConfigList &msg)
{
  std::list<int> FFC_score_list;
  std::list<int>::iterator FFC_score_list_iter;
  
  ROS_INFO("Select top-%d grasps", top_n_grasp_used);

  // Get the R1 - R4 region and FFC score of each GraspConfig
  for(int i = 0; i < top_n_grasp_used; i++)
   {
      // Read GraspConfig
      std::cout << "--- Compute grasp "<< i+1 << " ---" << std::endl;
      gpd::GraspConfig grasps = msg.grasps[i]; 

      // Get the R1 - R4 region
      sensor_msgs::PointCloud2 t_pc;
      sensor_msgs::PointCloud2 R1_R4_region_pc;

      GetR1toR4(grasps, t_pc);
      R1_R4_region_pc = Passthrough_R1_R4(t_pc);
      pub_r1_r4_pc.publish(R1_R4_region_pc);
      std::cout << "R1_R4 pointcloud points number: "<< R1_R4_region_pc.width * R1_R4_region_pc.height << std::endl;
      
      // Compute FFC score
      int FFC_score;
      FFC_score = Compute_Normal_and_Number(R1_R4_region_pc);

      FFC_score_list.push_back(FFC_score);

   }

  // Find out the highest score in the FFC score list and get its index
  FFC_score_list_iter = std::max_element(FFC_score_list.begin(), FFC_score_list.end());
  int index_of_max = std::distance(FFC_score_list.begin(), FFC_score_list_iter);

  std::cout << "--- Computed result --- " << std::endl;
  std::cout << "Index of GraspConfig: "<< index_of_max +1 << std::endl;

  // Publish GraspConfig to robot
  gpd::GraspConfig FFC_Grasp = msg.grasps[index_of_max];
  pub_FFC_GraspConfig.publish(FFC_Grasp);

  // Publish grasp marker to RVIZ
  visualization_msgs::MarkerArray original_candidate_marker, FFC_candidate_marker;

  std::string pc_frame = "kinect2_link";

  original_candidate_marker = convertToVisualGraspMsg(msg.grasps[0], pc_frame, false);
  pub_original_candidate.publish(original_candidate_marker);

  FFC_candidate_marker = convertToVisualGraspMsg(FFC_Grasp, pc_frame, true);
  pub_FFC_candidate.publish(FFC_candidate_marker);
}

int main(int argc, char **argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "frictional_force_compute");
  ros::NodeHandle n;
  tf_listener = new tf::TransformListener();  

  ROS_INFO("Start to compute frictional force of graps candidates.");

  // Subscriber of CNN candidates output
  ros::Subscriber sub_grasps = n.subscribe("/detect_grasps/clustered_grasps", 10, GraspConfigListCallback);

  // Publisher of each data
  pub_r1_r4_pc = n.advertise<sensor_msgs::PointCloud2> ("/r1_r4_pc", 1);
  pub_FFC_pc = n.advertise<sensor_msgs::PointCloud2> ("/FFC_pc", 1);
  pub_FFC_GraspConfig = n.advertise<gpd::GraspConfig> ("/FFC_GraspConfig", 10);
  pub_original_candidate = n.advertise<visualization_msgs::MarkerArray> ("/original_candidate", 1);
  pub_FFC_candidate = n.advertise<visualization_msgs::MarkerArray> ("/FFC_candidate", 1);

  ros::spin();

  return 0;
}
