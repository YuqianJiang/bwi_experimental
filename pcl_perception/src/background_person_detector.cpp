/*
 *
 * 
 * 
 * 
 * Implements the people detection algorithm described here:
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */
 
#include <signal.h> 
#include <vector>
#include <string>
#include <ros/ros.h>

#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <pcl_ros/impl/transforms.hpp>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>
#include <pcl/filters/crop_box.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//some custom functions
#include "utils/file_io.h"
#include "utils/viz_utils.h" 
#include "utils/pcl_utils.h"
 

 
//some constants
bool visualize = false;
bool calibrate_plane = false;

const std::string data_topic = "nav_kinect/depth_registered/points"; 
const std::string classifier_location = "/home/bwi/catkin_ws/src/bwi_experimental/pcl_perception/data/classifier.yaml";
const std::string node_name = "segbot_people_detector";

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

//refresh rate
double ros_rate = 10.0;


Eigen::VectorXf ground_coeffs;


// Mutex: //
boost::mutex cloud_mutex;


bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);
PointCloudT::Ptr person_cloud (new PointCloudT);
sensor_msgs::PointCloud2 person_cloud_ros;

void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	cloud_mutex.lock (); 
	
	//convert to PCL format
	pcl::fromROSMsg (*input, *cloud);

	//state that a new cloud is available
	new_cloud_available_flag = true;
	
	cloud_mutex.unlock ();
}

struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "segbot_background_person_detector");
	ros::NodeHandle nh;
	
	
	nh.param<bool>("background_person_detector/visualize", visualize, false);
	nh.param<double>("background_person_detector/rate", ros_rate, 10.0);
	
	string param_topic;
	nh.param<std::string>(std::string("background_person_detector/rgbd_topic"), param_topic, data_topic);
	
	string param_classifier;
	nh.param<std::string>(std::string("background_person_detector/classifier_location"), 
							param_classifier, 
							ros::package::getPath("pcl_perception")+"/data/classifier.yaml");
	
	
	string param_sensor_frame_id;
	nh.param<std::string>(std::string("background_person_detector/sensor_frame_id"), 
							param_sensor_frame_id, 
							"/nav_kinect_rgb_optical_frame");
	
	
	//nh.getParam("background_person_detector/rgbd_topic", data_topic);
	
	
	//initialize marker publisher
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("segbot_pcl_person_detector/marker", 10);
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("segbot_pcl_person_detector/human_poses", 10);
	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("segbot_pcl_person_detector/human_clouds", 10);
	  
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe (param_topic, 1, cloud_cb);

	// Algorithm parameters:
	std::string svm_filename = param_classifier;
	float min_confidence = -1.5;//-1.9
	float min_height = 1.3;
	float max_height = 2.3;
	float voxel_size = 0.06;
	Eigen::Matrix3f rgb_intrinsics_matrix;
	rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

	//register ctrl-c
	signal(SIGINT, sig_handler);

	//load ground plane coeffs
	ground_coeffs.resize(4);
	string plane_coefs_location = ros::package::getPath("pcl_perception")+"/data/ground_plane_avg.txt";
	ground_coeffs = load_vector_from_file(plane_coefs_location.c_str(),4);

	
	// Initialize new viewer:
	pcl::visualization::PCLVisualizer *viewer_display;          // viewer initialization
	if (visualize){
		viewer_display = new pcl::visualization::PCLVisualizer("People Viewer"); 
		viewer_display->setCameraPosition(0,0,-2,0,-1,0,0);
	}

	// Create classifier for people detection: 
	pcl::people::PersonClassifier<pcl::RGB> person_classifier;
	person_classifier.loadSVMFromFile(param_classifier);   // load trained SVM

	// People detection app initialization:
	pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
	people_detector.setVoxelSize(voxel_size);                        // set the voxel size
	people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
	people_detector.setClassifier(person_classifier);                // set person classifier
	people_detector.setHeightLimits(min_height, max_height);         // set person classifier
//  people_detector.setSensorPortraitOrientation(true);             // set sensor orientation to vertical

	// For timing:
	static unsigned count = 0;
	static double last = pcl::getTime ();

	//
	int detection_count=0;
	bool set_ground = false;
	
	ros::Rate r(ros_rate);

	tf::TransformListener listener;
	tf::StampedTransform transform;

	// Main loop:
	while (!g_caught_sigint && ros::ok())
	{
		//collect messages
		ros::spinOnce();
		
		r.sleep();
		
		if (new_cloud_available_flag && cloud_mutex.try_lock ())    // if a new cloud is available
		{
			new_cloud_available_flag = false;

			// Perform people detection on the new cloud:
			std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
			std::vector<pcl::people::PersonCluster<PointT> > clusters_filtered;
			people_detector.setInputCloud(cloud);
			people_detector.setGround(ground_coeffs);    
			
			people_detector.compute(clusters);                           // perform people detection

			ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

			// Draw cloud and people bounding boxes in the viewer:
			if (visualize){
				viewer_display->removeAllPointClouds();
				viewer_display->removeAllShapes();
			}
			
			
			pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
			
			if (visualize){
				viewer_display->addPointCloud<PointT> (cloud, rgb, "input_cloud");
			}
			
			
			//prepare vizualization message
			visualization_msgs::MarkerArray markers_msg;
			
			unsigned int k = 0;
			for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
			{
				if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
				{
					
					Eigen::Vector3f centroid_k = it->getCenter();
					Eigen::Vector3f top_k = it->getTop();
					Eigen::Vector3f bottom_k = it->getBottom();	
						
					//calculate the distance from the centroid of the cloud to the plane
					pcl::PointXYZ p_k;
						
					p_k.x=bottom_k(0);p_k.y=bottom_k(1);p_k.z=bottom_k(2);
					double dist_to_ground_bottom = pcl::pointToPlaneDistance(p_k,ground_coeffs);
							
					p_k.x=top_k(0);p_k.y=top_k(1);p_k.z=top_k(2);
					double dist_to_ground_top = pcl::pointToPlaneDistance(p_k,ground_coeffs);
							
					p_k.x=centroid_k(0);p_k.y=centroid_k(1);p_k.z=centroid_k(2);
					double dist_to_ground = pcl::pointToPlaneDistance(p_k,ground_coeffs);
							
					/*ROS_INFO("Cluter centroid: %f, %f, %f",centroid_k(0),centroid_k(1),centroid_k(2));
					ROS_INFO("\tDistance to ground (top): %f",dist_to_ground_top);
					ROS_INFO("\tDistance to ground (centroid): %f",dist_to_ground);
					ROS_INFO("\tDistance to ground (bottom): %f",dist_to_ground_bottom);
					ROS_INFO("\tCluster height: %f",it->getHeight());
					ROS_INFO("\tCluster points: %i",it->getNumberPoints());
					ROS_INFO("\tDistance from sensor: %f",it->getDistance());	
					ROS_INFO("\tconfidence: %f",it->getPersonConfidence());	*/

					bool accept = true;
						
					if (it->getNumberPoints() < 250) //a person should have about 350 points +- 50 depending on distance from kinect
						accept = false;
					else if (it->getNumberPoints() > 600) //a person should have about 450 points +- 50 depending on distance from kinect
						accept = false;
					else if (it->getHeight() < 1.1) //nobody should be shorter than a meter and 10 cm
						accept = false;
					else if (it->getHeight() > 2.2) //or taller than 2.2 meters
						accept = false;
					if (dist_to_ground_bottom > 0.3) //or hovering more than 30 cm over the floor
						accept = false;
						
							
					if (accept){
					
					
						// draw theoretical person bounding box in the PCL viewer:
						if (visualize)
							it->drawTBoundingBox(*viewer_display, k);
						
						//get just the person out of the whole cloud
						pcl_utils::applyBoxFilter(it->getMin(), it->getMax(),cloud,person_cloud);
						
						//publish person cloud
						pcl::toROSMsg(*person_cloud,person_cloud_ros);
						person_cloud_ros.header.frame_id = param_sensor_frame_id;
						
						
						cloud_pub.publish(person_cloud_ros);
						
						
						stringstream ss;
						ss << ros::package::getPath("pcl_perception") << "/data/human_cloud" << count << ".pcd";
						pcl::io::savePCDFileASCII (ss.str(), *person_cloud);
						count ++;
						
						//demean
						/*Eigen::Vector4f centroid;
						pcl::compute3DCentroid (*person_cloud, centroid);
						pcl::PointCloud<PointT>::Ptr cloud_demean (new pcl::PointCloud<PointT>);
						pcl::demeanPointCloud<PointT> (*cloudOut, centroid, *cloud_demean);
						
							
						pcl::io::savePCDFileASCII (ros::package::getPath("pcl_perception")+"/data/cropbox.pcd", *cloud_demean);*/
						
						//extract point cloud
						/*pcl::PointIndices inliers = it->getIndices();
						pcl::ExtractIndices<PointT> extract;
						
						extract.setInputCloud (people_detector.getFilteredCloud());
						extract.setIndices ( boost::shared_ptr<pcl::PointIndices>( 
																		new pcl::PointIndices( inliers ) ) );
							
						extract.setNegative (true);
						
						//this cloud will now contain everything but the person
						pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>);
						extract.filter (*cloud_p);
						
						pcl::io::savePCDFileASCII (ros::package::getPath("pcl_perception")+"/data/filtered_negative.pcd", *cloud_p);
				*/
				
				
				
						//ROS_INFO("%f, %f, %f",centroid_k(0),centroid_k(1),centroid_k(2));
						//ROS_INFO("%f",dist_to_ground);

						//transforms the pose into /map frame
						geometry_msgs::Pose pose_i;
						pose_i.position.x=centroid_k(0);
						pose_i.position.y=0.5;
						pose_i.position.z=centroid_k(2);
						pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-3.14/2);
						
						
						geometry_msgs::PoseStamped stampedPose;

						stampedPose.header.frame_id = param_sensor_frame_id;
						stampedPose.header.stamp = ros::Time(0);
						stampedPose.pose = pose_i;
						
						geometry_msgs::PoseStamped stampOut;
						listener.waitForTransform(param_sensor_frame_id, "/map", ros::Time(0), ros::Duration(3.0)); 
						listener.transformPose("/map", stampedPose, stampOut);
						
						
						//transform the human point cloud
						pcl_ros::transformPointCloud ("/map", person_cloud_ros, person_cloud_ros, listener);
						
						//get the actual transform
						/*tf::StampedTransform transform_to_map;
						listener.lookupTransform ("/map", param_sensor_frame_id,  ros::Time(0), transform_to_map);
						geometry_msgs::TransformStamped transform_to_map_msg;
						tf::transformStampedTFToMsg (transform_to_map, transform_to_map_msg)
						
						transformPointCloud ("/map", const tf::Transform &net_transform, const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out)*/
						
						stampOut.pose.position.z = 0.7;
						stampOut.header.stamp = ros::Time::now();
						
						//publish the marker
						visualization_msgs::Marker marker_k = create_next_person_marker(it,"/map","segbot_pcl_person_detector",detection_count);	
						marker_k.pose = stampOut.pose;
						marker_pub.publish(marker_k);
						
						//publish the pose
						stampOut.pose.position.z = 0.0;
						pose_pub.publish(stampOut);
		
						k++;
						
						detection_count++;
					}	
				}	
			}
						
			if (visualize){
				viewer_display->spinOnce();
			}

			cloud_mutex.unlock ();
		}
	}

	return 0;
}

