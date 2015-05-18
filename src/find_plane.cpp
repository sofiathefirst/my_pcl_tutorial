#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher output_pub;
ros::Publisher output_ex_pub;
ros::Publisher vis_pub;

void publish_marker(float a, float b, float c, float d);

void find_plane(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.1, 0.1, 0.1);
	sor.filter(cloud_filtered);

	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::moveFromPCL(cloud_filtered, output);

	// Publish the data
	output_pub.publish (output);

	//Then do planar segmentation
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;   // pcl::PointXYZ
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.05);

	pcl::PointCloud<pcl::PointXYZ> cloud_pcl;
	pcl::fromROSMsg(*cloud_msg, cloud_pcl);

	seg.setInputCloud (cloud_pcl.makeShared());
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	}

	//Output model coefficient
	ROS_INFO_STREAM("Model coefficients: " << coefficients->values[0] << " " 
	                                  << coefficients->values[1] << " "
	                                  << coefficients->values[2] << " " 
	                                  << coefficients->values[3]);

  	ROS_INFO_STREAM("Model inliers: " << inliers->indices.size ());

  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ex(new pcl::PointCloud<pcl::PointXYZ>);
  	// Create the filtering object
  	pcl::ExtractIndices<pcl::PointXYZ> extract;
  	extract.setInputCloud(cloud_pcl.makeShared());
  	extract.setIndices(inliers);
  	extract.setNegative(false);
  	extract.filter(*cloud_ex);

  	// Convert to ROS data type
	sensor_msgs::PointCloud2 output_ex;
  	pcl::toROSMsg(*cloud_ex, output_ex);

  	// Publish the data
	output_ex_pub.publish (output_ex);
	

  	publish_marker(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

}

void publish_marker(float a, float b, float c, float d)
{	
	Eigen::Vector3d up_vector(a, b, c);

	visualization_msgs::Marker marker;
	marker.header.frame_id = "camera_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 1;
	marker.pose.position.y = 1;
	marker.pose.position.z = d;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 0.01;
	marker.color.a = 1.0; // Don't forget to set the alpha!

	vis_pub.publish(marker);
}

int main (int argc, char** argv)
{
	// Initialize ROS
 	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, find_plane);

	// Create a ROS publisher for the output point cloud
	output_pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
	output_ex_pub = nh.advertise<sensor_msgs::PointCloud2> ("output_ex", 1);

	vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	// Spin
	ros::spin ();
}