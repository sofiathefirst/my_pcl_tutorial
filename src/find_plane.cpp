#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

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
ros::Publisher output_plane_pub;
ros::Publisher output_filtered_pub;
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

	//Then do planar segmentation, coefficients a*x+b*y+c*z+d=0
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

  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obj(new pcl::PointCloud<pcl::PointXYZ>);

  	// Create the filtering object
  	pcl::ExtractIndices<pcl::PointXYZ> extract;
  	extract.setInputCloud(cloud_pcl.makeShared());
  	extract.setIndices(inliers);
  	// Set to return indices or all points except the indices
  	extract.setNegative(false);
  	extract.filter(*cloud_plane);

  	//Get remaining points minus the ground
  	extract.setNegative(true);
  	extract.filter(*cloud_obj);

  	// Convert to ROS data type
	sensor_msgs::PointCloud2 output_plane;
  	pcl::toROSMsg(*cloud_plane, output_plane);

  	sensor_msgs::PointCloud2 output_filtered;
  	pcl::toROSMsg(*cloud_obj, output_filtered);

  	// Publish the data
	output_plane_pub.publish(output_plane);
	output_filtered_pub.publish(output_filtered);

  	publish_marker(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

}

void publish_marker(float a, float b, float c, float d)
{	
	//Eigen::Vector3d up_vector(a, b, c);
	tf::Vector3 norm(a,b,c);
	norm.normalize();

	tf::Vector3 up_vector(0.0, 0.0, 1.0);
	tf::Vector3 right_vector = norm.cross(up_vector);
	right_vector.normalized();

	float cos_theta = norm.dot(up_vector);
	float angle=acos(cos_theta);
	tf::Vector3 vec = norm.cross(up_vector);
	vec.normalize();
	tf::Quaternion q(vec, angle);

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	q.setEuler(0, roll, 0);

	ROS_INFO_STREAM("roll : "<< roll << " pitch : "<< pitch << " yaw : "<<yaw);

	visualization_msgs::Marker marker;
	marker.header.frame_id = "camera_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;

	tf::Vector3 plane_offset;
	plane_offset=d*norm;

	marker.pose.position.x = plane_offset[0];
	marker.pose.position.y = plane_offset[1];
	marker.pose.position.z = plane_offset[2];
	marker.pose.orientation.x = q[0];
	marker.pose.orientation.y = q[1];
	marker.pose.orientation.z = q[2];
	marker.pose.orientation.w = q[3];
	marker.scale.x = 2;
	marker.scale.y = 2;
	marker.scale.z = 0.01;
	marker.color.a = 1.0; // Don't forget to set the alpha!

	vis_pub.publish(marker);
}

int main (int argc, char** argv)
{
	// Initialize ROS
 	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, find_plane);

	output_pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
	output_plane_pub = nh.advertise<sensor_msgs::PointCloud2> ("output_plane", 1);
	output_filtered_pub = nh.advertise<sensor_msgs::PointCloud2> ("output_filtered", 1);

	vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	// Spin
	ros::spin ();
}