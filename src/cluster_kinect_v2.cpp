#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

// Source http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction
// Added cloud publishing and passThrough filter
// This version is using path through filter before finding the ground, so we can restrict the ground finding area

ros::Publisher cloud_filtered_pub;
ros::Publisher cloud_plane_pub;
ros::Publisher cloud_noground_pub;
ros::Publisher cloud_cluster1_pub;
ros::Publisher cloud_cluster2_pub;

void processCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{ 

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);


  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  /*
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
  */

  pcl::PassThrough<pcl::PointXYZ> pass;
  /*
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");  //Axes in camera_depth_optical_frame (Check in Rviz for orientation)
  pass.setFilterLimits (0.0, 4.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  */

  // Reuse PassThrough filter but change Filter Limits
  pass.setInputCloud (cloud); //cloud_filtered before
  pass.setFilterFieldName("y");
  pass.setFilterLimits (0.6, 2.);
  pass.filter(*cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 cloud_filtered_msg;
  pcl::toROSMsg(*cloud_filtered, cloud_filtered_msg);
  cloud_filtered_pub.publish(cloud_filtered_msg);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.05);
  // Can be useful to directly to add more constraints to the segmentation
  //     seg.setAxis(Eigen::Vector3f(0,0,1));
  //     seg.setEpsAngle(m_groundFilterAngle);
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }
  else if(inliers->indices.size () < 10000)
  {
    ROS_INFO("The plane found might not be the ground plane!");
  }

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);

  // Get the points associated with the planar surface
  extract.filter (*cloud_plane);
  ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points.");

  // Convert to ROS data type
  sensor_msgs::PointCloud2 cloud_plane_msg;
  pcl::toROSMsg(*cloud_plane, cloud_plane_msg);
  cloud_plane_pub.publish(cloud_plane_msg);

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_f);
  *cloud_filtered = *cloud_f;

  // Convert to ROS data type
  sensor_msgs::PointCloud2 cloud_noground_msg;
  pcl::toROSMsg(*cloud_filtered, cloud_noground_msg);
  cloud_noground_pub.publish(cloud_noground_msg);

  /*

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 cloud_cluster_msg;

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    ROS_INFO_STREAM("PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points.");

    pcl::toROSMsg(*cloud_cluster, cloud_cluster_msg);

    ROS_INFO_STREAM("Indice : " << j);

    cloud_cluster_msg.header.frame_id = "camera_depth_optical_frame";

    if(j==0)
    {
      cloud_cluster1_pub.publish(cloud_cluster_msg);
      ROS_INFO_STREAM("Published the Cluster: " << cloud_cluster->points.size () );
    }
    else if(j==1)
    {
      cloud_cluster2_pub.publish(cloud_cluster_msg);
    }

    j++;
  }
  */

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "cluster_kinect");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, processCloud);

  cloud_filtered_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_filtered", 1);

  cloud_plane_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_plane", 1);
  cloud_noground_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_noground", 1);

  cloud_cluster1_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_cluster1", 1);
  cloud_cluster2_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_cluster2", 1);

  // Spin
  ros::spin ();
}