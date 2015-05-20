#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/common/centroid.h>

// Source https://github.com/OctoMap/octomap_mapping/blob/indigo-devel/octomap_server/src/OctomapServer.cpp

ros::Publisher cloud_filtered_pub;
ros::Publisher cloud_nonground_pub;
ros::Publisher cloud_output_pub;
ros::Publisher cloud_cluster1_pub;
ros::Publisher cloud_cluster2_pub;
ros::Publisher cloud_voxeled_pub;

ros::Time last_time;

namespace find_plane{
  typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
}

using namespace find_plane;

void filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground); 
void findClusters(const PCLPointCloud& pc);
void processCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){
  if(ros::Time::now().toSec()-last_time.toSec()>0.1)
  {
    processCloud(cloud_msg);

    last_time=ros::Time::now();
  }
}

void processCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{

  PCLPointCloud pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud_msg, pc);

  // set up filter for height range
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setFilterFieldName("y");
  pass.setFilterLimits(0.6, 2.);

  PCLPointCloud pc_ground; // segmented ground plane
  PCLPointCloud pc_nonground; // everything else
  pass.setInputCloud(pc.makeShared());
  pass.filter(pc);


  filterGroundPlane(pc, pc_ground, pc_nonground);

  //Reconstruct without ground by taking the negative of passThrough filter
  pass.setNegative(true);
  pass.filter(pc);
  pc+=pc_nonground;

  //Filter far objects for testing
  //pass.setInputCloud()
  pass.setNegative(false);
  pass.setInputCloud(pc.makeShared());
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0., 4.5);
  pass.filter(pc);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 cloud_output_msg;
  pcl::toROSMsg(pc, cloud_output_msg);
  cloud_output_pub.publish(cloud_output_msg);

  // Downsample the dataset with a voxelgrid
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  PCLPointCloud cloud_voxeled;
  vg.setInputCloud (pc.makeShared());
  vg.setLeafSize (0.02f, 0.02f, 0.02f);
  vg.filter(cloud_voxeled);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 cloud_voxeled_msg;
  pcl::toROSMsg(pc, cloud_voxeled_msg);
  cloud_voxeled_pub.publish(cloud_voxeled_msg);

  findClusters(cloud_voxeled);
}

void findClusters(const PCLPointCloud& pc)
{
  
  PCLPointCloud cloud_filtered(pc);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered.makeShared());

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  //ec.setMaxClusterSize (1000000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered.makeShared());
  ec.extract (cluster_indices);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 cloud_cluster_msg;

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    PCLPointCloud::Ptr cloud_cluster (new PCLPointCloud);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered.points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    ROS_INFO_STREAM("PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points.");

    pcl::toROSMsg(*cloud_cluster, cloud_cluster_msg);

    ROS_INFO_STREAM("Indice : " << j);

    cloud_cluster_msg.header.frame_id = "camera_depth_optical_frame";

    //Compute cluster centroid
    Eigen::Vector4f centroid;
    //Eigen::Matrix<float,4,1> centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    ROS_INFO_STREAM("Centroid center x: " << centroid[0] << " y : " << centroid[1] << "z : " << centroid[2]);

    if(j==0)
    {
      cloud_cluster1_pub.publish(cloud_cluster_msg);
      ROS_INFO_STREAM("Published the Cluster: " << j << " Nb points : " << cloud_cluster->points.size () );
    }
    else if(j==1)
    {
      cloud_cluster2_pub.publish(cloud_cluster_msg);
      ROS_INFO_STREAM("Published the Cluster: " << j << " Nb points : " << cloud_cluster->points.size () );
    }
    else if(j>2)
    {
      //ros::Publisher multi_pub();
    }

    j++;
  }

}

void filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground)
{
    ground.header = pc.header;
    nonground.header = pc.header;

    // plane detection for ground plane removal:
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object and set up:
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.05);
    //seg.setAxis(Eigen::Vector3f(0,0,1));  Can be added to accurately find the ground
    //seg.setEpsAngle(m_groundFilterAngle);

    PCLPointCloud cloud_filtered(pc);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 cloud_filtered_msg;
    pcl::toROSMsg(cloud_filtered, cloud_filtered_msg);
    cloud_filtered_pub.publish(cloud_filtered_msg);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    seg.setInputCloud(cloud_filtered.makeShared());
    seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0){
        ROS_INFO("PCL segmentation did not find any plane.");
      }

    extract.setInputCloud(cloud_filtered.makeShared());
    extract.setIndices(inliers);
    extract.setNegative (false);
    extract.filter(ground);

    // remove ground points from full pointcloud:
    // workaround for PCL bug:
    if(inliers->indices.size() != cloud_filtered.size()){
      ROS_INFO("Different indices");

      extract.setNegative(true);
      PCLPointCloud cloud_out;
      extract.filter(cloud_out);
      nonground += cloud_out;
      cloud_filtered = cloud_out;
    }

    //Reconstruct nonground with passTrough output setNegative(true)
    // Convert to ROS data type
    sensor_msgs::PointCloud2 cloud_nonground_msg;
    pcl::toROSMsg(nonground, cloud_nonground_msg);
    cloud_nonground_pub.publish(cloud_nonground_msg);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "cluster_kinect");
  ros::NodeHandle nh;

  last_time=ros::Time::now();

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloudCallback);

  cloud_filtered_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_filtered", 1);
  cloud_nonground_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_nonground", 1);
  cloud_output_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_output", 1);

  cloud_cluster1_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_cluster1", 1);
  cloud_cluster2_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_cluster2", 1);

  cloud_voxeled_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_voxeled", 1);

  // Spin
  ros::spin ();
}