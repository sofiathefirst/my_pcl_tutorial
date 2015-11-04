int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "cluster_kinect");
  ros::NodeHandle nh;

  last_time=ros::Time::now();

  ROS_INFO("Node initialized");

  while()
  {
    
  }

  // Spin
  ros::spin ();
}