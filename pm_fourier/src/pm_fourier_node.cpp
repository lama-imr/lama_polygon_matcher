#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pm_fourier/fourier_dist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pm_fourier_dist_server");
  ros::NodeHandle nh("~");

  FourierDistance fdist(nh);
  ros::ServiceServer service = nh.advertiseService("compute_dissimilarity", &FourierDistance::dissimilarity, &fdist);

  int max_thread;
  nh.param<int>("max_thread", max_thread, 2);

  ROS_INFO("Ready to work (with %i threads)", max_thread);
  /* ros::Publisher pub = global_nh.advertise<std_msgs::String>("node_register", 10, true); */

  /* std_msgs::String msg; */
  /* msg.data = ros::this_node::getName(); */
  /* pub.publish(msg); */

  ros::MultiThreadedSpinner spinner(max_thread); 
  spinner.spin();

  return 0;
}


