#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pm_fourier/fourier_dist.h>

int main(int argc, char **argv)
{
  int max_thread;

  ros::init(argc, argv, "pm_fourier_dist_server");
  ros::NodeHandle n;
  FourierDistance fdist(n);
  // ros::ServiceServer service = n.advertiseService("pm_fourier_similarity", &FourierDistance::dissimilarity,&fdist);
  ros::ServiceServer service = n.advertiseService(ros::this_node::getName(), &FourierDistance::dissimilarity, &fdist);
  n.param<int>("max_thread", max_thread, 2);
  ROS_INFO("%s: ready to work (with %i threads)", ros::this_node::getName().c_str(), max_thread);
  ros::Publisher pub = n.advertise<std_msgs::String>("node_register", 10, true);

  std_msgs::String msg;
  msg.data = ros::this_node::getName();
  pub.publish(msg);
  ros::spin();

  ros::MultiThreadedSpinner spinner(max_thread); 
  spinner.spin();

  return 0;
}


