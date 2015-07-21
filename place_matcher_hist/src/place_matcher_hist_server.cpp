/********************************************
 * Server for polygon dissimilarity based on histograms.
 *
 *************************************/

#include <ros/ros.h>

#include <place_matcher_msgs/GetCapability.h>

#include <place_matcher_hist/dissimilarity_getter.h>

bool callback_getCapability(place_matcher_msgs::GetCapabilityRequest& req, place_matcher_msgs::GetCapabilityResponse& res)
{
  res.scale_invariance = place_matcher_msgs::GetCapabilityResponse::IRRELEVANT;
  res.translation_invariance = place_matcher_msgs::GetCapabilityResponse::IRRELEVANT;
  res.rotation_invariance = place_matcher_msgs::GetCapabilityResponse::IRRELEVANT;
  res.provides_dissimilarity = false;
  res.provides_pose = true;
  return true;
}

int main(int argc, char **argv)
{
  int max_thread;
  ros::init(argc, argv, "polygon_dissimilarity_server_hist");
  ros::NodeHandle nh("~");

  nh.param<int>("max_thread", max_thread, 1);

  place_matcher_hist::DissimilarityGetter dissimilarity_getter(nh);

  ros::ServiceServer get_capability_server = nh.advertiseService("get_capability", callback_getCapability);
  ros::ServiceServer compute_dissimilarity_server = nh.advertiseService("compute_dissimilarity",
      &place_matcher_hist::DissimilarityGetter::getDissimilarity, &dissimilarity_getter);

  ROS_INFO("Ready to work (with %i threads)", max_thread);

  ros::MultiThreadedSpinner spinner(max_thread);
  spinner.spin();

  return 0;
}


