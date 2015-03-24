#include <ros/ros.h>

#include <place_matcher_msgs/PolygonDissimilarity.h>

bool dissimilarity(place_matcher_msgs::PolygonDissimilarity::Request  &req,
    place_matcher_msgs::PolygonDissimilarity::Response &res)
{
  float diff = 0;
  ros::Time start = ros::Time::now();
  ROS_DEBUG("request: polygon1.size = %zu, polygon2.size = %zu", req.polygon1.points.size(), req.polygon2.points.size());
  for (size_t i = 0; i < req.polygon1.points.size(); i++)
  {
    for (size_t j = 0; j < req.polygon2.points.size(); j++)
    {
      diff += req.polygon1.points[i].x - req.polygon2.points[j].x;
    }
  }
  res.raw_dissimilarity = diff / (req.polygon1.points.size() + req.polygon2.points.size());
  res.processing_time = ros::Time::now() - start;
  ROS_DEBUG("sending back response in %f s", res.processing_time.toSec());

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_polygon_dissimilarity_server");
  ros::NodeHandle nh("~");

  ros::ServiceServer service = nh.advertiseService("compute_dissimilarity", dissimilarity);

  ROS_INFO_STREAM(ros::this_node::getName() << "/compute_dissimilarity: server ready");
  ros::spin();

  return 0;
}
