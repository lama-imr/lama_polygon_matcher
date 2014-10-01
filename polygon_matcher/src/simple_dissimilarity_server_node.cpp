#include <ros/ros.h>

#include <polygon_matcher/PolygonDissimilarity.h>

bool dissimilarity(polygon_matcher::PolygonDissimilarity::Request  &req,
    polygon_matcher::PolygonDissimilarity::Response &res)
{
  float diff = 0;
  ros::Time start = ros::Time::now();
  ROS_DEBUG("request: s1=%zu, s2=%zu", req.polygon1.points.size(), req.polygon2.points.size());
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
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("simple_polygon_dissimilarity", dissimilarity);

  ROS_INFO("%s: ready to work", ros::this_node::getName().c_str());
  ros::spin();

  return 0;
}
