#ifndef _PM_FOURIER_FOURIER_DIST_H_
#define _PM_FOURIER_FOURIER_DIST_H_

#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#include <lama_common/point.h>
#include <lama_common/polygon.h>
#include <place_matcher_msgs/PolygonDissimilarity.h>

#include <pm_fourier/fourier.h>

using std::vector;
using lama_common::Point2;

class FourierDistance
{
  public:

    FourierDistance(ros::NodeHandle& node);

    bool dissimilarity(place_matcher_msgs::PolygonDissimilarity::Request& req, place_matcher_msgs::PolygonDissimilarity::Response& res);

  private:

    ros::NodeHandle &node;
};

#endif // _PM_FOURIER_FOURIER_DIST_H_
