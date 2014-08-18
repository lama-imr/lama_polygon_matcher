#ifndef _PM_FOURIER_FOURIER_DIST_H_
#define _PM_FOURIER_FOURIER_DIST_H_

#include <ros/ros.h>
#include <polygon_matcher/PolygonSimilarity.h>

class FourierDistance
{
  public:
    FourierDistance(ros::NodeHandle& node);
    bool similarity(polygon_matcher::PolygonSimilarity::Request& req, polygon_matcher::PolygonSimilarity::Response& res);

  private:
    ros::NodeHandle &node;
};

#endif // _PM_FOURIER_FOURIER_DIST_H_
