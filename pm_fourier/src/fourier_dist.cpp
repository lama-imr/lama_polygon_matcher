#include <pm_fourier/fourier_dist.h>

FourierDistance::FourierDistance(ros::NodeHandle &node) :
  node(node)
{
}

bool FourierDistance::dissimilarity(polygon_matcher::PolygonDissimilarity::Request& req,
    polygon_matcher::PolygonDissimilarity::Response& res)
{
  ros::Time start = ros::Time::now();

  vector<Point2> pts1;
  for(size_t i = 0; i < req.polygon1.points.size(); i++)
  {
    pts1.push_back(Point2(req.polygon1.points[i].x, req.polygon1.points[i].y));
  }
  vector<Point2> pts2;
  for(size_t i = 0; i < req.polygon2.points.size(); i++)
  {
    pts2.push_back(Point2(req.polygon2.points[i].x, req.polygon2.points[i].y));
  }

  const int numOfSamples = 361;
  double delta1;
  double delta2;
  vector<Point2> rpol1(resamplePolygon(pts1, numOfSamples, delta1));
  vector<Point2> rpol2(resamplePolygon(pts2, numOfSamples, delta2));

  const int fftSize = 30; // number of harmonics
  res.raw_dissimilarity = getDissimilarityFourier(rpol1, rpol2, fftSize);
  res.processing_time = ros::Time::now() - start;
  ROS_DEBUG("sending back response in %f s", res.processing_time.toSec());

  return true;
}
