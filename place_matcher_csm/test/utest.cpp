#include <algorithm>
#include <cmath> // std::cos, std::sin
#include <cstdlib> // std::rand
#include <ctime> // std::time
#include <fstream> // std::ofstream
#include <vector>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>
#include <gtest/gtest.h>
#include <place_matcher_msgs/PolygonDissimilarity.h>
#include <ros/ros.h>
#include <tf/tf.h> // for getYaw().

#include <place_matcher_csm/dissimilarity_getter.h>

#define FILE_OUTPUT 0

ros::NodeHandlePtr g_nh_ptr;

#if FILE_OUTPUT
void saveToFile(const std::string& filename, const geometry_msgs::Polygon& poly)
{
  std::ofstream fout(filename.c_str());
  if (!fout.is_open())
  {
    std::cerr << "\"" << filename << "\" cannot be opened for writing";
    return;
  }
  std::vector<geometry_msgs::Point32>::const_iterator it = poly.points.begin();
  for (; it < poly.points.end(); ++it)
  {
    fout << it->x << "," << it->y << "\n";
  }
}
#endif

void transformPoint(const geometry_msgs::Pose& transform, geometry_msgs::Point32& point)
{
  const geometry_msgs::Point32 old_point = point;
  const double yaw = tf::getYaw(transform.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  point.x = transform.position.x + old_point.x * cos_yaw - old_point.y * sin_yaw;
  point.y = transform.position.y + old_point.x * sin_yaw + old_point.y * cos_yaw;
}

void transformPolygon(const geometry_msgs::Pose& transform, geometry_msgs::Polygon& poly)
{
  for (size_t i = 0; i < poly.points.size(); ++i)
  {
    transformPoint(transform, poly.points[i]);
  }
}

geometry_msgs::Polygon getRandPolygon()
{
  const double angle_resolution = M_PI / 180;
  const double range_min = 2.0;
  const double range_max = 5.0;

  geometry_msgs::Polygon poly;
  const size_t point_count = (size_t) (2 * M_PI / angle_resolution);
  poly.points.reserve(point_count);
  for (double theta = - 2 * M_PI + 1e-10; theta < 2 * M_PI; theta += angle_resolution)
  {
    geometry_msgs::Point32 point;
    double r = range_min + (range_max - range_min) * ((double) std::rand()) / RAND_MAX; 
    point.x = r * std::cos(theta);
    point.y = r * std::sin(theta);
    poly.points.push_back(point);
  }
  return poly;
}

std::vector<geometry_msgs::Point32> interpolate(const geometry_msgs::Point32& a, const geometry_msgs::Point32 b, double max_distance)
{
  std::vector<geometry_msgs::Point32> points;

  const double dx = b.x - a.x;
  const double dy = b.y - a.y;

  geometry_msgs::Point32 point;
  const double norm = std::sqrt(dx * dx + dy * dy);
  if (norm == 0)
  {
    points.push_back(a);
    return points;
  }

  // Unit vector from a to b.
  const double ux = dx / norm;
  const double uy = dy / norm;
  for (double s = 0; s <= norm; s += max_distance) 
  {
    point.x = a.x + s * ux;
    point.y = a.y + s * uy;
    points.push_back(point);
  }
  return points;
}

geometry_msgs::Point32 point(double x, double y)
{
  geometry_msgs::Point32 point;
  point.x = x;
  point.y = y;
  return point;
}

/* 
 * Profile:
 *     P4 ++++++++++++++++++++++ P3
 *       +                      +
 *      P0  +                    +
 *     +                          + 
 * P1 ++++++++++++++++++++++++++++ P2
 *
 */
geometry_msgs::Polygon getTestPolygon()
{
  geometry_msgs::Polygon poly;

  geometry_msgs::Point32 p0 = point(-0.6, 0);
  geometry_msgs::Point32 p1 = point(-1, -1);
  geometry_msgs::Point32 p2 = point(1, p1.y);
  geometry_msgs::Point32 p3 = point(0.8, 0.8);
  geometry_msgs::Point32 p4 = point(-0.9, p3.y);

  std::vector<geometry_msgs::Point32> points;
  points = interpolate(p0, p1, 0.1);
  std::copy(points.begin(), points.end(), std::back_inserter(poly.points));
  points = interpolate(p1, p2, 0.1);
  std::copy(points.begin(), points.end(), std::back_inserter(poly.points));
  points = interpolate(p2, p3, 0.1);
  std::copy(points.begin(), points.end(), std::back_inserter(poly.points));
  points = interpolate(p3, p4, 0.1);
  std::copy(points.begin(), points.end(), std::back_inserter(poly.points));
  points = interpolate(p4, p0, 0.1);
  std::copy(points.begin(), points.end(), std::back_inserter(poly.points));

  return poly;
}

TEST(TestSuite, testGetDissimilarity)
{
  const geometry_msgs::Polygon poly_ref = getTestPolygon();
  double dtheta = 0.4;
  geometry_msgs::Pose transform;
  transform.position.x = 0.4;
  transform.position.y = 0.2;
  transform.orientation = tf::createQuaternionMsgFromYaw(dtheta);
  geometry_msgs::Polygon poly_sens = poly_ref;
  transformPolygon(transform, poly_sens);

#if FILE_OUTPUT
  saveToFile("/tmp/poly_ref.csv", poly_ref);
  saveToFile("/tmp/poly_sens.csv", poly_sens);
#endif

  place_matcher_msgs::PolygonDissimilarityRequest req;
  place_matcher_msgs::PolygonDissimilarityResponse res;
  req.polygon1 = poly_ref;
  req.polygon2 = poly_sens;
  
  EXPECT_EQ(poly_sens.points.size(), req.polygon2.points.size());
  ros::ServiceClient client = g_nh_ptr->serviceClient<place_matcher_msgs::PolygonDissimilarity>("/place_matcher_csm/compute_dissimilarity");
  while (ros::ok() && !client.waitForExistence(ros::Duration(5)))
  {
    ROS_INFO_STREAM("Waiting for service " << client.getService());
  }
  ASSERT_TRUE(client.call(req, res));
  EXPECT_NEAR(res.pose.position.x, transform.position.x, 1e-3);
  EXPECT_NEAR(res.pose.position.y, transform.position.y, 1e-3);
  EXPECT_NEAR(tf::getYaw(res.pose.orientation), dtheta, 1e-4);

  const double dissimilarity_0 = res.raw_dissimilarity;

  dtheta = 1.4;
  transform.position.x = -0.4;
  transform.position.y = -0.2;
  transform.orientation = tf::createQuaternionMsgFromYaw(dtheta);
  poly_sens = poly_ref;
  transformPolygon(transform, poly_sens);

#if FILE_OUTPUT
  saveToFile("/tmp/poly_ref_2.csv", poly_ref);
  saveToFile("/tmp/poly_sens_2.csv", poly_sens);
#endif

  req.polygon1 = poly_ref;
  req.polygon2 = poly_sens;
  
  ASSERT_TRUE(client.call(req, res));
  EXPECT_NEAR(res.pose.position.x, transform.position.x, 1e-3);
  EXPECT_NEAR(res.pose.position.y, transform.position.y, 1e-3);
  EXPECT_NEAR(tf::getYaw(res.pose.orientation), dtheta, 1e-4);

  // Test that the dissimilarity is rotation invariant.
  EXPECT_NEAR(res.raw_dissimilarity, dissimilarity_0, 1e-2);
}

int main(int argc, char *argv[])
{
  std::srand(std::time(0));

  // ros::init is needed because DissimilarityGetter uses ros::NodeHandle.
  ros::init(argc, argv, "test_cpp_place_matcher_csm");
  ros::NodeHandle nh;
  g_nh_ptr.reset(&nh);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
