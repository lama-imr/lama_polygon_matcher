/********************************************
 * Shape dissimilarity measurement based on the Multi-scale
 * Convexity Concavity (MCC) representation.
 *
 * Based on article 
 * @ARTICLE{Adamek2004,
 *    author={Adamek, T. and O''Connor, N.E.},
 *    journal={Circuits and Systems for Video Technology, IEEE Transactions on}, 
 *    title={A multiscale representation method for nonrigid shapes with a single closed contour},
 *    year={2004},
 *    volume={14},
 *    number={5},
 *    pages={742-753},
 *    doi={10.1109/TCSVT.2004.826776},
 *    ISSN={1051-8215},
 *    }
 *************************************/

#include <algorithm>
#include <limits>
#include <cmath>
#include <cassert>

#include <boost/numeric/ublas/matrix.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <lama_common/point.h>
#include <lama_common/polygon.h>
#include <polygon_matcher/PolygonDissimilarity.h>

// Number of points to keep for the mcc computation.
unsigned int g_num_samples;

// Number of scales to compute.
unsigned int g_max_sigma;

bool g_rotation_invariance;

using std::sqrt;
using std::abs;

typedef boost::numeric::ublas::matrix<double> matrix;
typedef std::vector<geometry_msgs::Polygon> polygon_list;

int inpoly(const geometry_msgs::Polygon& p, const geometry_msgs::Point32& point)
{
  float x = point.x;
  float y = point.y;
  unsigned int i, j, c = 0;
  for (i = 0, j = p.points.size() - 1; i < p.points.size(); j = i++)
  {
    if ((((p.points[i].y <= y) && (y < p.points[j].y)) ||
          ((p.points[j].y <= y) && (y < p.points[i].y))) &&
        (x < (p.points[j].x - p.points[i].x) * (y - p.points[i].y) / (p.points[j].y - p.points[i].y) + p.points[i].x))
      c = !c;
  }
  return c;
}

/** Evolve a polygon
 *
 * @param[in] input Polygon to evolve
 * @param[out] output List of evolved polygons with scales { 0, 1, .., max_sigma}.
 * @param[in] max_sigma Scale count.
 */
bool evolve(const geometry_msgs::Polygon& input, polygon_list& output, unsigned int max_sigma)
{
  output.clear();
  output.reserve(max_sigma);
  output.push_back(input);  // With sigma == 0.
  const int size = input.points.size();
  for (unsigned int s = 1; s < max_sigma; s++)
  {
    geometry_msgs::Polygon evolved_polygon;
    evolved_polygon.points.reserve(size);
    // Index distance to consider the filter impulse reponse negligeable.
    // https://en.wikipedia.org/wiki/Scale_space_implementation#The_sampled_Gaussian_kernel.
    const int max_dist_to_filter = std::min((int)(4 * s + 1), size);
    for (int i = 0; i < size; i++)
    {
      double sumX = 0;
      double sumY = 0;
      for (int j = -max_dist_to_filter; j < max_dist_to_filter + 1; j++)
      {
        // Apply a Gaussian kernel.
        const double filter = 1.0 / (s * sqrt(2.0 * M_PI)) * std::exp(-((double)j * (double)j) / (2.0 * s * s));
        sumX += input.points[(i - j + size) % size].x * filter;
        sumY += input.points[(i - j + size) % size].y * filter;
      }
      geometry_msgs::Point32 p;
      p.x = sumX;
      p.y = sumY;
      p.z = 0;
      evolved_polygon.points.push_back(p);
    }
    output.push_back(evolved_polygon);
  }
  return true;
}

/** Compute the convexity of scaled polygons.
 *
 * Compute the convexity of scaled polygons by looking at the signed distance
 * between a point at scale s and the same point at scale (s - 1).
 * This approximates the curvature.
 *
 * Return the estimated shape complexity as the average of the differences
 * between max and min convexity/concavity measures over all scale levels.
 *
 * @param[in] polygons A list of s scaled polygons of n points each.
 * @param[out] m A multi-scale convexity concavity representation, an (n x s) matrix.
 * @return The polygon complexity.
 */
double compute_convexity(const polygon_list& polygons, matrix& m)
{
  double complexity = 0;

  for (unsigned int s = 1; s < g_max_sigma; s++)
  {
    double cMax = 0;
    double cMin = std::numeric_limits<double>::max();      

    for (unsigned int u = 0; u < g_num_samples; u++)
    {
      // k = 1 if point inside polygon; -1 otherwise.
      // k = 1 for convex part of the polygon contour.
      int k = 2 * inpoly(polygons[s], polygons[s - 1].points[u]) - 1;
      // TODO: talk with Karel why not m(u,s) as stated in the article.
      m(u, s - 1) = k * sqrt(
          (polygons[s].points[u].x - polygons[s - 1].points[u].x) *
          (polygons[s].points[u].x - polygons[s - 1].points[u].x) + 
          (polygons[s].points[u].y - polygons[s - 1].points[u].y) *
          (polygons[s].points[u].y - polygons[s - 1].points[u].y));

      if (m(u, s - 1) < cMin)
      {
        cMin = m(u, s - 1);
      }
      if (m(u, s - 1) > cMax)
      {
        cMax = m(u, s - 1);
      }
    }
    complexity += (cMax - cMin);
  }
  complexity /= (g_max_sigma - 1);
  return complexity;
}

/** Compute the distance between two MCC representations
 *
 * Compute the distance between two MCC representations, i.e. for each contour
 * point of a and b (rows) sum the absolute difference on all scale levels
 * (columns).
 * In order to achieve the optional rotational invariance, a circular shift is
 * applied on the rows of the second matrix.
 *
 * @param[in] a An (na x m) matrix.
 * @param[in] b An (nb x m) matrix.
 * @param[out] An (na x nb) matrix.
 */
matrix compare(const matrix& a, const matrix& b)
{
  assert(a.size2() == b.size2());

  matrix ret(a.size1(), b.size1());

  size_t last_row_shift = 1;
  if (g_rotation_invariance)
  {
    last_row_shift = b.size1();
  }

  for (size_t index_row_a = 0; index_row_a < a.size1(); index_row_a++)
  {
    for (size_t index_row_b = 0; index_row_b < last_row_shift; index_row_b++)
    {
      double sum = 0;
      // First and last scale levels are excluded.
      for (size_t s = 1; s < a.size2() - 1; s++)
      {
        sum += abs(a(index_row_a, s) - b(index_row_b, s));
      }
      ret(index_row_a, index_row_b) = (1.0 / a.size2()) * sum;
    }
  }
  return ret;
}	

/** Return the minimal distance
 */
double minDistance(const matrix& compared, int start)
{
  size_t n = compared.size1();
  size_t m = compared.size2();
  matrix D(n, m);
  D(0, 0) = compared(0, start);
  
  // Fill start column.
  for (size_t i = 1; i < n; i++)
  {
    D(i, 0) = compared(i, start) + D(i - 1, 0);
  }
  // Fill first row.
  for (size_t j = 1; j < m; j++)
  {
    D(0, j) = compared(0, (j+start) % m) + D(0, j - 1);
  }

  for (size_t i = 1 ; i < n; i++)
  {
    for (size_t j = 1; j < m; j++)
    {
      D(i, j) = compared(i, (j+start) % m) + std::min(D(i - 1, j), std::min(D(i, (j - 1)), D(i - 1, (j - 1))));
    }
  }
  return D(n - 1, m - 1); 
} 

// TODO: write a class for dissimilarity in order to avoid global variables.

/** PolygonDissimilarity service callback
 */
bool dissimilarity(polygon_matcher::PolygonDissimilarity::Request& req,
    polygon_matcher::PolygonDissimilarity::Response& res)
{

  ROS_DEBUG("Request: size_1=%zu, size_2=%zu", req.polygon1.points.size(), req.polygon2.points.size());
  ros::Time start = ros::Time::now();

  geometry_msgs::Polygon polygon1res;
  geometry_msgs::Polygon polygon2res; 
  double delta;
  polygon1res.points = lama_common::resamplePolygon(req.polygon1.points, g_num_samples, delta);
  polygon2res.points = lama_common::resamplePolygon(req.polygon2.points, g_num_samples, delta);
  // Create multi-scale representation (increasing sigma).
  polygon_list polygon1evo;
  polygon_list polygon2evo;
  evolve(polygon1res, polygon1evo, g_max_sigma);
  evolve(polygon2res, polygon2evo, g_max_sigma);
  res.processing_time = ros::Time::now() - start;
  ROS_DEBUG("Evolved polygons created after %.4f s", res.processing_time.toSec());

  matrix mcc1(g_num_samples, g_max_sigma);
  matrix mcc2(g_num_samples, g_max_sigma);

  // TODO: discuss with Karel the interest of the complexity normalization.
  const double C1 = compute_convexity(polygon1evo, mcc1);
  const double C2 = compute_convexity(polygon2evo, mcc2);
  ROS_DEBUG("Complexity normalization of polygon 1 = %f", C1);
  ROS_DEBUG("Complexity normalization of polygon 2 = %f", C2);

  res.processing_time = ros::Time::now() - start;
  ROS_DEBUG("Multi-scale convexity-concavity computed after %.4f s", res.processing_time.toSec());

  matrix comp = compare(mcc1, mcc2);

  res.processing_time = ros::Time::now() - start;
  ROS_DEBUG("Comparison created after %.4f s", res.processing_time.toSec());

  std::vector<double> result;
  result.reserve(comp.size2());
  for (size_t i = 0; i < comp.size2(); i++)
  {
    result.push_back(minDistance(comp, i));
  }
  res.raw_dissimilarity = (*std::min_element(result.begin(), result.end())) * 2.0 / ((C1 + C2) * (g_num_samples));

  res.processing_time = ros::Time::now() - start;
  ROS_DEBUG("Sending back response: %f  (in %.4f s)", res.raw_dissimilarity, res.processing_time.toSec());

  return true;
}

int main(int argc, char **argv)
{
  int max_thread;
  ros::init(argc, argv, "mcc_polygon_dissimilarity_server");
  ros::NodeHandle nh("~");

  nh.param<int>("max_thread", max_thread, 1);

  // Number of points to keep for the mcc computation.
  int sample_count;
  nh.param<int>("sample_count", sample_count, 100);
  g_num_samples = sample_count;

  // Number of scales to compute.
  int scale_count;
  nh.param<int>("scale_count", scale_count, 10);
  g_max_sigma = scale_count;

  // With rotation_invariance = true, no cyclic optimisation will be done in compare.
  nh.param<bool>("rotation_invariance", g_rotation_invariance, true);

  ros::ServiceServer service = nh.advertiseService("compute_dissimilarity", dissimilarity);

  ROS_INFO("Ready to work (with %i threads)", max_thread);

  ros::MultiThreadedSpinner spinner(max_thread);
  spinner.spin();

  return 0;
}
