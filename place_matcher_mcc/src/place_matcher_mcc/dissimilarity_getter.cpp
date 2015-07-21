#include <place_matcher_mcc/dissimilarity_getter.h>

namespace place_matcher_mcc
{

using std::sqrt;
using std::abs;

int inpoly(const geometry_msgs::Polygon& poly, const geometry_msgs::Point32& point)
{
  float x = point.x;
  float y = point.y;
  unsigned int i;
  unsigned int j;
  unsigned int c = 0;

  for (i = 0, j = poly.points.size() - 1; i < poly.points.size(); j = i++)
  {
    if ((((poly.points[i].y <= y) && (y < poly.points[j].y)) ||
          ((poly.points[j].y <= y) && (y < poly.points[i].y))) &&
        (x < (poly.points[j].x - poly.points[i].x) * (y - poly.points[i].y) / (poly.points[j].y - poly.points[i].y) + poly.points[i].x))
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

DissimilarityGetter::DissimilarityGetter() :
  scale_count(10),
  sample_count(100),
  rotation_invariance(true)
{
}

double DissimilarityGetter::getDissimilarity(const geometry_msgs::Polygon& polygon1, const geometry_msgs::Polygon& polygon2)
{
  geometry_msgs::Polygon polygon1res;
  geometry_msgs::Polygon polygon2res; 
  double delta;
  polygon1res.points = lama_common::resamplePolygon(polygon1.points, sample_count, delta);
  polygon2res.points = lama_common::resamplePolygon(polygon2.points, sample_count, delta);
  // Create multi-scale representation (increasing sigma).
  polygon_list polygon1evo;
  polygon_list polygon2evo;
  evolve(polygon1res, polygon1evo, scale_count);
  evolve(polygon2res, polygon2evo, scale_count);

  matrix mcc1(sample_count, scale_count);
  matrix mcc2(sample_count, scale_count);

  // TODO: discuss with Karel the interest of the complexity normalization.
  const double C1 = compute_convexity(polygon1evo, mcc1);
  const double C2 = compute_convexity(polygon2evo, mcc2);
  ROS_DEBUG("Complexity normalization of polygon 1 = %f", C1);
  ROS_DEBUG("Complexity normalization of polygon 2 = %f", C2);

  matrix comp = compare(mcc1, mcc2);

  std::vector<double> result;
  result.reserve(comp.size2());
  for (size_t i = 0; i < comp.size2(); i++)
  {
    result.push_back(minDistance(comp, i));
  }
  return (*std::min_element(result.begin(), result.end())) * 2.0 / ((C1 + C2) * (sample_count));
}

bool DissimilarityGetter::getDissimilarity(place_matcher_msgs::PolygonDissimilarityRequest& req, place_matcher_msgs::PolygonDissimilarityResponse& res)
{

  ROS_DEBUG("Request: size_1 = %zu, size_2 = %zu", req.polygon1.points.size(), req.polygon2.points.size());
  ros::WallTime start = ros::WallTime::now();

  geometry_msgs::Polygon polygon1res;
  geometry_msgs::Polygon polygon2res; 
  double delta;
  polygon1res.points = lama_common::resamplePolygon(req.polygon1.points, sample_count, delta);
  polygon2res.points = lama_common::resamplePolygon(req.polygon2.points, sample_count, delta);
  // Create multi-scale representation (increasing sigma).
  polygon_list polygon1evo;
  polygon_list polygon2evo;
  evolve(polygon1res, polygon1evo, scale_count);
  evolve(polygon2res, polygon2evo, scale_count);
  res.processing_time = ros::Duration((ros::WallTime::now() - start).toSec());
  ROS_DEBUG("Evolved polygons created after %.4f s", res.processing_time.toSec());

  matrix mcc1(sample_count, scale_count);
  matrix mcc2(sample_count, scale_count);

  // TODO: discuss with Karel the interest of the complexity normalization.
  const double C1 = compute_convexity(polygon1evo, mcc1);
  const double C2 = compute_convexity(polygon2evo, mcc2);
  ROS_DEBUG("Complexity normalization of polygon 1 = %f", C1);
  ROS_DEBUG("Complexity normalization of polygon 2 = %f", C2);

  res.processing_time = ros::Duration((ros::WallTime::now() - start).toSec());
  ROS_DEBUG("Multi-scale convexity-concavity computed after %.4f s", res.processing_time.toSec());

  matrix comp = compare(mcc1, mcc2);

  res.processing_time = ros::Duration((ros::WallTime::now() - start).toSec());
  ROS_DEBUG("Comparison created after %.4f s", res.processing_time.toSec());

  std::vector<double> result;
  result.reserve(comp.size2());
  for (size_t i = 0; i < comp.size2(); i++)
  {
    result.push_back(minDistance(comp, i));
  }
  res.raw_dissimilarity = (*std::min_element(result.begin(), result.end())) * 2.0 / ((C1 + C2) * (sample_count));

  res.processing_time = ros::Duration((ros::WallTime::now() - start).toSec());
  ROS_DEBUG("Sending back response: %f  (in %.4f s)", res.raw_dissimilarity, res.processing_time.toSec());

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
double DissimilarityGetter::compute_convexity(const polygon_list& polygons, matrix& m)
{
  double complexity = 0;

  for (unsigned int s = 1; s < scale_count; s++)
  {
    double cMax = 0;
    double cMin = std::numeric_limits<double>::max();      

    for (unsigned int u = 0; u < sample_count; u++)
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
  complexity /= (scale_count - 1);
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
matrix DissimilarityGetter::compare(const matrix& a, const matrix& b)
{
  assert(a.size2() == b.size2());

  matrix ret(a.size1(), b.size1());

  size_t last_row_shift = 1;
  if (rotation_invariance)
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

} /* namespace place_matcher_mcc */
