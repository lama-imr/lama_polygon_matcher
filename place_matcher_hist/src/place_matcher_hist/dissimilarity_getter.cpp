#include <place_matcher_hist/dissimilarity_getter.h>

#include <fstream> // DEBUG
#include <algorithm> // DEBUG

namespace place_matcher_hist
{

/* DEBUG */
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
/* DEBUG */

static void getXMinMax(const geometry_msgs::Polygon& poly1, const geometry_msgs::Polygon& poly2, double& x_min, double& x_max)
{
  x_min = std::numeric_limits<float>::max();
  x_max = std::numeric_limits<float>::min();

  std::vector<geometry_msgs::Point32>::const_iterator pt;
  for (pt = poly1.points.begin(); pt != poly1.points.end(); ++pt)
  {
    if (x_min > pt->x)
    {
      x_min = pt->x;
    }
    else if (x_max < pt->x)
    {
      x_max = pt->x;
    }
  }
  for (pt = poly2.points.begin(); pt != poly2.points.end(); ++pt)
  {
    if (x_min > pt->x)
    {
      x_min = pt->x;
    }
    else if (x_max < pt->x)
    {
      x_max = pt->x;
    }
  }
}

static void getYMinMax(const geometry_msgs::Polygon& poly1, const geometry_msgs::Polygon& poly2, double& y_min, double& y_max)
{
  y_min = std::numeric_limits<float>::max();
  y_max = std::numeric_limits<float>::min();

  std::vector<geometry_msgs::Point32>::const_iterator pt;
  for (pt = poly1.points.begin(); pt != poly1.points.end(); ++pt)
  {
    if (y_min > pt->y)
    {
      y_min = pt->y;
    }
    else if (y_max < pt->y)
    {
      y_max = pt->y;
    }
  }
  for (pt = poly2.points.begin(); pt != poly2.points.end(); ++pt)
  {
    if (y_min > pt->y)
    {
      y_min = pt->y;
    }
    else if (y_max < pt->y)
    {
      y_max = pt->y;
    }
  }
}

/** Return the polygon rotated with a given angle and around (0, 0).
 */
inline geometry_msgs::Polygon rotatedPolygon(const geometry_msgs::Polygon& poly_in, double angle)
{
  geometry_msgs::Polygon poly_out;
  poly_out.points.reserve(poly_in.points.size());
  const double cos_ = std::cos(angle);
  const double sin_ = std::sin(angle);
  std::vector<geometry_msgs::Point32>::const_iterator pt;
  for (pt = poly_in.points.begin(); pt != poly_in.points.end(); ++pt)
  {
    geometry_msgs::Point32 pt_out;
    pt_out.x = pt->x * cos_ - pt->y * sin_;
    pt_out.y = pt->x * sin_ + pt->y * cos_;
    poly_out.points.push_back(pt_out);
  }
  return poly_out;
}

DissimilarityGetter::DissimilarityGetter(const ros::NodeHandle& nh_private) :
  trans_resolution_(0.1),
  max_trans_bin_count_(200),
  rot_resolution_(0.017453292519943295),
  max_dist_(0.5),
  nh_private_(nh_private)
{
  initParams();
}

/* Init parameters of the csm method
 */
void DissimilarityGetter::initParams()
{
  nh_private_.getParam("translational_resolution", trans_resolution_);
  if (trans_resolution_ == 0)
  {
    ROS_ERROR_STREAM(nh_private_.getNamespace() << "/translational_resolution cannot be 0, setting to default");
    trans_resolution_ = 0.1;
  }
  nh_private_.getParam("rotational_resolution", rot_resolution_);
  nh_private_.getParam("max_distance_normal_considered", max_dist_);
  nh_private_.getParam("max_trans_bin_count", max_trans_bin_count_);
  if (max_trans_bin_count_ < 1)
  {
    ROS_ERROR_STREAM(nh_private_.getNamespace() << "/max_trans_bin_count cannot be 0, setting to default");
    max_trans_bin_count_ = 200;
  }
}

bool DissimilarityGetter::getDissimilarity(place_matcher_msgs::PolygonDissimilarityRequest& req, place_matcher_msgs::PolygonDissimilarityResponse& res)
{
  ros::WallTime start = ros::WallTime::now();

  // Compute the angle histogram for both polygons and the rotation between
  // both histograms.
  Hist rot_hist_1 = getRotHist(req.polygon1);
  Hist rot_hist_2 = getRotHist(req.polygon2);
  const size_t angle_index = crossCorrelationHist(rot_hist_1, rot_hist_2);
  const double dyaw = angle_index * rot_resolution_;

  /* DEBUG */
  ROS_INFO("dyaw = %f", dyaw);
  std::ofstream ofs1("/tmp/rot_hist_1.dat");
  std::copy(rot_hist_1.begin(), rot_hist_1.end(), std::ostream_iterator<double>(ofs1, " "));
  ofs1.close();
  std::ofstream ofs2("/tmp/rot_hist_2.dat");
  std::copy(rot_hist_2.begin(), rot_hist_2.end(), std::ostream_iterator<double>(ofs2, " "));
  ofs2.close();
  /* DEBUG */
  

  // Get the main normal direction for rot_hist_1.
  size_t max_idx_hist1 = 0; 
  unsigned int max_bin = 0;
  for (size_t i = 0; i < rot_hist_1.size(); ++i)
  {
    if (rot_hist_1[i] > max_bin)
    {
      max_bin = rot_hist_1[i];
      max_idx_hist1 = i;
    }
  }
  const double main_axis_poly1 = max_idx_hist1 * rot_resolution_;

  // Rotate both polygons so that they are aligned and that req.polygon1
  // is oriented according to its second main direction (main direction of the normals).
  const geometry_msgs::Polygon poly1_rotated = rotatedPolygon(req.polygon1, main_axis_poly1);
  const geometry_msgs::Polygon poly2_rotated = rotatedPolygon(req.polygon2, main_axis_poly1 - dyaw);

  /* DEBUG */
  saveToFile("/tmp/poly_ref_rotated.csv", poly1_rotated);
  saveToFile("/tmp/poly_sens_rotated.csv", poly2_rotated);
  Hist rot_hist_3 = getRotHist(poly2_rotated);
  std::ofstream ofs3("/tmp/rot_hist_2_rot.dat");
  std::copy(rot_hist_3.begin(), rot_hist_3.end(), std::ostream_iterator<double>(ofs3, " "));
  ofs3.close();
  /* DEBUG */

  // Get the translation between both polygons.
  double x_min;
  double x_max;
  double y_min;
  double y_max;
  getXMinMax(poly1_rotated, poly2_rotated, x_min, x_max);
  getYMinMax(poly1_rotated, poly2_rotated, y_min, y_max);
  const size_t trans_x_bin_count = std::min(getTransBinCount(x_min, x_max), max_trans_bin_count_);
  const size_t trans_y_bin_count = std::min(getTransBinCount(y_min, y_max), max_trans_bin_count_);

  // Set the bin width slightly larger so that the index for the value that is
  // x_mmax or y_max is correct.
  const double bin_width_x = 1.00001 * (x_max - x_min) / trans_x_bin_count;
  const double bin_width_y = 1.00001 * (y_max - y_min) / trans_y_bin_count;

  if (bin_width_x < 1e-6 || bin_width_y < 1e-6)
  {
    ROS_ERROR("Null bin width");
    return false;
  }

  // Compute the histograms for translations and get the max. correlation.
  Hist hist_x_1(trans_x_bin_count, 0);
  Hist hist_y_1(trans_y_bin_count, 0);
  ROS_INFO("hist_x1.size() = %zu, hist_y_1.size() = %zu", hist_x_1.size(), hist_y_1.size()); // DEBUG
  for (size_t i = 0; i < poly1_rotated.points.size(); ++i)
  {
    hist_x_1[(size_t) std::floor((poly1_rotated.points[i].x - x_min) / bin_width_x)] += 1;
    hist_y_1[(size_t) std::floor((poly1_rotated.points[i].y - y_min) / bin_width_y)] += 1;
  }

  Hist hist_x_2(trans_x_bin_count, 0);
  Hist hist_y_2(trans_y_bin_count, 0);
  for (size_t i = 0; i < poly2_rotated.points.size(); ++i)
  {
    hist_x_2[(size_t) std::floor((poly2_rotated.points[i].x - x_min) / bin_width_x)] += 1;
    hist_y_2[(size_t) std::floor((poly2_rotated.points[i].y - y_min) / bin_width_y)] += 1;
  }
  int x_index = crossCorrelationHist(hist_x_1, hist_x_2);
  int y_index = crossCorrelationHist(hist_y_1, hist_y_2);
  if (x_index > trans_x_bin_count / 2)
  {
    x_index -= trans_x_bin_count;
  }
  if (y_index > trans_y_bin_count / 2)
  {
    y_index -= trans_y_bin_count;
  }
  const double dx_rotated = x_index * bin_width_x;
  const double dy_rotated = y_index * bin_width_y;

  /* DEBUG */
  ROS_INFO("main_axis_poly1 = %f", main_axis_poly1);
  ROS_INFO("bbox = (%f, %f), (%f, %f)", x_min, y_min, x_max, y_max);
  ROS_INFO("bin_width_x = %f", bin_width_x);
  ROS_INFO("bin_width_y = %f", bin_width_y);
  ROS_INFO("x_index = %d", x_index);
  ROS_INFO("y_index = %d", y_index);
  ROS_INFO("dx_rotated = %f", dx_rotated);
  ROS_INFO("dy_rotated = %f", dy_rotated);
  /* DEBUG */

  // Get the translation in the original frame (poly1)
  const double cos_ = std::cos(-main_axis_poly1 + dyaw);
  const double sin_ = std::sin(-main_axis_poly1 + dyaw);
  const double dx = cos_ * dx_rotated - sin_* dy_rotated;
  const double dy = sin_ * dx_rotated + cos_ * dy_rotated;
  
  /* DEBUG */
  ROS_INFO("dx = %f", dx);
  ROS_INFO("dy = %f", dy);
#if 0
  std::ofstream ofs4("/tmp/hist_x_1.dat");
  std::copy(hist_x_1.begin(), hist_x_1.end(), std::ostream_iterator<double>(ofs4, " "));
  ofs4.close();
  std::ofstream ofs5("/tmp/hist_y_1.dat");
  std::copy(hist_y_1.begin(), hist_y_1.end(), std::ostream_iterator<double>(ofs5, " "));
  ofs5.close();
  std::ofstream ofs6("/tmp/hist_x_2.dat");
  std::copy(hist_x_2.begin(), hist_x_2.end(), std::ostream_iterator<double>(ofs6, " "));
  ofs6.close();
  std::ofstream ofs7("/tmp/hist_y_2.dat");
  std::copy(hist_y_2.begin(), hist_y_2.end(), std::ostream_iterator<double>(ofs7, " "));
  ofs7.close();
#endif
  /* DEBUG */

  // Set the response.
  res.pose.position.x = dx;
  res.pose.position.y = dy;
  res.pose.orientation = tf::createQuaternionMsgFromYaw(dyaw);
  res.processing_time = ros::Duration((ros::WallTime::now() - start).toSec());
  return true;
}

DissimilarityGetter::Hist DissimilarityGetter::getRotHist(const geometry_msgs::Polygon poly)
{
  const size_t n = poly.points.size();
  const size_t bin_count = (size_t) std::ceil(2 * M_PI / rot_resolution_);
  Hist hist(bin_count, 0);

  for (size_t i = 0; i < n; ++i)
  {
    for (size_t shift = 1; shift < shift_ + 1; ++shift)
    {
      const double dx = poly.points[(i + shift) % n].x - poly.points[i].x;
      const double dy = poly.points[(i + shift) % n].y - poly.points[i].y;
      const double distance = std::sqrt(dx * dx + dy * dy);
      if (distance < max_dist_)
      {
        // We compute the normal of the angle by rotating the vector (dx, dy)
        // by 90 deg ==> normal = (-dy, dx).
        double angle = std::atan2(dx, -dy);
        if (angle < 0)
        {
          angle += 2 * M_PI;
        }
        hist[(size_t) std::floor(angle / rot_resolution_)] += 1;
      }
    }
  }
  return hist;
}

/**
 * @return the index in h1
 */
size_t DissimilarityGetter::crossCorrelationHist(DissimilarityGetter::Hist h1, DissimilarityGetter::Hist h2)
{
  const size_t n1 = h1.size();
  const size_t n2 = h2.size();
  double max_sum = 0.0;
  size_t ind_max = 0;
  for (size_t shift = 0; shift < n1; ++shift)
  {
    double sum = 0.0; 
    for (size_t i = 0; i < n1; ++i)
    {
      sum += h1[i] * h2[(i + shift) % n2];
    }
    if (sum > max_sum)
    {
      max_sum = sum;
      ind_max = shift;
    }
  }
  return ind_max;
}


} /* namespace place_matcher_hist */
