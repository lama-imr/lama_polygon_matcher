#include <place_matcher_csm/dissimilarity_getter.h>

namespace place_matcher_csm
{

/** Compute the laser-data from a polygon
 *
 * Copied and adapted from laser_scan_matcher, PointCloudToLDP.
 */
void polygonToLDP(const geometry_msgs::Polygon& poly, LDP& ldp)
{
  const size_t n = poly.points.size();

  ldp = ld_alloc_new(n);
  ldp->nrays = n;

  for (size_t i = 0; i < n; ++i)
  {
    // calculate position in laser frame
    if (is_nan(poly.points[i].x) || is_nan(poly.points[i].y))
    {
      ROS_WARN("place_matcher_csm: polygon input contains NaN values. \
                Please use a filtered polygon input.");
    }
    else
    {
      const double r = std::sqrt(poly.points[i].x * poly.points[i].x +
          poly.points[i].y * poly.points[i].y);

      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }

    ldp->theta[i] = std::atan2(poly.points[i].y, poly.points[i].x);
    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

DissimilarityGetter::DissimilarityGetter(ros::NodeHandle& nh_private) :
  nh_private_(nh_private)
{
  initParams();
}

/* Init parameters of the csm method
 */
void DissimilarityGetter::initParams()
{
  csm_input_.min_reading = 0;
  csm_input_.max_reading = std::numeric_limits<double>::max();

  // **** CSM parameters - comments copied from algos.h (by Andrea Censi)
  // Defaults taken from the laser_scan_matcher package.

  // Maximum angular displacement between scans
  csm_input_.max_angular_correction_deg = 45.0;
  nh_private_.getParam("max_angular_correction_deg", csm_input_.max_angular_correction_deg);

  // Maximum translation between scans (m)
  csm_input_.max_linear_correction = 0.50;
  nh_private_.getParam ("max_linear_correction", csm_input_.max_linear_correction);

  // Maximum ICP cycle iterations
  csm_input_.max_iterations = 10;
  nh_private_.getParam ("max_iterations", csm_input_.max_iterations);

  // A threshold for stopping (m)
  csm_input_.epsilon_xy = 0.000001;
  nh_private_.getParam ("epsilon_xy", csm_input_.epsilon_xy);

  // A threshold for stopping (rad)
  csm_input_.epsilon_theta = 0.000001;
  nh_private_.getParam ("epsilon_theta", csm_input_.epsilon_theta);

  // Maximum distance for a correspondence to be valid
  csm_input_.max_correspondence_dist = 0.3;
  nh_private_.getParam ("max_correspondence_dist", csm_input_.max_correspondence_dist);

  // Noise in the scan (m)
  csm_input_.sigma = 0.010;
  nh_private_.getParam ("sigma", csm_input_.sigma);

  // Use smart tricks for finding correspondences.
  csm_input_.use_corr_tricks = 1;
  nh_private_.getParam ("use_corr_tricks", csm_input_.use_corr_tricks);

  // Restart: Restart if error is over threshold
  csm_input_.restart = 0;
  nh_private_.getParam ("restart", csm_input_.restart);

  // Restart: Threshold for restarting
  csm_input_.restart_threshold_mean_error = 0.01;
  nh_private_.getParam ("restart_threshold_mean_error", csm_input_.restart_threshold_mean_error);

  // Restart: displacement for restarting. (m)
  csm_input_.restart_dt = 1.0;
  nh_private_.getParam ("restart_dt", csm_input_.restart_dt);

  // Restart: displacement for restarting. (rad)
  csm_input_.restart_dtheta = 0.1;
  nh_private_.getParam ("restart_dtheta", csm_input_.restart_dtheta);

  // Max distance for staying in the same clustering
  csm_input_.clustering_threshold = 0.25;
  nh_private_.getParam ("clustering_threshold", csm_input_.clustering_threshold);

  // Number of neighbour rays used to estimate the orientation
  csm_input_.orientation_neighbourhood = 20;
  nh_private_.getParam ("orientation_neighbourhood", csm_input_.orientation_neighbourhood);

  // If 0, it's vanilla ICP
  csm_input_.use_point_to_line_distance = 1;
  nh_private_.getParam ("use_point_to_line_distance", csm_input_.use_point_to_line_distance);

  // Discard correspondences based on the angles
  csm_input_.do_alpha_test = 0;
  nh_private_.getParam ("do_alpha_test", csm_input_.do_alpha_test);

  // Discard correspondences based on the angles - threshold angle, in degrees
  csm_input_.do_alpha_test_thresholdDeg = 20.0;
  nh_private_.getParam ("do_alpha_test_thresholdDeg", csm_input_.do_alpha_test_thresholdDeg);

  // Percentage of correspondences to consider: if 0.9,
  // always discard the top 10% of correspondences with more error
  csm_input_.outliers_maxPerc = 0.90;
  nh_private_.getParam ("outliers_maxPerc", csm_input_.outliers_maxPerc);

  // Parameters describing a simple adaptive algorithm for discarding.
  //  1) Order the errors.
  //  2) Choose the percentile according to outliers_adaptive_order.
  //     (if it is 0.7, get the 70% percentile)
  //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
  //     with the value of the error at the chosen percentile.
  //  4) Discard correspondences over the threshold.
  //  This is useful to be conservative; yet remove the biggest errors.
  csm_input_.outliers_adaptive_order = 0.7;
  nh_private_.getParam ("outliers_adaptive_order", csm_input_.outliers_adaptive_order);

  csm_input_.outliers_adaptive_mult = 2.0;
  nh_private_.getParam ("outliers_adaptive_mult", csm_input_.outliers_adaptive_mult);

  // If you already have a guess of the solution, you can compute the polar angle
  // of the points of one scan in the new position. If the polar angle is not a monotone
  // function of the readings index, it means that the surface is not visible in the
  // next position. If it is not visible, then we don't use it for matching.
  csm_input_.do_visibility_test = 0;
  nh_private_.getParam ("do_visibility_test", csm_input_.do_visibility_test);

  // no two points in laser_sens can have the same corr.
  csm_input_.outliers_remove_doubles = 1;
  nh_private_.getParam ("outliers_remove_doubles", csm_input_.outliers_remove_doubles);

  // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  csm_input_.do_compute_covariance = 0;
  nh_private_.getParam ("do_compute_covariance", csm_input_.do_compute_covariance);

  // Checks that find_correspondences_tricks gives the right answer
  csm_input_.debug_verify_tricks = 0;
  nh_private_.getParam ("debug_verify_tricks", csm_input_.debug_verify_tricks);

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
  // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
  csm_input_.use_ml_weights = 0;
  nh_private_.getParam ("use_ml_weights", csm_input_.use_ml_weights);

  // If 1, the field 'readings_sigma' in the second scan is used to weight the
  // correspondence by 1/sigma^2
  csm_input_.use_sigma_weights = 0;
  nh_private_.getParam ("use_sigma_weights", csm_input_.use_sigma_weights);
}

bool DissimilarityGetter::getDissimilarity(place_matcher_msgs::PolygonDissimilarityRequest& req, place_matcher_msgs::PolygonDissimilarityResponse& res)
{
  ros::WallTime start = ros::WallTime::now();

  LDP ldp1;
  polygonToLDP(req.polygon1, ldp1);
  csm_input_.laser_ref = ldp1;

  LDP ldp2;
  polygonToLDP(req.polygon2, ldp2);
  csm_input_.laser_sens = ldp2;

  // Polygon match - using point to line icp from CSM.
  sm_result csm_output;
  sm_icp(&csm_input_, &csm_output);
  if (!csm_output.valid)
  {
    ROS_INFO("Canonical scan match failed");
    res.raw_dissimilarity = std::numeric_limits<double>::max();
    res.pose.orientation.w = 1.0;
    res.processing_time = ros::Duration((ros::WallTime::now() - start).toSec());
    return true;
  }

  res.raw_dissimilarity = csm_output.error;
  res.pose.position.x = csm_output.x[0];
  res.pose.position.y = csm_output.x[1];
  res.pose.orientation = tf::createQuaternionMsgFromYaw(csm_output.x[2]);
  res.processing_time = ros::Duration((ros::WallTime::now() - start).toSec());

  return true;
}

} /* namespace place_matcher_csm */

