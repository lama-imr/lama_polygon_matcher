#ifndef PLACE_MATCHER_CSM_DISSIMILARITY_GETTER_H
#define PLACE_MATCHER_CSM_DISSIMILARITY_GETTER_H

#include <cmath>
#include <limits>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Polygon.h>

#include <place_matcher_msgs/PolygonDissimilarity.h>
extern "C"
{
#include <csm/csm_all.h>
#include <csm/icp/icp.h>
#include <egsl/egsl_macros.h>
// csm creates max macro that causes issues with std::numeric_limits<>::max.
#undef min
#undef max
}

namespace place_matcher_csm
{

class DissimilarityGetter
{
  public :

    DissimilarityGetter(ros::NodeHandle& nh_private_);

    bool getDissimilarity(place_matcher_msgs::PolygonDissimilarityRequest& req, place_matcher_msgs::PolygonDissimilarityResponse& res);

  private :

    // Internals.
    void initParams();

    ros::NodeHandle nh_private_;
    sm_params csm_input_;
};

} /* mamespace place_matcher_csm */

#endif /* PLACE_MATCHER_CSM_DISSIMILARITY_GETTER_H */

