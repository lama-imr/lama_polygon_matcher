#pragma once
#ifndef PLACE_MATCHER_HIST_DISSIMILARITY_GETTER_H
#define PLACE_MATCHER_HIST_DISSIMILARITY_GETTER_H

#include <cmath> // std::sqrt, atan2
#include <vector>
#include <limits>

#include <lama_common/polygon_utils.h> // for normalizePolygon
#include <place_matcher_msgs/PolygonDissimilarity.h>
#include <ros/ros.h>
#include <tf/tf.h> // for createQuaternionMsgFromYaw

namespace place_matcher_hist
{

class DissimilarityGetter
{
  public :

    DissimilarityGetter(const ros::NodeHandle& nh_private);

    bool getDissimilarity(place_matcher_msgs::PolygonDissimilarityRequest& req, place_matcher_msgs::PolygonDissimilarityResponse& res);

  private :

    typedef std::vector<unsigned int> Hist;

    // ROS parameters.
    double trans_resolution_; //!< Translational resolution (m), parameter translational_resolution.
    int max_trans_bin_count_; //!< Max. number of bins for the translational
                              //!< histogram, in the case that the number of
                              //!< bins is actually limited by this numnber,
                              //!< translational_resolution will not be respected.
    double rot_resolution_; //!< Rotational resolution (rad), parameter rotational_resolution.
    double max_dist_; //!< The normal calculed from points that are apart by
                      //!< more than max_dist_ is not included in the histogram,
                      //!< parameter max_distance_normal_considered.

    // Hard-coded parameters.
    static const size_t shift_ = 4; //!< Max. number of indexes to skip when
                                    //!< computing the normals.
                                     
    // Internals.
    void initParams();
    Hist getRotHist(const geometry_msgs::Polygon poly);
    size_t crossCorrelationHist(Hist h1, Hist h2);

    inline int getTransBinCount(double min, double max)
    {
      return (int) std::ceil((max - min) / ((double) trans_resolution_));
    }


    ros::NodeHandle nh_private_;
};

} /* namespace place_matcher_hist */

#endif /* PLACE_MATCHER_HIST_DISSIMILARITY_GETTER_H */
