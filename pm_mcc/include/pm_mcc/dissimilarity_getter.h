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

#ifndef PM_MCC_DISSIMILARITY_GETTER_H
#define PM_MCC_DISSIMILARITY_GETTER_H

#include <algorithm>
#include <limits>
#include <cmath>
#include <cassert>

#include <boost/numeric/ublas/matrix.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <lama_common/point.h>
#include <lama_common/polygon.h>
#include <polygon_matcher/GetCapability.h>
#include <polygon_matcher/PolygonDissimilarity.h>

#include <polygon_matcher/PolygonDissimilarity.h>

namespace pm_mcc
{

typedef boost::numeric::ublas::matrix<double> matrix;
typedef std::vector<geometry_msgs::Polygon> polygon_list;

class DissimilarityGetter
{
  public :

    DissimilarityGetter();

    bool getDissimilarity(polygon_matcher::PolygonDissimilarityRequest& req, polygon_matcher::PolygonDissimilarityResponse& res);

    // Number of scales to compute, i.e. max. sigma for the Gaussian filtering.
    unsigned int scale_count;
    
    // Number of points to keep for the mcc computation.
    unsigned int sample_count;

    // Optional rotation invariance.
    bool rotation_invariance;

  private :

    double compute_convexity(const polygon_list& polygons, matrix& m);
    matrix compare(const matrix& a, const matrix& b);

    ros::ServiceServer get_capability_server_;
    ros::ServiceServer get_dissimilarity_server_;
};

} /* namespace pm_mcc */

#endif /* PM_MCC_DISSIMILARITY_GETTER_H */
