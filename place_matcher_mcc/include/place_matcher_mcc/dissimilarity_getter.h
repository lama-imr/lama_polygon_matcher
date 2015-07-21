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

#include <lama_common/point.h>
#include <lama_common/polygon_utils.h> // for resamplePolygon().
#include <place_matcher_msgs/PolygonDissimilarity.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace place_matcher_mcc
{

typedef boost::numeric::ublas::matrix<double> matrix;
typedef std::vector<geometry_msgs::Polygon> polygon_list;

class DissimilarityGetter
{
  public :

    DissimilarityGetter();

    double getDissimilarity(const geometry_msgs::Polygon& polygon1, const geometry_msgs::Polygon& polygon2);
    bool getDissimilarity(place_matcher_msgs::PolygonDissimilarityRequest& req, place_matcher_msgs::PolygonDissimilarityResponse& res);

    // Number of scales to compute, i.e. max. sigma for the Gaussian filtering.
    unsigned int scale_count;
    
    // Number of points to keep for the mcc computation.
    unsigned int sample_count;

    // Optional rotation invariance.
    bool rotation_invariance;

  private :

    double compute_convexity(const polygon_list& polygons, matrix& m);
    matrix compare(const matrix& a, const matrix& b);
};

} /* namespace place_matcher_mcc */

#endif /* PM_MCC_DISSIMILARITY_GETTER_H */
