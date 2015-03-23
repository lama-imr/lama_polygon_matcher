/********************************************
 * Server for polygon dissimilarity service based on the Multi-scale Convexity
 * Concavity (MCC) representation.
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

#include <ros/ros.h>

#include <polygon_matcher/GetCapability.h>

#include <pm_mcc/dissimilarity_getter.h>

bool callback_getCapability(polygon_matcher::GetCapabilityRequest& req, polygon_matcher::GetCapabilityResponse& res)
{
  res.scale_invariance = polygon_matcher::GetCapabilityResponse::TRUE;
  res.translation_invariance = polygon_matcher::GetCapabilityResponse::TRUE;
  res.rotation_invariance = polygon_matcher::GetCapabilityResponse::OPTIONAL;
  res.provides_dissimilarity = true;
  res.provides_pose = false;
  return true;
}

int main(int argc, char **argv)
{
  int max_thread;
  ros::init(argc, argv, "polygon_dissimilarity_server_mcc");
  ros::NodeHandle nh("~");

  nh.param<int>("max_thread", max_thread, 1);

  pm_mcc::DissimilarityGetter dissimilarity_getter;

  // Number of points to keep for the mcc computation.
  if (nh.hasParam("sample_count"))
  {
    int sample_count;
    nh.getParam("sample_count", sample_count);
    if (sample_count > 0)
    {
      dissimilarity_getter.sample_count = sample_count;
    }
    else
    {
      ROS_WARN_STREAM(nh.getNamespace() <<
          "/sample_count must be positive, setting to default (" <<
          dissimilarity_getter.sample_count << ")");
    }
  }

  // Number of scales to compute.
  if (nh.hasParam("scale_count"))
  {
    int scale_count;
    nh.getParam("scale_count", scale_count);
    if (scale_count > 0)
    {
      dissimilarity_getter.scale_count = scale_count;
    }
    else
    {
      ROS_WARN_STREAM(nh.getNamespace() <<
          "/scale_count must be positive, setting to default (" <<
          dissimilarity_getter.scale_count << ")");
    }
  }

  // With rotation_invariance = true, no cyclic optimization will be done in compare.
  nh.getParam("rotation_invariance", dissimilarity_getter.rotation_invariance);

  ros::ServiceServer get_capability_server = nh.advertiseService("get_capability", callback_getCapability);
  ros::ServiceServer compute_dissimilarity_server = nh.advertiseService("compute_dissimilarity",
      &pm_mcc::DissimilarityGetter::getDissimilarity, &dissimilarity_getter);

  ROS_INFO("Ready to work (with %i threads)", max_thread);

  ros::MultiThreadedSpinner spinner(max_thread);
  spinner.spin();

  return 0;
}
