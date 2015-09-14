/********************************************
 * Jockey for the Large Maps Framework with polygon dissimilarity based on the
 * Multi-scale Convexity Concavity (MCC) representation.
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

#include <place_matcher_msgs/GetCapability.h>

#include <place_matcher_mcc/jockey.h>

bool callback_getCapability(place_matcher_msgs::GetCapabilityRequest& req, place_matcher_msgs::GetCapabilityResponse& res)
{
  res.scale_invariance = place_matcher_msgs::GetCapabilityResponse::TRUE;
  res.translation_invariance = place_matcher_msgs::GetCapabilityResponse::TRUE;
  res.rotation_invariance = place_matcher_msgs::GetCapabilityResponse::OPTIONAL;
  res.provides_dissimilarity = true;
  res.provides_pose = false;
  return true;
}

int main(int argc, char **argv)
{
  int max_thread;
  ros::init(argc, argv, "lj_mcc");
  ros::NodeHandle nh("~");

  nh.param<int>("max_thread", max_thread, 1);


  std::string localizing_jockey_server;
  nh.param<std::string>("localizing_jockey_server_name",
      localizing_jockey_server, ros::this_node::getName() + "_server");

  place_matcher_mcc::Jockey jockey(localizing_jockey_server);

  // Number of points to keep for the mcc computation.
  if (nh.hasParam("sample_count"))
  {
    int sample_count;
    nh.getParam("sample_count", sample_count);
    if (sample_count > 0)
    {
      jockey.setSampleCount(sample_count);
    }
    else
    {
      ROS_WARN_STREAM(nh.getNamespace() <<
          "/sample_count must be positive, setting to default (" <<
          jockey.getSampleCount() << ")");
    }
  }

  // Number of scales to compute.
  if (nh.hasParam("scale_count"))
  {
    int scale_count;
    nh.getParam("scale_count", scale_count);
    if (scale_count > 0)
    {
      jockey.setScaleCount(scale_count);
    }
    else
    {
      ROS_WARN_STREAM(nh.getNamespace() <<
          "/scale_count must be positive, setting to default (" <<
          jockey.getScaleCount() << ")");
    }
  }

  // With rotation_invariance = true, no cyclic optimization will be done in compare.
  bool rotation_invariance = jockey.getRotationInvariance();
  nh.getParam("rotation_invariance", rotation_invariance);
  jockey.setRotationInvariance(rotation_invariance);

  ros::ServiceServer get_capability_server = nh.advertiseService("get_capability", callback_getCapability);

  ROS_INFO("Ready to work (with %i threads)", max_thread);

  ros::MultiThreadedSpinner spinner(max_thread);
  spinner.spin();

  return 0;
}

