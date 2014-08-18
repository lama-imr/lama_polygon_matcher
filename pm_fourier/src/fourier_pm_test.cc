
#include "CFourierPMatcher.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
   FourierPmatcher zmpm;
   ros::init(argc, argv, "fourier_tester");
   ros::NodeHandle n;
   ros::ServiceServer service = n.advertiseService<polygon_matcher::polygon_similarity::Request, polygon_matcher::polygon_similarity::Response>("fourier_matcher", boost::bind (&FourierPmatcher::similarity, &zmpm,_1,_2)); 
   ros::spin();
   return 0;
}
