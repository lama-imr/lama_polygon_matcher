#ifndef __HISTOGRAM
#define __HISTOGRAM
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
typedef std::vector<int> Hist;
Hist hist(sensor_msgs::LaserScan scan, int bins);
geometry_msgs::Pose2D localize (sensor_msgs::LaserScan a, sensor_msgs::LaserScan b, int anglebins) ;
int crossCorrelationHist (Hist x , Hist y) ;
double error(sensor_msgs::LaserScan scan, sensor_msgs::LaserScan reference, geometry_msgs::Pose2D goal, ros::Publisher draw ,ros::Publisher drawRef); 
double error(sensor_msgs::LaserScan scan, sensor_msgs::LaserScan reference, geometry_msgs::Pose2D goal); 

#define ANGLEBIN 100
#define TRANSCOEF 10
#define HISTSIZE 500
#define HISTSIZEHALF 250
#endif
