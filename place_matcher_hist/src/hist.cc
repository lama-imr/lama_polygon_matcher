#include <math.h>
#include <vector>
#include <stdio.h>
#include "place_matcher_hist/hist.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#define SHIFT 5 

Hist hist(const sensor_msgs::LaserScan& scan, int anglebins)
{
  int angleHistBin = anglebins;
  double bin_width = angleHistBin / ( 2*M_PI);
  Hist hist(angleHistBin,0);
  double px1;
  double px2;
  double py1;
  double py2;
  for (unsigned int shift = 1 ; shift < SHIFT; shift++)
  {
    px1 = scan.ranges[0] * sin(0 * scan.angle_increment + scan.angle_min);
    py1 = -scan.ranges[0] * cos(0 * scan.angle_increment + scan.angle_min);   

    for(unsigned int i = 1; i < scan.ranges.size() - SHIFT ; i++)
    {
      px2 = scan.ranges[i+SHIFT] * sin((i+SHIFT) * scan.angle_increment + scan.angle_min);
      py2 = -scan.ranges[i+SHIFT] * cos((i+SHIFT) * scan.angle_increment + scan.angle_min);   
      const double distance = std::sqrt((py1 - py2) * (py1 - py2) + (px1 - px2) * (px1 - px2));
      if (distance  < 0.1)
      {
        double ath = std::atan2(py1 - py2, px1 - px2);
        if (ath <0)
        {
          ath= ath + 2*M_PI;
        }
        hist[(int) floor(ath * bin_width)] += 1;
      }
      px1 = px2;
      py1 = py2;
      px1 = scan.ranges[i] * sin((i) * scan.angle_increment + scan.angle_min);
      py1 = -scan.ranges[i] * cos((i) * scan.angle_increment + scan.angle_min);   
    }
  }
  return hist;
}

int crossCorrelationHist(Hist x , Hist y)
{
  double max = 0.0;
  int maxind = 0;
  for (unsigned int shift = 0; shift < x.size(); shift++)
  {
    double sum = 0; 
    for (unsigned int i = 0; i < x.size(); i++)
    {
      sum += x[i] * y[(i + shift) % y.size()];
    }
    if (sum > max)
    {
      max = sum;
      maxind = shift;
    }
  }
  return maxind;
}


geometry_msgs::Pose2D localize(sensor_msgs::LaserScan a, sensor_msgs::LaserScan b, int rot_bin_count)
{
  double trans_bin_width = TRANSCOEF;
  double resolution = 2 * M_PI / a.ranges.size();
  double rot_bin_width = rot_bin_count / (2 * M_PI);

  Hist histA = hist(a, rot_bin_count);
  Hist histB = hist(b, rot_bin_count);
  int angleIndex = crossCorrelationHist(histA,histB);
  int histAngleMaxIndex=0; 
  int histMax = 0;
  for (unsigned int i = 0; i < histA.size(); i++)
  {
    if (histMax < histA[i]) {
      histMax = histA[i];
      histAngleMaxIndex = i;
    }
  }
  int histAngleMaxIndexB=0; 
  int histMaxB = 0;
  for (unsigned int i = 0; i < histB.size(); i++)
  {
    if (histMaxB < histB[i]) {
      histMaxB = histB[i];
      histAngleMaxIndexB = i;
    }
  }
  std::cout << "hist angle  A " << histAngleMaxIndex  << " B " << histAngleMaxIndexB << " cross " << angleIndex << " diff " << histAngleMaxIndex - histAngleMaxIndexB << std::endl;

  // compute projections into two perpendicular planes in direction of biggest peak in angle hist
  Hist histax (HISTSIZE ,0);
  Hist histay (HISTSIZE ,0);
  for (unsigned int i = 0; i < a.ranges.size(); i++)
  {
    if (a.ranges[i] < 7.9)
    {
      histax[(int) floor(trans_bin_width * a.ranges[i] * cos(a.angle_min + i * resolution - histAngleMaxIndex / rot_bin_width)) + HISTSIZEHALF]++;
      histay[(int) floor(trans_bin_width * a.ranges[i] * sin(a.angle_min + i * resolution - histAngleMaxIndex / rot_bin_width)) + HISTSIZEHALF]++;
    }
  }
  // compute projections into two perpendicular planes also for scan b 
  Hist histbx (HISTSIZE ,0);
  Hist histby (HISTSIZE ,0);
  for (unsigned int i = 0; i < b.ranges.size(); i++)
  {
    if (b.ranges[i] < 7.9)
    {
      histbx[(int) floor(trans_bin_width * b.ranges[i] * cos(b.angle_min + i * resolution - histAngleMaxIndex / rot_bin_width - (angleIndex) / rot_bin_width)) + HISTSIZEHALF]++;
      histby[(int) floor(trans_bin_width * b.ranges[i] * sin(b.angle_min + i * resolution - histAngleMaxIndex / rot_bin_width - (angleIndex) / rot_bin_width)) + HISTSIZEHALF]++;
    }
  }
  int xIndex = crossCorrelationHist(histax, histbx);
  int yIndex = crossCorrelationHist(histay, histby);
  if (xIndex > HISTSIZEHALF) xIndex -= HISTSIZE;
  if (yIndex > HISTSIZEHALF) yIndex -= HISTSIZE;
  std::cout << xIndex/trans_bin_width << " " << yIndex/trans_bin_width << " angle index " << angleIndex/rot_bin_width << " max hist angle " << histAngleMaxIndex/rot_bin_width <<  "cos " << cos(-histAngleMaxIndex / rot_bin_width) << " sin " << sin(-histAngleMaxIndex/rot_bin_width) << "\n";
  geometry_msgs::Pose2D ret ;
  //    double ty = cos(-histAngleMaxIndex / rot_bin_width) * yIndex / trans_bin_width - sin(-histAngleMaxIndex / rot_bin_width) * xIndex / trans_bin_width;
  //    double tx = (xIndex / trans_bin_width + sin(-histAngleMaxIndex / rot_bin_width) * ty) / cos(-histAngleMaxIndex / rot_bin_width);
  double x = xIndex / trans_bin_width;
  double y = yIndex / trans_bin_width;
  double cosin = cos(histAngleMaxIndex/rot_bin_width);
  double sinus = sin(histAngleMaxIndex/rot_bin_width);
  double xx = cosin*x - sinus* y;
  double yy = sinus*x + cosin*y;
  std::cout << "x " << x << " y " << y << " cos " << cosin << " sin " << sinus << " xx " <<xx << " YY " << yy<< std::endl;
  double tx = (cos(histAngleMaxIndex /rot_bin_width) * (xIndex / trans_bin_width)) - (sin( histAngleMaxIndex) * (yIndex / trans_bin_width));
  double ty = (sin(histAngleMaxIndex /rot_bin_width) * (xIndex / trans_bin_width)) + (cos( histAngleMaxIndex) * (yIndex / trans_bin_width));
  std::cout << " tx ty " << tx << " " << ty << std::endl; 
  ret.x = -yy;
  ret.y = -xx;
  ret.theta= 2*M_PI-angleIndex/rot_bin_width;
  /*  ret.px = ty;
      ret.py = -tx;*/

  return ret;
}

double error(sensor_msgs::LaserScan scan, sensor_msgs::LaserScan reference, geometry_msgs::Pose2D goal)
{
  double rx, ry,sx,sy;
  double error = 0;
  double distance ;
  double min = 9000000;
  int count = 0;
  geometry_msgs::PolygonStamped poly;
  poly.header.frame_id="/mf";
  geometry_msgs::PolygonStamped poly2;
  poly2.header.frame_id="/mf";
  geometry_msgs::Point32 p;
  for (unsigned int i = 0 ; i < reference.ranges.size(); i++) {
    min = 90000000;
    if (reference.ranges[i] < 7.9) {
      rx = reference.ranges[i] * sin( (i * reference.angle_increment) + reference.angle_min + goal.theta )  + goal.x;
      ry =  -reference.ranges[i] * cos( (i * reference.angle_increment) + reference.angle_min + goal.theta) + goal.y;

      for (unsigned int  j = 0 ; j < scan.ranges.size(); j++) {
        if (scan.ranges[j] < 7.9){
          sx = scan.ranges[j] * sin( (j * scan.angle_increment) + scan.angle_min);
          sy =  -scan.ranges[j] * cos( (j * scan.angle_increment) + scan.angle_min); 
          distance = sqrt((rx-sx)*(rx-sx) + (ry-sy)*(ry-sy));
          if (distance < min) {
            min = distance;
          }
        }
      }
      error += min;
      count++;
    }
  }
  return error / count;
}


