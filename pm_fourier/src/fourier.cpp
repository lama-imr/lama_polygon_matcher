#include <vector>
#include <cmath>
#include <algorithm>
#include <list>
#include <numeric>
#include <iostream>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_fft_real.h>
#include <gsl/gsl_fft_halfcomplex.h>
#include <gsl/gsl_fft_complex.h>

#include <pm_fourier/angle_shift.h>
#include <lama_common/point.h>

using std::vector;
using lama_common::Point2;

double getSimpleDissimilarityFft(const vector<double> &fft1, const vector<double> &fft2, const int size)
{

  if (fft1.size() != fft2.size() || fft1.size() == 0 || fft2.size() == 0)
  {
    std::cerr << __FUNCTION__ << ": desA.size()!=desB.size() (" << fft1.size();
    std::cerr << "!=" << fft2.size() << ")!\n";
    return 0;
  }

  size_t size_ok = size;
  if (size_ok > fft1.size() - 1)
    size_ok = fft1.size() - 1;

  vector<double> a(size_ok, 0.0);
  vector<double> b(size_ok, 0.0);

  for(size_t i = 0; i < size_ok; i++)
  {
    a[i] = sqrt(fft1[2 * i] * fft1[2 * i] + fft1[2 * i + 1] * fft1[2 * i + 1]);
    b[i] = sqrt(fft2[2 * i] * fft2[2 * i] + fft2[2 * i + 1] * fft2[2 * i + 1]);
    //only real part
    //a[i] = (fft1[2*i]);
    //b[i] = (fft2[2*i]);
  }

  double r = 0;
  for(size_t i = 0; i < a.size(); i++)
  {
    r += sqrt((a[i] - b[i]) * (a[i] - b[i]));
  }
  r /= a.size();
  return r;
}


/** convert a polygon to a 'range' similar to LaserScan.ranges
 *
 * Convert the points to polar coordinates and just ignore the phase part.
*/
static vector<double> convertPolygonToRange(const vector<Point2> &polygon)
{
  vector<double> r;
  r.reserve(polygon.size());

  double sx = 0;
  double sy = 0;

  // Compute the center of the polygon.
  for (size_t i = 0; i < polygon.size(); i++)
  {
    sx += polygon[i].x;
    sy += polygon[i].y;
  }
  sx /= (double)polygon.size();
  sy /= (double)polygon.size();

  for (size_t i = 0; i< polygon.size(); i++)
  {
    const double dx = sx - polygon[i].x;
    const double dy = sy - polygon[i].y;
    const double d = std::sqrt(dx * dx + dy * dy);
    r.push_back(d);
  }
  return r;
}


double getDissimilarityFourier(const vector<Point2> &polygon1, const vector<Point2> &polygon2, const int fftSize)
{
  if (fftSize <= 0)
  {
	  std::cerr << "fftSize must be >0 !\n";
  }

  vector<double> range1(convertPolygonToRange(polygon1));
  vector<double> range2(convertPolygonToRange(polygon2));

  vector<double> f1(fft2(range1));
  vector<double> f2(fft2(range2));
  vector<double> a1;
  vector<double> a2;

  a1.reserve(f1.size());
  a2.reserve(f1.size());

  for(int i=0;i<(int)f1.size()/2;i++) {
    a1.push_back(f1[2*i]);
    a1.push_back(f1[2*i+1]);

    a2.push_back(f2[2*i]);
    a2.push_back(f2[2*i+1]);
  }

  //const double distance = getDissimilarityNccFft(a1,a2,fftSize);
  const double distance = getSimpleDissimilarityFft(a1,a2,fftSize);

  range1.clear();
  range2.clear();
  f1.clear();
  f2.clear();
  a1.clear();
  a2.clear();

  return distance;

}


/* returns dissimilarity through cross correlation */
double getDissimilarityCorrelation(const vector<double> &range1, const vector<double> &range2)
{
  vector<double> f1(fft2(range1));
  vector<double> f2(fft2(range2));

  double val;
  getAngleShiftFFT(f1, f2, val);

  return val;
}


/* returns dissimilarity through cross correlation */
double getDissimilarityCorrelation(const int n1, const int n2, double *des1, double *des2)
{
  /*
     vector<double> range1(getDescriptorData(n1,des1));	
     vector<double> range2(getDescriptorData(n2,des2));	
     vector<double> cncc(curvesNcc(range1,range2));

     double val = *std::max_element(cncc.begin(),cncc.end());
     range1.clear();
     range2.clear();
     */

  return 0;//val;
}

double getDissimilarityNccFft(const vector<double> &fft1, const vector<double> &fft2, const int size)  
{
  if (fft1.size() != fft2.size() || fft1.size() == 0 || fft2.size() == 0)
  {
    std::cerr << __FUNCTION__ << ": desA.size()!=desB.size() (" << fft1.size();
    std::cerr << "!=" << fft2.size() << ")!\n";
    return 0;
  }

  size_t size_ok = size;
  if (size_ok > fft1.size() - 1)
    size_ok = fft1.size() - 1;

  vector<double> a(size_ok, 0.0);
  vector<double> b(size_ok, 0.0);

  for(size_t i = 0; i < size_ok; i++)
  {
    a[i] = fft1[2 * i] * fft1[2 * i] + fft1[2 * i + 1] * fft1[2 * i + 1];
    b[i] = fft2[2 * i] * fft2[2 * i] + fft2[2 * i + 1] * fft2[2 * i + 1];
  }

  double meanA = std::accumulate(a.begin(), a.end(), 0.0) / a.size();
  double meanB = std::accumulate(b.begin(), b.end(), 0.0) / b.size();

  for (size_t i = 0; i < a.size(); i++)
  {
    a[i] = a[i] - meanA;
    b[i] = b[i] - meanB;
  }

  double sa = 0;
  double sb = 0;
  for(size_t i = 0; i < a.size(); i++)
  {
    sa += a[i] * a[i];
    sb += b[i] * b[i];
  }
  sa = sqrt(sa / a.size());
  sb = sqrt(sb / b.size());

  if (sa == 0  || sb == 0)
  {
    return 0;
  }

  for (size_t i = 0; i < a.size(); i++)
  {
    a[i] = a[i] / sa;
    b[i] = b[i] / sb;
  }

  double r = 0;
  for(int i = 0; i < a.size(); i++)
  {
    r += a[i] * b[i];
  }
  r /= a.size();

  r = (r + 1) / 2.0;

  // r = 0.99xxxx --> r = 0.xxxx
  r =  (10000.0 * r - 9900.0) / 100.0;
  if (r < 0)
    return 0;

  return r;
}

