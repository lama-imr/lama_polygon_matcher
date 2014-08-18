
#ifndef LALOC_SPOINT_H_
#define LALOC_SPOINT_H_

#include <iostream>
#include <cmath>

struct SPoint
{
  double x;
  double y;
  SPoint():x(0),y(0) {}
  SPoint(const double xx, const double yy):x(xx),y(yy) {}
  SPoint(const SPoint &p):x(p.x),y(p.y){}
  bool operator==(const SPoint &p) {
    return (p.x == x && p.y == y);
  }
  friend std::ostream &operator<<(std::ostream &os, const SPoint &p);
};

template<typename T>
double pointDistanceEucleidSquared(const T &pa, const T &pb)
{
  const double dx = pa.x - pb.x;
  const double dy = pa.y - pb.y;
  return dx * dx + dy * dy;
}

template<typename T>
double pointDistanceEucleid(const T &pa, const T &pb)
{
  return std::sqrt(pointDistanceEucleidSquared(pa, pb));
}

#endif // LALOC_SPOINT_H_

