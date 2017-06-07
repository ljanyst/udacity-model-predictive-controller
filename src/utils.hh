//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   26.05.2017
//------------------------------------------------------------------------------

#pragma once

#include <cmath>
#include <vector>

//------------------------------------------------------------------------------
//! Convert degrees to radians
//------------------------------------------------------------------------------
inline double Deg2Rad(double x) { return x * M_PI / 180; }

//------------------------------------------------------------------------------
//! Calculate distance between points
//------------------------------------------------------------------------------
inline double CalculateDistance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

//------------------------------------------------------------------------------
//! Convert waypoints to the vehicle coordinate system
//------------------------------------------------------------------------------
void ConvertWaypoints(std::vector<double>       &local_xs,
                      std::vector<double>       &local_ys,
                      const std::vector<double> &global_xs,
                      const std::vector<double> &global_ys,
                      double x, double y, double psi);
