//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   26.05.2017
//------------------------------------------------------------------------------

#include "utils.hh"

//------------------------------------------------------------------------------
//! Convert waypoints to the vehicle coordinate system
//------------------------------------------------------------------------------
void ConvertWaypoints(std::vector<double>       &local_xs,
                      std::vector<double>       &local_ys,
                      const std::vector<double> &global_xs,
                      const std::vector<double> &global_ys,
                      double x, double y, double psi) {
  for(int i = 0; i < global_xs.size(); ++i) {
    double dist_x  = global_xs[i] - x;
    double dist_y  = global_ys[i] - y;
    double dist    = sqrt(dist_x*dist_x + dist_y*dist_y);
    if(dist < 10e-5) {
      local_xs.push_back(0);
      local_ys.push_back(0);
      continue;
    }
    double theta = atan2(dist_y, dist_x) - psi;
    local_xs.push_back(dist * cos(theta));
    local_ys.push_back(dist * sin(theta));
  }
}
