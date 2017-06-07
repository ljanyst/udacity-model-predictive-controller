//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   26.05.2017
//------------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

//------------------------------------------------------------------------------
//! Evaluate a polynomial at a point
//------------------------------------------------------------------------------
double PolyEval(const Eigen::VectorXd &coeffs, double x);

//------------------------------------------------------------------------------
// Fit a polynomial to a set of points.
//------------------------------------------------------------------------------
Eigen::VectorXd PolyFit(const Eigen::VectorXd &xvals,
                        const Eigen::VectorXd &yvals,
                        int                    order);
