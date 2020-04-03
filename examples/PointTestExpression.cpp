/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SFMExample.cpp
 * @brief   A structure-from-motion problem on a simulated dataset
 * @author  Duy-Nguyen Ta
 */

// For loading the data, see the comments therein for scenario (camera rotates around cube)
#include "SFMdata.h"

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Projection factors to model the camera's landmark observations.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use a
// trust-region method known as Powell's Degleg
#include <gtsam/nonlinear/DoglegOptimizer.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/geometry/Point3.h>

#include <vector>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

  // Create the set of ground-truth landmarks
  vector<Point3> points = createPoints();
  vector<vector<int> > mapping {
        { 0, 2, 7 },
        { 0, 5, 7 },
        { 1, 4, 6 },
        { 2, 5, 7 },
        { 3, 4, 6 }
     };

  // Create a factor graph
  ExpressionFactorGraph graph;
  
  // Add prior on 3 points to constrain it
  noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.01);
  for(size_t i=0; i<3; i++){
    graph.addExpressionFactor(Point3_('l',i), points[i], pointNoise); // add directly to graph
  }

  
  auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.4, 0.4, 0.4));
  for(size_t i=3; i<8; i++){
    for(size_t j=0; j<3; j++){
      int idx_i = i;
      int idx_j = mapping[i-3][j];
      Point3 measurement = points[idx_j] - points[i];
      graph.addExpressionFactor(between(Point3_('l',idx_i), Point3_('l',idx_j)), measurement, odometryNoise);
    }
  }
  
  graph.print("Factor Graph:\n");
  
  Values initials;
  for (size_t j = 0; j < points.size(); ++j){
    if(j<3)
      initials.insert(Symbol('l', j), points[j]);
    else{
      Point3 noisy_initial = points[j] + Point3(-0.25, 0.20, 0.15);
      initials.insert(Symbol('l', j), noisy_initial);
    }
  }
  initials.print("Initial Estimates:\n");
  
  // Optimize the graph and print results 
  Values result = DoglegOptimizer(graph, initials).optimize();
  result.print("Final results:\n");
  //cout << "initial error = " << graph.error(initialEstimate) << endl;
  //cout << "final error = " << graph.error(result) << endl;
  //*/
  return 0;
}
/* ************************************************************************* */

