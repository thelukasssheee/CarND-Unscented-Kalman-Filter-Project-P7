#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Prepare RMSE variable and initialize with zeros
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // Script abortion: estimation vector too small or size unequal to ground truth vector
  if (estimations.size() < 1) {
    return rmse;
  }
  if (estimations.size() != ground_truth.size()) {
    return rmse;
  }

  // Calculate squared residuals
  for (unsigned int j=1; j < estimations.size(); ++j) {
    // Calculate
    VectorXd resid = estimations[j] - ground_truth[j];

    // Coefficient-wise multiplication
    resid = resid.array() * resid.array();
    rmse += resid;
  }

  // Calculate the mean
  rmse = rmse / estimations.size();

  // Calculate square root of RMSE
  rmse = rmse.array().sqrt();

  // Return RMSE result, exit function
  return rmse;
}
