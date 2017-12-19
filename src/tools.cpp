#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */	
	VectorXd RMSE(4);
	RMSE.fill(0.0);

	float length = estimations.size();

	// check if it's ok to go on
	if (estimations.size() != ground_truth.size() || estimations.size() == 0)
	{
		std::cout << "Invalid estimation or ground truth data! \n";
		return RMSE;
	}
	else {
		for (int i = 0; i < length; i++) {
			VectorXd residuals = estimations[i] - ground_truth[i];
			residuals = residuals.array() * residuals.array();
			RMSE += residuals;
		}

		RMSE /= length;
		RMSE = RMSE.array().sqrt();
		return RMSE;
	}
}