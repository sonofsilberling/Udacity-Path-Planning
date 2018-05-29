#include "utils.h"
#include <iostream>

#ifdef LOG

#endif


/**
 * @brief Get lane from d value
 */
Lane get_lane(double d0) {

	if (d0 < 0)
		return unknown;
    if (d0 < LANE_WIDTH)
    	return left;
    if (d0 < 2 * LANE_WIDTH)
    	return center;
    if (d0 < 3 * LANE_WIDTH)
    	return right;

    return unknown	;

};

/**
 * @brief Evaluate polynomial at point x
 */
double eval_polynomial(const double& x, const std::vector<double>& coefficients) {
	assert(coefficients.size()>0);
	double y = 0;
	for (unsigned int i=0; i<coefficients.size(); i++) {
		y += coefficients[i] * std::pow(x,i);
	}
	return y;
}

/**
 * @brief Evaluate polynomial prime at point x
 */
double eval_polynomial_dot(const double& x, const std::vector<double>& coefficients) {
	assert(coefficients.size()>0);
	if (coefficients.size() == 1) {
		return 0.0; // for single polynomial the derivative is defined as 0
	}
	double y = 0;
	for (unsigned int i=1; i<coefficients.size(); i++) {
		y += i * coefficients[i] * std::pow(x,i-1);
	}
	return y;
}

/**
 * @brief Evaluate polynomial prime prime at point x
 */
double eval_polynomial_dot_dot(const double& x, const std::vector<double>& coefficients) {
	assert(coefficients.size()>0);
	if (coefficients.size() < 3 ) {
		return 0.0; // for quatratic polynomial the double derivative is defined as 0
	}	
	return (2*coefficients[2] + 6*coefficients[3]*x + 12* coefficients[4]*x*x + 20*coefficients[5]*x*x*x);
}
