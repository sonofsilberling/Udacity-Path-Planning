#pragma once

#ifndef UTILS_CARND
#define UTILS_CARND


#include "constants.h"
#include <assert.h>
#include <cmath>
#include <vector>
#include <map>
#include <ostream>
#include <iterator>

#ifdef LOG
#include <glog/logging.h>
#endif

/**
 * @brief Lane type
 */
enum Lane { 
	left, 
	center, 
	right,
	unknown
};

// std::map<Lane, double> lane_centers;

/**
 * @brief Get lane from d value
 */
Lane get_lane(double d0) ;

/**
 * @brief Evaluate polynomial at point x
 */
double eval_polynomial(const double& x, const std::vector<double>& coefficients) ;

/**
 * @brief Evaluate polynomial prime at point x
 */
double eval_polynomial_dot(const double& x, const std::vector<double>& coefficients) ;

/**
 * @brief Evaluate polynomial prime at point x
 */
double eval_polynomial_dot_dot(const double& x, const std::vector<double>& coefficients) ;

/**
 * @brief Output vector
 */
template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
  if ( !v.empty() ) {
    out << '[';
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

/**
 * @brief Determine euclidean distance of two points
 */
// double get_distance(double x1, double y1, double x2, double y2) ;


#endif