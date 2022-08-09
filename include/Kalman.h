#pragma once
#ifndef KALMAN_H
#define KALMAN_H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Kalman {
	// Parametric configurations
	MatrixXd m_cov_a; // covariance of the representation's Gaussian distribution
	MatrixXd m_precision_a; // precision of the repr. distribution, should be the inverse of cov_a
	double m_var_e; // covariance of measurement error
	double m_precision_e; // precision of measurement error, should be the inverse of cov_e
	VectorXd m_mean; // mean functional of the FPCA decomposition
	MatrixXd m_basis; // the matrix whose each row is a basis function
	// Metaparametric configurations
	
	bool m_filterInitialized{ false }; // check if the Kalman filter has been properly initialized
	// Internal states
	MatrixXd Cx;
	VectorXd Ct;
	bool posterioriUpToDate = false; // implies that getaPosterioriMean and getaPosterioriCov can be called directly without recalculation
	VectorXd posterioriMean;
	MatrixXd posterioriCov;
	
public:
	Kalman(const VectorXd& mean, const MatrixXd& basis, const MatrixXd& cov_a, const double var_e);
	int m_trajectoryLength{ 0 }; // length of the trajectory/basis functionals
	int m_basisComponents{ 0 }; // number of basis components
	void step(int t, double y);
	void calculatePosteriori();
	VectorXd getaPosterioriMean();
	MatrixXd getaPosterioriCov();
	double getPredictionAtt(VectorXd scores, int t);
};

#endif