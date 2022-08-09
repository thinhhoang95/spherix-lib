#include "Kalman.h"
#include <iostream>

/// <summary>
/// Initialization of the Kalman filter by covariance parameters
/// </summary>
/// <param name="cov_a">Covariance of the representation distribution</param>
/// <param name="cov_e">Covariance of measurement</param>
/// <param name="mean_fun">Mean functional of the FPCA decomposition</param>
Kalman::Kalman(const VectorXd& mean, const MatrixXd& basis, const MatrixXd& cov_a, const double var_e)
{
	// Set the status of the filter to intialized
	m_filterInitialized = true;
	// Initialize the parameters
	m_cov_a = cov_a;
	m_var_e = var_e;
	m_basis = basis;
	// Calculate the precision
	m_precision_a = cov_a.inverse();
	m_precision_e = 1 / var_e;
	m_mean = mean;
	// Set the metaparametric configuration states
	m_trajectoryLength = mean.size();
	m_basisComponents = basis.rows();
	// printf("Kalman: Trajectory length: %d \n", m_trajectoryLength);
	// printf("Kalman: Number of basis components: %d \n", m_basisComponents);
	// Initialize the internal state matrices
	Cx = MatrixXd::Zero(m_basisComponents, m_basisComponents);
	Ct = VectorXd::Zero(m_basisComponents);
	// printf("Kalman: filter was initialized\n");
}

/// <summary>
/// Fuse one observation in real-time to the prior knowledge
/// </summary>
/// <param name="t">Timestep</param>
/// <param name="y">1D observation value</param>
void Kalman::step(int t, double y)
{
	posterioriUpToDate = false;
	VectorXd phi = m_basis(Eigen::all, Eigen::seq(t,t));
	MatrixXd newCx = Cx + m_precision_e * phi * phi.transpose();
	VectorXd newCt = Ct + m_precision_e * (y - m_mean(t)) * phi;
	Cx = newCx;
	Ct = newCt;
}

void Kalman::calculatePosteriori()
{
	MatrixXd new_prec = m_precision_a + Cx;
	posterioriCov = new_prec.inverse();
	posterioriMean = posterioriCov * Ct;
	posterioriUpToDate = true;
}

/// <summary>
/// Get the a-posteriori estimate of the mean
/// </summary>
VectorXd Kalman::getaPosterioriMean()
{
	if (posterioriUpToDate)
	{
		return posterioriMean;
	}
	else
	{
		throw std::runtime_error("Error: posteriori values are not up to date, may be forgot to call calculatePosteriori?");
	}
}

MatrixXd Kalman::getaPosterioriCov()
{
	if (posterioriUpToDate)
	{
		return posterioriCov;
	}
	else
	{
		throw std::runtime_error("Error: posteriori values are not up to date, may be forgot to call calculatePosteriori?");
	}
}

double Kalman::getPredictionAtt(VectorXd scores, int t)
{
	MatrixXd y_hat = m_basis(Eigen::all, Eigen::seq(t, t)).transpose() * scores;
	return y_hat.value() + m_mean(t);
}