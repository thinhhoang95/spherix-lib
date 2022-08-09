#include "SpherixEncoder.h"
#include <stdlib.h>
#include <math.h>
#include <iostream>

/// <summary>
/// Initialize the Spherix Encoder
/// </summary>
/// <param name="latticeDiameter">The diameter of the rounding sphere of the functional domain</param>
/// <param name="filter">The Kalman filter</param>
SpherixEncoder::SpherixEncoder(double latticeDiameter, Kalman& filter, double coordinateTolerance) : m_filter(filter)
{
	m_basisComponents = filter.m_basisComponents;
	m_trajectoryLength = filter.m_trajectoryLength;
	m_latticeRadius = latticeDiameter;
	m_coordinateTolerance = coordinateTolerance;
	m_clientPCA = VectorXd::Zero(m_basisComponents);
}

/// <summary>
/// Check if it is necessary to rebroadcast the messages again (i.e., recipients are not up to date with the latest information)
/// </summary>
/// <param name="y">Latest observation from the object</param>
bool SpherixEncoder::checkQualifyToSend(double y)
{
	double predicted_y = m_filter.getPredictionAtt(m_clientPCA, m_time);
	std::cout << "---" << std::endl;
	std::cout << "Predicted y: " << predicted_y << ", actual y: " << y << std::endl;
	std::cout << "Client PCA: " << m_clientPCA.transpose() << std::endl;
	std::cout << "New PCA: " << m_filter.getaPosterioriMean().transpose() << std::endl;
	if (abs(predicted_y - y) > m_coordinateTolerance)
	{
		return true;
	}
	else {
		return false;
	}
}

DeltaVector SpherixEncoder::calculateRawSphericalCoordinates()
{
	VectorXd newPCA = m_filter.getaPosterioriMean();
	// currentPCA = m_clientPCA
	VectorXd deltaPCA = newPCA - m_clientPCA;
	double r = deltaPCA.norm();
	VectorXd phi(m_basisComponents - 1);
	if (m_basisComponents > 2)
	{
		for (int i = 0; i < m_basisComponents - 2; i++) // n-1 angles share the same formula
		{
			phi(i) = acos(deltaPCA(i) / deltaPCA(Eigen::seq(i, Eigen::last)).norm());
		}
	}
	// but the last angle has a different formula
	if (deltaPCA(m_basisComponents - 1) > 0)
	{
		phi(m_basisComponents - 2) = acos(deltaPCA(m_basisComponents - 2) / deltaPCA(Eigen::seq(m_basisComponents - 2, Eigen::last)).norm());
	}
	else
	{
		phi(m_basisComponents - 2) = 2 * EIGEN_PI - acos(deltaPCA(m_basisComponents - 2) / deltaPCA(Eigen::seq(m_basisComponents - 2, Eigen::last)).norm());
	}

	DeltaVector dv;
	dv.radius = r;
	dv.angles = phi;
	//std::cout << "Precise PCA Delta Vector: " << deltaPCA.transpose() << std::endl;
	return dv;
}

QDeltaVector SpherixEncoder::quantizeDeltaVector(DeltaVector dv)
{
	int qr = static_cast<int>(round(dv.radius / m_latticeRadius));
	int qang[64];
	//std::cout << "Quantized radius is " << qr << std::endl;
	if (qr > 0)
	{
		double qangblocked = 2 * atan(1 / static_cast<double>(qr)); // angle blocked by 1 sphere
		//std::cout << "qangblocked is " << qangblocked << std::endl;
		for (int i = 0; i < m_basisComponents - 1; i++) // n-1 angles share the same formula
		{
			qang[i] = static_cast<int>(round(dv.angles[i] / qangblocked));
			//std::cout << "Quantized angle " << i << " is " << qang[i] << std::endl;
		}
		QDeltaVector qdv;
		qdv.radius = qr;
		std::copy(qang, qang + m_basisComponents - 1, qdv.angles);
		return qdv;
	}
	else
	{
		QDeltaVector qdv;
		qdv.radius = 0;
		return qdv;
	}
}

QDeltaVector SpherixEncoder::getBroadcastMessage(bool updateClientPCA)
{
	DeltaVector dv = calculateRawSphericalCoordinates();
	//std::cout << "Received delta vector. Radius: " << dv.radius << ", angles: " << dv.angles << std::endl;
	QDeltaVector qdv = quantizeDeltaVector(dv);
	// Get the client PCA: first n-1 components
	VectorXd reconstructedDelta(m_basisComponents);
	
	if (qdv.radius > 0)
	{
		double radius = qdv.radius * m_latticeRadius;
		//std::cout << "Reconstruction radius is " << radius << std::endl;
		double angles[64];
		double qangblocked = 2 * atan(m_latticeRadius / radius);
		//std::cout << "decode qangblocked is " << qangblocked << std::endl;
		if (m_basisComponents > 2)
		{
			for (int i = 0; i < m_basisComponents; i++) // n-1 first components share the same calculation method
			{
				angles[i] = qdv.angles[i] * qangblocked;
				//std::cout << "Decoded angle " << i << " is " << angles[i] << std::endl;
				double x = radius;
				if (i == m_basisComponents - 1)
				{
					for (int j = 0; j < i - 1; j++)
					{
						x = x * sin(angles[j]);
					}
					x = x * sin(angles[i - 1]); // the last coordinate will have the sin instead of the cos
				}
				else
				{
					for (int j = 0; j < i; j++)
					{
						x = x * sin(angles[j]);
					}
					x = x * cos(angles[i]);
				}
				reconstructedDelta(i) = x;
			}
		}
		if (updateClientPCA)
		{
			m_clientPCA = m_clientPCA + reconstructedDelta;
		}
		//std::cout << "Client PCA delta vector: " << reconstructedDelta.transpose() << std::endl;
		return qdv;
	}
	else
	{
		reconstructedDelta = VectorXd::Zero(m_basisComponents);
		// It's a zero delta vector so no need to update m_clientPCA
		//std::cout << "Client PCA delta vector: " << reconstructedDelta.transpose() << std::endl;
		return qdv;
	}
	
}

void SpherixEncoder::stepTime(int d)
{
	m_time += d;
}