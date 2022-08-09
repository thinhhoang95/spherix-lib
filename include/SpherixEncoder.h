#pragma once

#ifndef SPHERIX_H
#define SPHERIX_H

#include <Eigen/Dense>;
#include "Kalman.h";
using Eigen::VectorXd;

struct DeltaVector {
	double radius;
	VectorXd angles;
};

struct QDeltaVector {
	int radius;
	int angles[64];
};

class SpherixEncoder
{
	double m_latticeRadius{ 1e-3 }; // quantized size of each lattice diameter
	double m_coordinateTolerance{ 1.0 }; // the tolerance error that requires retransmission of messages
	VectorXd m_clientPCA; // the euclidean coordinates of the MAP estimate the client received
	int m_basisComponents{ 2 };
	int m_trajectoryLength{ 0 };
	int m_time{ 0 };
	DeltaVector calculateRawSphericalCoordinates();
	QDeltaVector quantizeDeltaVector(DeltaVector dv);
public:
	Kalman m_filter;
	SpherixEncoder(double latticeDiameter, Kalman& filter, double coordinateTolerance);
	bool checkQualifyToSend(double y);
	QDeltaVector getBroadcastMessage(bool updateClientPCA);
	void stepTime(int d);
};

#endif // !
