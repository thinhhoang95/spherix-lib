#pragma once

#ifndef CONFIG_READER_H
#define CONFIG_READER_H

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Map;
using namespace std;

class ConfigReader {
	VectorXd m_mean;
	MatrixXd m_basis;
	MatrixXd m_cov_a;
	double m_var_e{ 1e-6 };
public:
	ConfigReader(string pathToFile, int trajectoryLength, int basisComponents);
	VectorXd getMean();
	MatrixXd getBasis();
	MatrixXd getCov_a();
	double getVar_e();
};

#endif