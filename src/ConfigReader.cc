#include "ConfigReader.h"

ConfigReader::ConfigReader(string pathToFile, int trajectoryLength, int basisComponents)
{
    double coordinates[512];
    MatrixXd basis(basisComponents, trajectoryLength);
    MatrixXd cov_a(basisComponents, basisComponents);

    std::ifstream inFile(pathToFile);
    if (inFile.is_open())
    {
        std::string line;

        // First line of the config file contains the mean vector
        std::getline(inFile, line);
        std::stringstream ss(line);
        

        std::string x;
        for (int i = 0; i < trajectoryLength; i++)
        {
            std::getline(ss, x, ',');
            coordinates[i] = stod(x);
        }
        m_mean = Map<VectorXd>(coordinates, trajectoryLength);

        // The next basisComponents lines will define the basis matrix
        for (int i = 0; i < basisComponents; i++)
        {
            std::getline(inFile, line);
            ss = std::stringstream(line);
            for (int j = 0; j < trajectoryLength; j++)
            {
                std::getline(ss, x, ',');
                basis(i,j) = stod(x);
            }
        }
        m_basis = basis;

        // The next basisComponents lines will define the covariance of the Gaussian distribution of representations
        for (int i = 0; i < basisComponents; i++)
        {
            std::getline(inFile, line);
            ss = std::stringstream(line);
            for (int j = 0; j < basisComponents; j++)
            {
                std::getline(ss, x, ',');
                cov_a(i, j) = stod(x);
            }
        }
        m_cov_a = cov_a;

        // The final line will define the measurement variance
        std::getline(inFile, line);
        m_var_e = stod(line);
        // printf("ConfigReader: prior data was loaded successfully\n");
    }
}

VectorXd ConfigReader::getMean()
{
    return m_mean;
}
MatrixXd ConfigReader::getBasis()
{
    return m_basis;
}

MatrixXd ConfigReader::getCov_a()
{
    return m_cov_a;
}

double ConfigReader::getVar_e()
{
    return m_var_e;
}
