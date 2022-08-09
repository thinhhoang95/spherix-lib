#include <iostream>
#include <Eigen/Dense>

#include "../spherix-dev/ConfigReader.h";
#include "../spherix-dev/Kalman.h";
#include "../spherix-dev/SpherixEncoder.h";

using namespace std;

using Eigen::MatrixXd;
using Eigen::DiagonalMatrix;
using Eigen::Vector2d;
using Eigen::VectorXd;

int main()
{
	// Initialization of the components
	ConfigReader configReader("spherixcfg.txt", 60, 5);
	Kalman filter(configReader.getMean(), configReader.getBasis(), configReader.getCov_a(), configReader.getVar_e());
	SpherixEncoder encoder(6.0, filter, 6.0);
	// VectorXd testTrajectory = configReader.getMean().transpose() + (2 * configReader.getBasis()(Eigen::seq(1, 1), Eigen::all));
	VectorXd testTrajectory{ {0, 5.1 , 7.289622256997973 , 10.959905171929858 , 16.95260986429639 , 24.35450022027362 , 33.41854052538984 , 44.40590026455466 , 57.24579716105946 , 71.69435903139669 , 84.604128808052 , 97.0693111322244 , 108.54838098428635 , 119.7402740022096 , 130.52722112096004 , 141.02883724606772 , 150.38645574738365 , 160.0655100065832 , 168.42675493053542 , 177.07306794804637 , 186.9045159595269 , 196.9348713166097 , 208.5718746224691 , 222.0469165877037 , 236.36986102526032 , 249.93196646265892 , 263.67867287740114 , 278.0788951160971 , 292.7240544175921 , 306.38071255178966 , 320.26143940602066 , 333.7691468974751 , 347.9015051629642 , 361.7653831306933 , 375.9164181698506 , 390.173207658273 , 404.38159937752187 , 418.9703699042698 , 432.6356275630864 , 446.8597210098345 , 461.6228974591601 , 475.5387671910192 , 489.36129957612513 , 503.99664398630784 , 517.8743545185258 , 531.7463362875882 , 545.8240810188792 , 560.344892576237 , 574.9979289086075 , 589.6003682763859 , 604.3800089582019 , 618.5690169565834 , 633.2604045568095 , 646.9020715872808 , 661.6703691699544 , 674.2484992897872 , 687.2696651644625 , 700.593605231334 , 713.6067750038758 , 727.7392448684672 , 741.2602764098665} };
	int messagesSent = 0;
	for (int i = 0; i < 60; i++)
	{
		encoder.m_filter.calculatePosteriori();
		if (encoder.checkQualifyToSend(testTrajectory(i)))
		{
			QDeltaVector qdv = encoder.getBroadcastMessage(true);
			// Send the message here!
			cout << "Message sent at time " << i << endl;
			messagesSent += 1;
		}
		encoder.m_filter.step(i, testTrajectory(i));
		encoder.stepTime(1); // to allow prediction in checkQualifyToSend
	}
	cout << "Total messages sent: " << messagesSent << endl;
	
}