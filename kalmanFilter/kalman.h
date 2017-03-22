#pragma once
// kalman.h

//#ifdef KALMANDLL_EXPORTS
//#define KALMANDLL_API __declspec(dllexport) 
//#else
//#define KALMANDLL_API __declspec(dllimport) 
//#endif

class Kalman
{
public:
	 __declspec(dllexport)  Kalman();
	 __declspec(dllexport)  double getAngle(double newAngle, double newRate, double dt);
	 __declspec(dllexport)  void setAngle(double newAngle);
	 __declspec(dllexport)  double getRate();
	 __declspec(dllexport)  void setQangle(double newQ_angle);
	 __declspec(dllexport)  void setQbias(double newQ_bias);
	 __declspec(dllexport)  void setRmeasure(double newR_measure);

	 __declspec(dllexport)  double getQangle();
	 __declspec(dllexport)  double getQbias();
	 __declspec(dllexport)  double getRmeasure();

private:
	/* Kalman filter variables */
	 double Q_angle; // Process noise variance for the accelerometer
	 double Q_bias; // Process noise variance for the gyro bias
	 double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

	 double angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
	 double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	 double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	 double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
	 double K[2]; // Kalman gain - This is a 2x1 vector
	 double y; // Angle difference
	 double S; // Estimate error
};