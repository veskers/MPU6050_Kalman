#ifndef _Kalman_h
#define _Kalman_h

typedef struct
{
    double Q_angle; // Process noise variance for the accelerometer
    double Q_bias; // Process noise variance for the gyro bias
    double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    double angle; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
    double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
    double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    double K[2]; // Kalman gain - This is a 2x1 matrix
    double y; // Angle difference - 1x1 matrix
    double S; // Estimate error - 1x1 matrix
} Kalman;


void initKalmanStruct(Kalman *coord);
double getAngle(Kalman *coord, double newAngle, double newRate, double dt);
void setAngle(Kalman *coord, double newAngle);
double getRate();
void setQangle(double newQ_angle);
void setQbias(double newQ_bias);
void setRmeasure(double newR_measure);

#endif
