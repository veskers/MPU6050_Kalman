#include "Kalman.h"


void initKalmanStruct(Kalman *coord)
{
	coord->Q_angle = 0.001;
	coord->Q_bias = 0.003;
	coord->R_measure = 0.03;
	coord->P[0][0] = 0.00;
	coord->P[0][1] = 0.00;
	coord->P[1][1] = 0.00;
	coord->P[1][0] = 0.00;
	coord->K[0] = 0.00;
	coord->K[1] = 0.00;
	coord->y = 0.00;
	coord->S = 0.03;
	coord->angle = 0.00;
	coord->rate = 0.00;
	coord->bias = 0.00;
}

double getAngle(Kalman *coord, double newAngle, double newRate, double dt)
{
	coord->rate = newRate - coord->bias;
	coord->angle += dt * coord->rate;

	coord->P[0][0] += dt * (dt*coord->P[1][1] - coord->P[0][1] - coord->P[1][0] + coord->Q_angle);
	coord->P[0][1] -= dt * coord->P[1][1];
	coord->P[1][0] -= dt * coord->P[1][1];
	coord->P[1][1] += coord->Q_bias * dt;

	coord->S = coord->P[0][0] + coord->R_measure;

	coord->K[0] = coord->P[0][0] / coord->S;
	coord->K[1] = coord->P[1][0] / coord->S;

	coord->y = newAngle - coord->angle;


	coord->angle += coord->K[0] * coord->y;
	coord->bias += coord->K[1] * coord->y;


	coord->P[0][0] -= coord->K[0] * coord->P[0][0];
	coord->P[0][1] -= coord->K[0] * coord->P[0][1];
	coord->P[1][0] -= coord->K[1] * coord->P[0][0];
	coord->P[1][1] -= coord->K[1] * coord->P[0][1];


	return coord->angle;
}

void setAngle(Kalman *coord, double newAngle)
{
	coord->angle = newAngle;

};



