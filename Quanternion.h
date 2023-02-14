#pragma once
#ifndef Quanternion_H
#define Quanternion_H
#include<iostream>
#include<Eigen\Core>
#include<vector>
#include<Eigen\Dense>
#include<math.h>
#define pi 3.1415926
struct quanternion
	{
	double w = 0.0;
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	};
struct RPY
{
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
};
class Quanternion
{
public:
	
	Eigen::MatrixXd n = Eigen::MatrixXd::Zero(3,1);//旋转向量（旋转轴）
	double theta;
	double theta_degree;
	Eigen::MatrixXd R;
	void Quanternion2n(quanternion qtr);
	void getR(quanternion qtr);
	void getR(Eigen::MatrixXd n);
	quanternion n2Quanternion(Eigen::MatrixXd n, double theta);
	RPY Quanternion2RPY(quanternion qtr);
	quanternion RPY2Quanternion(RPY rpy);
};

#endif // !Quanternion_H