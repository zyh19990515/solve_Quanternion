#include "Quanternion.h"

void Quanternion::Quanternion2n(quanternion qtr) {

	double Theta = 2.0 * acos(qtr.w);
	this->theta = Theta;
	Eigen::MatrixXd N = Eigen::MatrixXd::Zero(3, 1);
	this->n(0, 0) = qtr.x / sin(this->theta / 2);
	this->n(1, 0) = qtr.y / sin(this->theta / 2);
	this->n(2, 0) = qtr.z / sin(this->theta / 2);
}

void Quanternion::getR(quanternion qtr) {
	this->theta = 2.0 * acos(qtr.w);
	this->n(0, 0) = qtr.x / sin(this->theta / 2);
	this->n(1, 0) = qtr.y / sin(this->theta / 2);
	this->n(2, 0) = qtr.z / sin(this->theta / 2);
	Eigen::MatrixXd n_ = Eigen::MatrixXd::Zero(3, 3);
	n_(0, 1) = -n(2,0);
	n_(0, 2) = -n(1,0);
	n_(1, 0) = -n(2,0);
	n_(1, 2) = -n(0,0);
	n_(2, 0) = -n(1,0);
	n_(2, 1) = -n(0,0);
	this->R = cos(this->theta) * Eigen::MatrixXd::Identity(3, 3) + (1 - cos(this->theta)) * n * n.transpose() + sin(this->theta) * n_;
}

void Quanternion::getR(Eigen::MatrixXd n) {
	Eigen::MatrixXd n_ = Eigen::MatrixXd::Zero(3, 3);
	n_(0, 1) = -n(2, 0);
	n_(0, 2) = -n(1, 0);
	n_(1, 0) = -n(2, 0);
	n_(1, 2) = -n(0, 0);
	n_(2, 0) = -n(1, 0);
	n_(2, 1) = -n(0, 0);
	this->R = cos(this->theta) * Eigen::MatrixXd::Identity(3, 3) + (1 - cos(this->theta)) * n * n.transpose() + sin(this->theta) * n_;
}

quanternion Quanternion::n2Quanternion(Eigen::MatrixXd n, double theta) {
	quanternion qtr;
	qtr.w = cos(theta / 2.0);
	qtr.x = n(0, 0) * sin(theta / 2.0);
	qtr.y = n(1, 0) * sin(theta / 2.0);
	qtr.z = n(2, 0) * sin(theta / 2.0);
	return qtr;
}

RPY Quanternion::Quanternion2RPY(quanternion qtr) {
	RPY rpy;
	
	rpy.roll = (180.0 / pi) * atan2(2 * (qtr.w * qtr.x + qtr.y * qtr.z), 1 - 2 * (qtr.x * qtr.x + qtr.y * qtr.y));
	rpy.pitch = (180.0 / pi) * asin(2 * (qtr.w * qtr.y - qtr.z * qtr.x));
	rpy.yaw = (180.0 / pi) * atan2(2 * (qtr.w * qtr.z + qtr.x * qtr.y), 1 - 2 * (qtr.y * qtr.y + qtr.z * qtr.z));

	return rpy;
}

quanternion Quanternion::RPY2Quanternion(RPY rpy) {
	quanternion qtr;
	rpy.roll = rpy.roll * pi / 180;
	rpy.pitch = rpy.pitch * pi / 180;
	rpy.yaw = rpy.yaw * pi / 180;
	qtr.w = cos(rpy.roll / 2) * cos(rpy.pitch / 2) * cos(rpy.yaw / 2) - sin(rpy.roll / 2) * sin(rpy.pitch / 2) * sin(rpy.yaw / 2);
	qtr.x = sin(rpy.roll / 2) * cos(rpy.pitch / 2) * cos(rpy.yaw / 2) + cos(rpy.roll / 2) * sin(rpy.pitch / 2) * sin(rpy.yaw / 2);
	qtr.y = -sin(rpy.roll / 2) * cos(rpy.pitch / 2) * sin(rpy.yaw / 2) + cos(rpy.roll / 2) * sin(rpy.pitch / 2) * cos(rpy.yaw / 2);
	qtr.z = cos(rpy.roll / 2) * cos(rpy.pitch / 2) * sin(rpy.yaw / 2) + sin(rpy.roll / 2) * sin(rpy.pitch / 2) * cos(rpy.yaw / 2);
	return qtr;
}