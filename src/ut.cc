#include "ut.h"
#include "opencv2/core/persistence.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>
#include <glog/logging.h>
Ut::Ut(std::string config_file_path)
    : costmap_2d::Layer("UT")
{
  LOG(INFO) << "Initial UT";
  LOG(INFO) << "Start read config";
  cv::FileStorage fs(config_file_path, cv::FileStorage::READ);
  fs["field_of_view"] >> max_angle_;
  max_angle_ /= 2;
  fs["max_dis"] >> phi_v_;
  fs.release();
  LOG(INFO) << "Set UT config ok";
}
double Ut::Gamma(double theta)
{
  if (fabs(theta) > max_angle_)
    return 0.0;
  else
    return 1 - pow(theta / max_angle_, 2);
}

double Ut::Delta(double phi)
{
  return 1 - (1 + tanh(2 * (phi - phi_v_))) / 2;
}

void Ut::GetDeltas(double angle, double* dx, double* dy)
{
  double ta = tan(angle);
  if (ta == 0)
    *dx = 0;
  else
    *dx = this->getResolution() / ta;

  *dx = copysign(*dx, cos(angle));
  *dy = copysign(this->getResolution(), sin(angle));
}

double Ut::SensorModel(double r, double phi, double theta)
{
  double lbda = Delta(phi) * Gamma(theta);

  double delta = this->getResolution();

  if (phi >= 0.0 && phi < r - 2 * delta * r)
    return (1 - lbda) * (0.5);
  else if (phi < r - delta * r)
    return lbda * 0.5 * pow((phi - (r - 2 * delta * r)) / (delta * r), 2) + (1 - lbda) * .5;
  else if (phi < r + delta * r) {
    double J = (r - phi) / (delta * r);
    return lbda * ((1 - (0.5) * pow(J, 2)) - 0.5) + 0.5;
  } else
    return 0.5;
}
void Ut::getSensorInWorld(double robot_x, double robot_y, double robot_yaw, double& sensor_x, double& sensor_y, double r, double theta)
{
  double my_sensor_x = 1.0 * cos(theta * PI / 180) * r;
  double my_sensor_y = 1.0 * sin(theta * PI / 180) * r;
  sensor_x = TransPosToWorld(robot_x, robot_y, robot_yaw, my_sensor_x, my_sensor_y).first;
  sensor_y = TransPosToWorld(robot_x, robot_y, robot_yaw, my_sensor_x, my_sensor_y).second;
  return;
}
void Ut::FeedData(std::vector<UtSensor> data)
{
  for (int i = 0; i < data.size(); i++) {
    LOG(INFO) << "Update" << i << ":angle " << data[i].theta;
  }
}
