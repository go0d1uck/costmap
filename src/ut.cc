#include "ut.h"
#include "opencv2/core/persistence.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>
#include <glog/logging.h>
#include <utility>
#include <vector>
Ut::Ut(std::string config_file_path)
    : costmap_2d::Layer("UT")
{
  LOG(INFO) << "Initial UT";
  LOG(INFO) << "Start read config";
  cv::FileStorage fs(config_file_path, cv::FileStorage::READ);
  fs["map_size_metre"] >> map_size_;
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

  double delta = 0.02;

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
void Ut::FeedData(std::vector<UtSensor> data, double robot_x, double robot_y, double robot_yaw)
{
  for (int i = 0; i < data.size(); i++) {
    LOG(INFO) << "Update" << i << ":angle " << data[i].theta;
    double max_angle_ = data[i].fov / 2;
    // calculate the bound
    double min_x, min_y, max_x, max_y;
    min_x = min_y = max_x = max_y = 0;
    double tx = cos(data[i].toward_angle - max_angle_) * data[i].detect_dis * 1.2;
    double ty = sin(data[i].toward_angle - max_angle_) * data[i].detect_dis * 1.2;
    min_x = std::min(tx, min_x);
    min_y = std::min(ty, min_y);
    max_x = std::max(tx, max_x);
    max_y = std::max(ty, max_y);
    tx = cos(data[i].toward_angle + max_angle_) * data[i].detect_dis * 1.2;
    ty = sin(data[i].toward_angle + max_angle_) * data[i].detect_dis * 1.2;
    min_x = std::min(tx, min_x);
    min_y = std::min(ty, min_y);
    max_x = std::max(tx, max_x);
    max_y = std::max(ty, max_y);
    int grid_min_x, grid_min_y, grid_max_x, grid_max_y;
    grid_min_x = int(grid_min_x / getResolution());
    grid_min_y = int(grid_min_y / getResolution());
    grid_max_x = int(grid_max_x / getResolution());
    grid_max_y = int(grid_max_y / getResolution());
    for (int x = grid_min_x; x <= grid_max_x; x++) {
      for (int y = grid_min_y; y <= grid_max_y; y++) {
        double phi = sqrt(x * x + y * y); // grid
        double deviation_angle = atan2(x, y);
        double theta = deviation_angle - data[i].toward_angle;
        double sensor_x_in_robot = data[i].r * cos(data[i].theta);
        double sensor_y_in_robot = data[i].r * sin(data[i].theta);
        double obstacle_x_in_sensor = phi * getResolution() * cos(deviation_angle);
        double obstacle_y_in_sensor = phi * getResolution() * sin(deviation_angle);
        double obstacle_x_in_robot = trans_coordinate(obstacle_x_in_sensor, obstacle_y_in_sensor, deviation_angle, sensor_x_in_robot, sensor_y_in_robot).first;
        double obstacle_y_in_robot = trans_coordinate(obstacle_x_in_sensor, obstacle_y_in_sensor, deviation_angle, sensor_x_in_robot, sensor_y_in_robot).second;
        double world_x = trans_coordinate(obstacle_x_in_robot, obstacle_y_in_robot, robot_yaw, robot_x, robot_y).first;
        double world_y = trans_coordinate(obstacle_x_in_robot, obstacle_y_in_robot, robot_yaw, robot_x, robot_y).second;
        double sensor = SensorModel(data[i].detect_dis, phi * getResolution(), theta);
        double prior = getPreProb(world_x, world_y);
        double prob_occ = sensor * prior;
        double prob_not = (1 - sensor) * (1 - prior);
        probability_map_[std::make_pair(world_x, world_y)] = prob_occ / (prob_occ + prob_not);
      }
    }
  }
}
std::vector<std::vector<bool>> Ut::getGridMap(double x, double y)
{
  double min_x = x - map_size_ / 2, min_y = y - map_size_ / 2;
  double max_x = x + map_size_ / 2, max_y = y + map_size_ / 2;
  std::pair<int, int> min_pos = ToGridMapPos(min_x, min_y), max_pos = ToGridMapPos(max_x, max_y);
  std::vector<std::vector<bool>> local_map;
  local_map.resize(max_pos.second - min_pos.second);
  for (int i = min_pos.second; i < max_pos.second; i++) {
    for (int j = min_pos.first; j < max_pos.first; j++) {
      local_map[i - min_pos.second].push_back(getPreProb(j, i) <= PROB_THRESHOLD ? false : true);
    }
  }
  return local_map;
}
