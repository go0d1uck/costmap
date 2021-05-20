#include "laser.h"
#include "opencv2/core/persistence.hpp"
#include "opencv2/opencv.hpp"
#include <string>

Laser::Laser(const std::string& config_file_path)
    : costmap_2d::Layer("Laser")
{
  LOG(INFO) << "Initial Laser";
  LOG(INFO) << "Start read config";
  cv::FileStorage fs(config_file_path, cv::FileStorage::READ);
  fs["map_size_metre"] >> map_size_;
  fs["sensor_x"] >> sensor_x_;
  fs["sensor_y"] >> sensor_y_;
  fs["sensor_toward_angle"] >> sensor_toward_angle_;
  fs["max_dis"] >> max_dis_;
  fs.release();
  LOG(INFO) << "Set Laser config ok";
}
std::vector<std::vector<bool>> Laser::getGridMap(double x, double y)
{
  LOG(INFO) << "start get laser map";
  laser_map_mutex_.lock();
  std::vector<std::vector<bool>> local_map(map_size_ / getResolution(), std::vector<bool>(map_size_ / getResolution(), false));
  std::pair<int, int> origin;
  ToGridMapPos(x + map_size_ / 2, y + map_size_ / 2, origin.first, origin.second);
  for (int i = 0; i < local_map.size(); i++) {
    for (int j = 0; j < local_map[i].size(); j++) {
      int globl_x = origin.first - i, globl_y = origin.second - j;
      local_map[i][j] = (save_map_.find({ globl_x, globl_y }) == save_map_.end()) ? 0 : 1;
    }
  }
  laser_map_mutex_.unlock();
  LOG(INFO) << "get laser map ok";
  return local_map;
}
void Laser::FeedDate(std::vector<float> ranges, std::vector<float> angles, float robot_x, float robot_y, float robot_yaw)
{
  LOG(INFO) << "START LARSER COMPUTE";
  laser_map_mutex_.lock();
  save_map_.clear();
  for (int i = 0; i < ranges.size(); i++) {
    if (ranges[i] == 0 || ranges[i] > max_dis_)
      continue; //ignored dis
    double target_x_local = ranges[i] * cos(angles[i]);
    double target_y_local = ranges[i] * sin(angles[i]);
    double tx, ty;
    LocalToGlobl(target_x_local, target_y_local, tx, ty, robot_x, robot_y, robot_yaw);
    int Tx, Ty;
    ToGridMapPos(tx, ty, Tx, Ty);
    if ((robot_x - tx) * (robot_x - tx) + (robot_y - ty) * (robot_y - ty) < 0.3 * 0.3)
      continue;
    save_map_[std::make_pair(Tx, Ty)] = true;
  }
  laser_map_mutex_.unlock();
  LOG(INFO) << "FINISH LARSER COMPUTE";
}
