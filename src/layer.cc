#include <glog/logging.h>

#include <cmath>
#include <ostream>
#include <queue>
#include <utility>
#include <vector>

#include "layer.h"
namespace costmap_2d {
void Layer::Update(const std::vector<std::vector<bool>>& grid_map,
                   double robot_x, double robot_y, double robot_yaw) {
  grid_map_ = grid_map;
  robot_yaw_ = robot_yaw;
  origin_x_ = robot_x + (this->getSize().first * this->getResolution()) / 2;
  origin_y_ = robot_y + (this->getSize().second * this->getResolution()) / 2;
}
void Layer::ResetGridMap() {
  for (auto it = grid_map_.begin(); it != grid_map_.end(); it++) {
    fill(it->begin(), it->end(), false);
  }
}
void Layer::UpdateOrigin(double robot_x, double robot_y,
                         std::vector<std::vector<bool>>& new_grid_map,
                         bool fill_value) {
  LOG(INFO) << name_ << " is updating origin...";
  double new_x = robot_x + (this->getSize().first * this->getResolution()) / 2;
  double new_y = robot_y + (this->getSize().second * this->getResolution()) / 2;
  /** @brief calculate  offset*/
  int offset_x = (origin_x_ - new_x) / resolution_;
  int offset_y = (origin_y_ - new_y) / resolution_;
  /** @brief calculate updated map */
  std::vector<std::vector<bool>> local_map = grid_map_;
  this->ResetGridMap();
  /** @brief translate coordinate */
  int end_x = this->getSize().first;
  int end_y = this->getSize().second;
  for(int i = 0;i < grid_map_.size();i++)
  {
    for(int j = 0;j < grid_map_[i].size();j++)
    {
      grid_map_[i][j] = fill_value;
    }
  }
  for (int i = 0; i < end_x; i++) {
    for (int j = 0; j < end_y; j++) {
      int destination_x = i - offset_x;
      int destination_y = j - offset_y;
      /** @brief check coordinate is legal */
      if ((destination_x >= 0 && destination_x < end_x) &&
          (destination_y >= 0 && destination_y < end_y)) {
        grid_map_[destination_x][destination_y] = local_map[i][j];
      }
    }
  }
  /* TODO:读取地图会有一条线，目前不确定是读取地图引起的  <03-06-21, go0d1uck> */
  if (fill_value) {
    if (new_grid_map.size() > grid_map_.size()) {
      int cnt = (new_grid_map.size() - grid_map_.size()) / 2;
      for (int i = 0; i < cnt; i++) {
        grid_map_.push_back(
            std::vector<bool>(grid_map_[0].size(), fill_value));
        grid_map_.insert(grid_map_.begin(),
                         std::vector<bool>(grid_map_[0].size(), fill_value));
      }
    }
    if (new_grid_map.size() != 0 && new_grid_map[0].size() > grid_map_[0].size()) {
      for (int j = 0; j < new_grid_map.size(); j++) {
        int cnt = (new_grid_map[j].size() - grid_map_[j].size())/2;
        for (int i = 0; i < cnt; i++) {
          grid_map_[j].push_back(fill_value);
          grid_map_[j].insert(grid_map_[j].begin(), fill_value);
        }
      }
    }
  }
  new_grid_map = grid_map_;
  grid_map_ = local_map;
  LOG(INFO) << name_ << " origin is updated";
}
}  // namespace costmap_2d
