#include "layer.h"
#include <cmath>
#include <glog/logging.h>
#include <ostream>
#include <utility>
#include <vector>
#include <queue>
namespace costmap_2d {
void Layer::Update(std::vector<std::vector<bool>>& grid_map, double robot_x, double robot_y)
{
    grid_map_ = grid_map;
    origin_x_ = robot_x - (this->getSize().first * this->getResolution()) / 2;
    origin_y_ = robot_y - (this->getSize().second * this->getResolution()) / 2;
}
void Layer::ResetGridMap()
{
    for (auto it = grid_map_.begin(); it != grid_map_.end(); it++) {
        fill(it->begin(), it->end(), false);
    }
}
void Layer::UpdateOrigin(double robot_x, double robot_y) 
{
    LOG(INFO) << name_ << " is updating origin...";
    double new_x = robot_x - (this->getSize().first * this->getResolution()) / 2;
    double new_y = robot_y - (this->getSize().second * this->getResolution()) / 2;
    /** @brief calculate  offset*/
    int offset_x = (origin_x_ - new_x) / resolution_;
    int offset_y = (origin_y_ - new_y) / resolution_;
    /** @brief calculate updated map */
    std::vector<std::vector<bool>> local_map = grid_map_;
    this->ResetGridMap();
    /** @brief update origin */
    origin_x_ = new_x;
    origin_y_ = new_y;
    /** @brief translate coordinate */
    int start_x = 0;
    int start_y = 0;
    int end_x = this->getSize().first;
    int end_y = this->getSize().second;
    for (int i = start_x; i < end_x; i++) {
        for (int j = start_y; j < end_y; j++) {
            int destination_x = i - offset_x;
            int destination_y = j - offset_y;
            /** @brief coordinate is legal */
            if ((destination_x >= 0 && destination_x < end_x) && (destination_y >= 0 && destination_y < end_y)) {
                grid_map_[destination_x][destination_y] = local_map[i][j];
            }
        }
    }
    LOG(INFO) << name_ << " origin is updated";
}
}
