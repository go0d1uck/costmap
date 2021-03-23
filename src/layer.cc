#include "layer.h"
#include <glog/logging.h>
#include <ostream>
#include <vector>
namespace costmap_2d {
Layer::Layer(std::string name, double robot_x, double robot_y, double robot_yaw, double size)
    : name_(name)
    , size_(size)
    , resolution_(RESOLUTION)
{
    origin_x_ = robot_x - size / 2;
    origin_y_ = robot_y - size / 2;
    /** @brief resolution_ need read from config.yaml */
    int limit_grid = size / resolution_;
    limit_grid++;
    grid_map_ = std::vector<std::vector<bool>>(limit_grid, std::vector<bool>(limit_grid, false));
}
void Layer::Update(std::vector<std::vector<bool>>& grid_map, double robot_x, double robot_y)
{
    grid_map_ = grid_map;
    origin_x_ = robot_x - size_ / 2;
    origin_y_ = robot_y - size_ / 2;
}
void Layer::ResetGridMap()
{
    for (auto it = grid_map_.begin(); it != grid_map_.end(); it++) {
        fill(it->begin(), it->end(), false);
    }
}
void Layer::UpdateOrigin(double new_x, double new_y)
{
    LOG(INFO) << name_ << " is updating origin...";
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
    int end_x = size_ / resolution_;
    int end_y = size_ / resolution_;
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
