#include "layer.h"
namespace costmap {
Layer::Layer(std::vector<std::vector<bool>> gridMap, std::string name, double robot_x, double robot_y, double robot_yaw, double size)
    : gridMap_(gridMap)
    , name_(name)
    , size_(size)
{
    origin_x_ = robot_x - size / 2;
    origin_y_ = robot_y - size / 2;
}
void Layer::Update(std::vector<std::vector<bool>>& gridMap, double robot_x, double robot_y)
{
    gridMap_ = gridMap;
    origin_x_ = robot_x - size_ / 2;
    origin_y_ = robot_y - size_ / 2;
}
}
