#ifndef COSTMAP_H
#define COSTMAP_H
#include "layer.h"
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
namespace costmap_2d {

class Costmap {
  private:
  std::map<std::string, std::shared_ptr<Layer>> plugins_;
  /** @brief the layer after layered */
  std::shared_ptr<Layer> pLayered_;

  public:
  Costmap(double robot_x, double robot_y, double robot_yaw, double size);
  ~Costmap();
  std::vector<std::vector<unsigned char>> GetCostMap(double robot_x, double robot_y, double robot_yaw);
  bool AddPlug(std::vector<std::vector<bool>>& gridMap, std::string name, double robot_x, double robot_y, double robot_yaw, double size);
  std::vector<std::vector<bool>> GetLayeredMap(double new_origin_x, double new_origin_y);
};
}

#endif /* COSTMAP_H */
