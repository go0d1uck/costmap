#ifndef COSTMAP_H
#define COSTMAP_H
#include "layer.h"
#include <cmath>
#include <glog/logging.h>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#define LETHAL_OBSTACLE 255
#define INSCRIBED_RADIUS 1.0
#define INSCRIBED_RADIUS_OBSTACLE 254
#define INFLATION_WEIGHT 1
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
  inline unsigned char ComputeCost(double distance) const
  {
    unsigned char cost = 0;
    if (distance == 0) {
      cost = LETHAL_OBSTACLE;
    } else if (distance * pLayered_->getResolution() <= INSCRIBED_RADIUS) {
      cost = INSCRIBED_RADIUS_OBSTACLE;
    } else {
      double euclidean_distance = distance * pLayered_->getResolution();
      double factor = std::exp(-1.0 * INFLATION_WEIGHT * (euclidean_distance - INSCRIBED_RADIUS));
      cost = (unsigned char)((INSCRIBED_RADIUS_OBSTACLE - 1) * factor);
    }
    return cost;
  }
  bool Inflation();
};
}

#endif /* COSTMAP_H */
