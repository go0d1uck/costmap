#ifndef COSTMAP_H
#define COSTMAP_H
#include "layer.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <glog/logging.h>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#define LETHAL_OBSTACLE 255
#define INSCRIBED_RADIUS 1.0
#define INSCRIBED_RADIUS_OBSTACLE 254
#define INFLATION_WEIGHT 10
#define INFLATION_RADIUS 0.5
namespace costmap_2d {

class Costmap {
  private:
  std::map<std::string, std::shared_ptr<Layer>> plugins_;
  /** @brief the layer after layered */
  std::shared_ptr<Layer> pLayered_;

  /** @brief useful for inflation */
  struct Cell {
    int mx, my, src_x, src_y;
    Cell(int x, int y, int tx, int ty)
        : mx(x)
        , my(y)
        , src_x(tx)
        , src_y(ty)
    {
    }
  };

  public:
  Costmap(double robot_x, double robot_y, double robot_yaw, double size);
  ~Costmap();
  std::vector<std::vector<unsigned char>> GetCostMap(double robot_x, double robot_y, double robot_yaw);
  bool AddPlug(std::vector<std::vector<bool>>& gridMap, std::string name, double robot_x, double robot_y, double robot_yaw, double size);
  std::vector<std::vector<bool>> GetLayeredMap(double new_robot_x, double new_robot_y);
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
  inline unsigned char GetCost(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = std::abs(mx - src_x);
    unsigned int dy = std::abs(my - src_y);
    if(hypot(dx,dy)/100.0 > INFLATION_RADIUS) return 0;
    else return ComputeCost(std::hypot(dx, dy));
  }
  void Inflation(std::vector<std::vector<unsigned char>>& return_costmap);
};
}

#endif /* COSTMAP_H */
