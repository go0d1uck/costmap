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
#define INSCRIBED_RADIUS_OBSTACLE 254
namespace costmap_2d {

class Costmap {
  private:
  std::map<std::string, std::shared_ptr<Layer>> plugins_;
  /** @brief the layer after layered */
  std::shared_ptr<Layer> pLayered_;
  std::vector<std::vector<unsigned char>> costmap_need_;
  /** @brief useful for inflation */
  double inscribed_radius_;
  double inflation_weight_;
  double inflation_radius_;
  struct Cell {
    int mx, my, src_x, src_y;
    int dis;
    Cell(int x, int y, int tx, int ty, int d)
        : mx(x)
        , my(y)
        , src_x(tx)
        , src_y(ty)
        , dis(d)
    {
    }
    bool operator<(const Cell a) const
    {
      return dis > a.dis;
    }
  };
  void EnQueue(int x, int y, int src_x, int src_y, std::vector<std::vector<bool>>& seen, std::priority_queue<Cell>&, std::vector<std::vector<unsigned char>>& cost_map);

  public:
  Costmap(double robot_x, double robot_y, double robot_yaw, double size, std::string file_name);

  ~Costmap();
  void UpdateCostMap(double robot_x, double robot_y, double robot_yaw);
  unsigned char GetCellCost(double x, double y);
  bool AddPlug(std::vector<std::vector<bool>>& gridMap, std::string name, double robot_x, double robot_y, double robot_yaw, double size);
  std::vector<std::vector<bool>> GetLayeredMap(double new_robot_x, double new_robot_y);
  inline unsigned char ComputeCost(double distance) const
  {
    distance *= pLayered_->getResolution();
    unsigned char cost = 0;
    if (distance == 0) {
      cost = LETHAL_OBSTACLE;
    } else if (distance <= inscribed_radius_) {
      cost = INSCRIBED_RADIUS_OBSTACLE;
    } else {
      double factor = std::exp(-1.0 * inflation_radius_ * (distance - inscribed_radius_));
      cost = (unsigned char)((INSCRIBED_RADIUS_OBSTACLE - 1) * factor);
    }
    return cost;
  }
  inline unsigned char GetCost(Cell t)
  {
    /** @brief out of inflation radius is zero*/
    int grid_dis = GetDistanceIngrid(t);
    if (grid_dis * pLayered_->getResolution() > inflation_radius_)
      return 0;
    else
      return ComputeCost(grid_dis);
  }
  inline unsigned int GetDistanceIngrid(Cell& t)
  {
    unsigned int dx = std::abs(t.mx - t.src_x);
    unsigned int dy = std::abs(t.my - t.src_y);
    t.dis = hypot(dx, dy);
    return t.dis;
  }
  void Inflation(std::vector<std::vector<unsigned char>>& return_costmap);
};
}

#endif /* COSTMAP_H */
