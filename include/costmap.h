#ifndef COSTMAP_H
#define COSTMAP_H
#include "layer.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <glog/logging.h>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#define LETHAL_OBSTACLE 255
#define INSCRIBED_RADIUS_OBSTACLE 254
namespace costmap_2d {

class Costmap : private Layer {
  private:
  std::map<std::string, std::shared_ptr<Layer>> plugins_;
  std::vector<std::vector<unsigned char>> costmap_need_;
  /** @brief useful for inflation */
  double inscribed_radius_;
  double inflation_weight_;
  double inflation_radius_;
  std::mutex plug_map_mutex_;
  /** @brief useful for update queue */
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
  Costmap(double robot_x, double robot_y, double robot_yaw, std::string file_name);
  ~Costmap();
  Costmap(const Costmap&);
  Costmap& operator=(const Costmap&);
#ifdef TEST
  bool TestMap(const std::vector<std::vector<bool>>& m);
#endif
  public:
  std::vector<std::vector<bool>> GetLayeredMap(double new_robot_x, double new_robot_y);
  static Costmap& getInstance(std::string file_name)
  {
    static Costmap instance_ = Costmap(0.0, 0.0, 0.0, file_name);
    return instance_;
  }
  static Costmap& getInstance()
  {
    return getInstance("");
  }
  std::vector<std::vector<unsigned char>> UpdateCostMap(double robot_x, double robot_y, double robot_yaw);
  bool AddPlug(std::vector<std::vector<bool>>& gridMap, std::string name, double robot_x, double robot_y, double robot_yaw);
  unsigned char GetCellCost(double x, double y);

  protected:
  inline unsigned char ComputeCost(double distance) const
  {
    unsigned char cost = 0;
    if (distance == 0) {
      cost = LETHAL_OBSTACLE;
    } else if (distance < inscribed_radius_) {
      cost = INSCRIBED_RADIUS_OBSTACLE;
    } else {
      double factor = std::exp(-1.0 * inflation_weight_ * (distance - inscribed_radius_));
      cost = (unsigned char)((INSCRIBED_RADIUS_OBSTACLE - 1) * factor);
    }
    return cost;
  }
  inline unsigned char GetCost(Cell t)
  {
    /** @brief out of inflation radius is zero*/
    double dis_mile = GetDistanceInGrid(t) * this->getResolution();
    if (dis_mile > inflation_radius_)
      return 0;
    else
      return ComputeCost(dis_mile);
  }
  inline unsigned int GetDistanceInGrid(Cell& t)
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
