#ifndef COSTMAP_H
#define COSTMAP_H
#include "layer.h"
#include "ut.h"
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
#define TEST
namespace costmap_2d {

class Costmap : private Layer {
  private:
  std::map<std::string, std::shared_ptr<Layer>> plugins_;
  std::vector<std::vector<unsigned char>> costmap_need_;
  /** @brief useful for inflation */
  double inscribed_radius_;
  double inflation_weight_;
  double inflation_radius_;
  double map_size_;
  std::mutex plug_map_mutex_;
  std::mutex layered_mutex_;
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
  void getGloblPos(const int& x, const int& y, double& globl_x, double& glob_y)
  {
    std::pair<double, double> origin;
    origin.first = getOriginX(), origin.second = getOriginY();
    globl_x = origin.first - x * getResolution(), glob_y = origin.second - y * getResolution();
  }
  std::vector<std::vector<bool>> GetLayeredMap(double new_robot_x, double new_robot_y, double new_robot_yaw);
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
  //unsigned char GetCellCost(double x, double y);
  unsigned char GetCellCost(double x, double y, int& cx, int& cy);

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
    int obstacle_x = -t.mx + map_size_ / this->getResolution() / 2;
    int obstacle_y = -t.my + map_size_ / this->getResolution() / 2;
    double obstacle_angle_in_robot = std::atan2(obstacle_y, obstacle_x);
    double result = fmod(robot_yaw_ - obstacle_angle_in_robot + M_PI, 2.0 * M_PI);
    result = (result <= 0) ? (result + M_PI) : (result - M_PI);
    bool right_toward_in_robot = result > 0;
    if (dis_mile > inflation_radius_)
      return 0;
    else {
      unsigned char t = ComputeCost(right_toward_in_robot ? (dis_mile * 1.3) : dis_mile);
      if (t < 10)
        return 0;
      else
        return t;
    }
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
