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
#define INFLATION_WEIGHT 1
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
  inline unsigned char GetCost(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = std::abs(mx - src_x);
    unsigned int dy = std::abs(my - src_y);
    return ComputeCost(std::hypot(dx, dy));
  }
  void Inflation(std::vector<std::vector<unsigned char>>& return_costmap)
  {
    int limit = pLayered_->getSize() / pLayered_->getResolution();
    int grid_inscribed_dis = INSCRIBED_RADIUS / pLayered_->getResolution();
    limit++;
    grid_inscribed_dis++;
    bool seen[limit][limit];
    std::fill(seen[0], seen[0] + limit * limit, false);
    std::queue<Cell> q; /** @brief need inflation queue */
    std::vector<std::vector<bool>> local_map = pLayered_->getMap();
    for (int i = 0; i < limit; i++) {
      for (int j = 0; j < limit; j++) {
        if (local_map[i][j]) {
          return_costmap[i][j] = LETHAL_OBSTACLE;
          q.push(Cell(i, j, i, j));
          seen[i][j] = true;
        }
      }
    }
    while (!q.empty()) {
      const Cell& cell_data = q.front();
      q.pop();
      if (cell_data.mx > 0)
        q.push(Cell(cell_data.mx - 1, cell_data.my, cell_data.src_x, cell_data.src_y));
      if (cell_data.my > 0)
        q.push(Cell(cell_data.mx, cell_data.my - 1, cell_data.src_x, cell_data.src_y));
      if (cell_data.mx < limit - 1)
        q.push(Cell(cell_data.mx + 1, cell_data.my, cell_data.src_x, cell_data.src_y));
      if (cell_data.my < limit - 1)
        q.push(Cell(cell_data.mx, cell_data.my + 1, cell_data.src_x, cell_data.src_y));
      /** @brief choose big cost */
      return_costmap[cell_data.mx][cell_data.my] = std::max(return_costmap[cell_data.mx][cell_data.my], GetCost(cell_data.mx, cell_data.my, cell_data.src_x, cell_data.src_y));
    }
  }
};
}

#endif /* COSTMAP_H */
