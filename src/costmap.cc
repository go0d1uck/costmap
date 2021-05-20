#include "costmap.h"
#include "layer.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <glog/logging.h>
#include <new>
#include <opencv2/core/persistence.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#define VERSION 3.0
namespace costmap_2d {
#ifdef TEST
bool Costmap::TestMap(const std::vector<std::vector<bool>>& m)
{
  int center_i = m.size() / 2;
  int center_j = m[0].size() / 2;
  std::map<std::pair<int, int>, bool> seen;
  std::queue<std::pair<int, int>> q;
  if (m[center_i][center_j] == 1)
    return false;
  q.push(std::make_pair(center_i, center_j));
  while (!q.empty()) {
    std::pair<int, int> t = q.front();
    q.pop();
    if (m[t.first][t.second] == 1) {
      LOG(INFO) << "collision position" << t.first << "," << t.second << "--dis:" << std::hypot(center_i - t.first, center_j - t.second);
      return false;
    } else {
      double dis = std::hypot(center_i - t.first - 1, center_j - t.second);
      if (!seen[std::make_pair(t.first + 1, t.second)] && dis <= inscribed_radius_ * 20) {
        q.push(std::make_pair(t.first + 1, t.second));
        seen[std::make_pair(t.first + 1, t.second)] = true;
      }
      dis = std::hypot(center_i - t.first + 1, center_j - t.second);
      if (!seen[std::make_pair(t.first - 1, t.second)] && dis <= inscribed_radius_ * 20) {
        q.push(std::make_pair(t.first - 1, t.second));
        seen[std::make_pair(t.first - 1, t.second)] = true;
      }
      dis = std::hypot(center_i - t.first, center_j - t.second - 1);
      if (!seen[std::make_pair(t.first, t.second + 1)] && dis <= inscribed_radius_ * 20) {
        q.push(std::make_pair(t.first, t.second + 1));
        seen[std::make_pair(t.first, t.second + 1)] = true;
      }
      dis = std::hypot(center_i - t.first, center_j - t.second + 1);
      if (!seen[std::make_pair(t.first, t.second - 1)] && dis <= inscribed_radius_ * 20) {
        q.push(std::make_pair(t.first, t.second - 1));
        seen[std::make_pair(t.first + 1, t.second)] = true;
      }
    }
  }
  return true;
}
#endif
Costmap::Costmap(double robot_x, double robot_y, double robot_yaw, std::string file_name)
    : Layer("costmap")
{
  LOG(INFO) << "Start initing CostMap"
            << "---Version:" << VERSION;
  LOG(INFO) << "Read config";
  LOG(INFO) << "Config Path"
            << " " << file_name;
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  try {
    fs["inflation_weight"] >> inflation_weight_;
    fs["inflation_radius"] >> inflation_radius_;
    fs["inscribed_radius"] >> inscribed_radius_;
    fs["map_size"] >> map_size_;
    LOG(INFO) << "Set config ok";
  } catch (std::exception) {
    LOG(ERROR) << "Read config error!";
    throw;
  }
  try {
    std::vector<std::vector<bool>> local_map(map_size_ / RESOLUTION, std::vector<bool>(map_size_ / RESOLUTION, false));
    this->setGridMap(local_map);
  } catch (std::bad_alloc) {
    LOG(INFO) << "costmap molloc error!";
  }
  fs.release();
  /** @brief init layered grid_map */
  this->ResetGridMap();
  LOG(INFO) << "initing CostMap...OK";
}
Costmap::~Costmap()
{
  /** @brief free memory */
  if (plugins_.size() > 0) {
    plugins_.clear();
    LOG(INFO) << "Plugin was cleared";
  }
}
bool Costmap::AddPlug(std::vector<std::vector<bool>>& gridMap, std::string name, double robot_x, double robot_y, double robot_yaw)
{
  LOG(INFO) << "Adding plugin---" << name;
  if (plugins_.find(name) != plugins_.end()) {
    LOG(INFO) << "Plugin is existed--just update";
  } else {
    LOG(INFO) << "Creat new layer---" << name;
    plug_map_mutex_.lock();
    plugins_[name] = std::shared_ptr<Layer>(new Layer(name));
    plug_map_mutex_.unlock();
  }
  layered_mutex_.lock();
  plugins_[name]->Update(gridMap, robot_x, robot_y,robot_yaw);
  layered_mutex_.unlock();
  LOG(INFO) << "Plugin---" << name << "---finished";
  return true;
}
std::vector<std::vector<bool>> Costmap::GetLayeredMap(double robot_x, double robot_y,double robot_yaw)
{
  /** @brief if no map */
  LOG(INFO) << "Rending layered map...";
  if (plugins_.size() == 0) {
    LOG(INFO) << "No plugin...return self";
    return this->getMap();
  }
  /** @brief start layered */
  this->ResetGridMap();
  this->Update(this->getMap(), robot_x, robot_y,robot_yaw);
  std::vector<std::vector<bool>> local_map = this->getMap();
  for (auto it = plugins_.begin(); it != plugins_.end(); it++) {
    std::vector<std::vector<bool>> tmp_grid_map;
    it->second->UpdateOrigin(robot_x, robot_y, tmp_grid_map);
#ifdef TEST
    LOG(INFO) << "test map";
    if (tmp_grid_map.size() > 0) {
      if (!TestMap(tmp_grid_map)) {
        LOG(ERROR) << "ERROR MAP---" << it->first;
        for (int i = 0; i < tmp_grid_map.size(); i++) {
          std::string one_line = "";
          for (int j = 0; j < tmp_grid_map[i].size(); j++)
            one_line += std::to_string(tmp_grid_map[i][j]);
          LOG(INFO) << one_line;
        }
      }
    }
#endif
    int offset = (it->second->getSize().first - this->getSize().first) / 2;
    if (offset > 0) /** @brief the sensor map is bigger than cost map */
    {

      for (int i = 0; i < local_map.size() && i < tmp_grid_map.size(); i++) {
        for (int j = 0; j < local_map[i].size() && j < tmp_grid_map[i].size(); j++) {
          local_map[i][j] = local_map[i][j] | tmp_grid_map[i + offset][j + offset]; /** @brief if one is obstacle,the map obstacle */
        }
      }
    } else {
      offset = abs(offset);
      for (int i = 0; i < local_map.size() && i < tmp_grid_map.size(); i++) {
        for (int j = 0; j < local_map[i].size() && j < tmp_grid_map[i].size(); j++) {
          local_map[i + offset][j + offset] = local_map[i + offset][j + offset] | tmp_grid_map[i][j]; /** @brief if one is obstacle,the map obstacle */
        }
      }
    }
  }
  this->setGridMap(local_map);
  LOG(INFO) << "Rending finished";
  return local_map;
}

void Costmap::Inflation(std::vector<std::vector<unsigned char>>& return_costmap)
{
  LOG(INFO) << "Start inflation";
  int limit_x = this->getSize().first;
  int limit_y = this->getSize().second;
  if (limit_x < 0 || limit_y < 0 || limit_x > 1000 || limit_y > 1000) {
    LOG(INFO) << limit_x << " " << limit_y << " ERROR";
    return;
  }
  std::vector<std::vector<bool>> seen(limit_x, std::vector<bool>(limit_y, false));
  return_costmap = std::vector<std::vector<unsigned char>>(limit_x, std::vector<unsigned char>(limit_y, 0));
  std::priority_queue<Cell> que;
  std::vector<std::vector<bool>> local_map = this->getMap();
  LOG(INFO) << "Inflation initialized";
  /** @brief convert occupied to cosmap obstacle */
  for (int i = 0; i < limit_x; i++) {
    for (int j = 0; j < limit_y; j++) {
      if (local_map[i][j]) {
        return_costmap[i][j] = LETHAL_OBSTACLE;
        que.push(Cell(i, j, i, j, 0)); /** @brief push into queue */
        seen[i][j] = true;
      }
    }
  }
  while (!que.empty()) {
    const Cell t = que.top();
    if (t.dis != 0)
      return_costmap[t.mx][t.my] = GetCost(t);
    que.pop();
    if (t.mx > 0)
      EnQueue(t.mx - 1, t.my, t.src_x, t.src_y, seen, que, return_costmap);
    if (t.my > 0)
      EnQueue(t.mx, t.my - 1, t.src_x, t.src_y, seen, que, return_costmap);
    if (t.mx < limit_x - 1)
      EnQueue(t.mx + 1, t.my, t.src_x, t.src_y, seen, que, return_costmap);
    if (t.my < limit_y - 1)
      EnQueue(t.mx, t.my + 1, t.src_x, t.src_y, seen, que, return_costmap);
  }
  LOG(INFO) << "Inflation ok";
}
std::vector<std::vector<unsigned char>> Costmap::UpdateCostMap(double robot_x, double robot_y, double robot_yaw)
{
  std::vector<std::vector<unsigned char>> local_map;
  layered_mutex_.lock();
  GetLayeredMap(robot_x, robot_y,robot_yaw);
  layered_mutex_.unlock();
  Inflation(local_map);
  costmap_need_ = local_map;
#ifdef TEST
  if (local_map[local_map.size() / 2][local_map[0].size() / 2] >= INSCRIBED_RADIUS_OBSTACLE)
    LOG(INFO) << "ERROR cosmap";
#endif
  bool empty = false;
  for (auto i : local_map)
    for (auto j : i)
      empty = empty || (j == LETHAL_OBSTACLE);
  if (!empty)
    LOG(INFO) << "EMPTY MAP---COSTMAP" << robot_x << " " << robot_y << "robot_yaw";
  return local_map;
}
void Costmap::EnQueue(int x, int y, int src_x, int src_y, std::vector<std::vector<bool>>& seen, std::priority_queue<Cell>& q, std::vector<std::vector<unsigned char>>& cost_map)
{
  Cell t(x, y, src_x, src_y, 0);
  GetDistanceInGrid(t);
  if (t.dis * this->getResolution() > inflation_radius_)
    return; /** @brief out of range */
  unsigned char cost = GetCost(t);
  if (!seen[x][y]) {
    q.push(t);
    seen[x][y] = true;
  }
}
unsigned char Costmap::GetCellCost(double x, double y, int& cx, int& cy)
{
  cx = -1, cy = -1;
  int ox = this->getXInMap();
  int oy = this->getYInMap();
  int new_x = x / this->getResolution();
  int new_y = y / this->getResolution();
  int cost_x = ox - new_x;
  int cost_y = oy - new_y;
  if (cost_x < 0 || cost_y < 0 || cost_x >= costmap_need_.size() || (costmap_need_.size() > 0 && cost_y >= costmap_need_[0].size())) {
    return LETHAL_OBSTACLE;
  } else {
    cx = cost_x, cy = cost_y;
    return costmap_need_[cost_x][cost_y];
  }
}
} // end namespace costmap_2d
