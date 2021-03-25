#include "costmap.h"
#include "layer.h"
#include <exception>
#include <glog/logging.h>
#include <new>
#include <opencv2/core/persistence.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <vector>
namespace costmap_2d {
Costmap::Costmap(double robot_x, double robot_y, double robot_yaw, double size, std::string file_name)
{
  LOG(INFO) << "Start initing CostMap";
  LOG(INFO) << "Read config";
  LOG(INFO) << "Config Path"
            << " " << file_name;
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  try {
    fs["inflation_weight"] >> inflation_weight_;
    fs["inflation_radius"] >> inflation_radius_;
    fs["inscribed_radius"] >> inscribed_radius_;
    LOG(INFO) << "Set config ok";
  } catch (std::exception) {
    LOG(ERROR) << "Read config error!";
    throw;
  }
  fs.release();
  try {
    pLayered_ = std::shared_ptr<Layer>(new Layer("layered", robot_x, robot_y, robot_yaw, size));
  } catch (const std::bad_alloc& e) {
    /** @brief new error,add glog here */
    LOG(ERROR) << "malloc costmap failed!";
  }
  /** @brief init layered grid_map */
  pLayered_->ResetGridMap();
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
bool Costmap::AddPlug(std::vector<std::vector<bool>>& gridMap, std::string name, double robot_x, double robot_y, double robot_yaw, double size)
{
  LOG(INFO) << "Adding plugin---" << name;
  if (plugins_.find(name) != plugins_.end()) {
    LOG(INFO) << "Plugin is existed--just update";
  }
  {
    plugins_[name] = std::shared_ptr<Layer>(new Layer(name, robot_x, robot_y, robot_yaw, size));
  }
  plugins_[name]->Update(gridMap, robot_x, robot_y);
  LOG(INFO) << "Plugin---" << name << "---finished";
  return true;
}
std::vector<std::vector<bool>> Costmap::GetLayeredMap(double new_origin_x, double new_origin_y)
{
  /** @brief if no map */
  LOG(INFO) << "Rending layered map...";
  pLayered_->UpdateOrigin(new_origin_x, new_origin_y);
  if (plugins_.size() == 0) {
    LOG(INFO) << "No plugin...return self";
    return pLayered_->getMap();
  }
  /** @brief update all plugin origin*/
  for (auto it = plugins_.begin(); it != plugins_.end(); it++) {
    it->second->UpdateOrigin(new_origin_x, new_origin_y);
  }
  /** @brief start layered */
  pLayered_->ResetGridMap();
  std::vector<std::vector<bool>> local_map = pLayered_->getMap();
  for (auto it = plugins_.begin(); it != plugins_.end(); it++) {
    std::vector<std::vector<bool>> tmp_grid_map = it->second->getMap();
    for (int i = 0; i < tmp_grid_map.size(); i++) {
      for (int j = 0; j < tmp_grid_map[i].size(); j++) {
        local_map[i][j] = local_map[i][j] | tmp_grid_map[i][j]; /** @brief if one is obstacle,the map obstacle */
      }
    }
  }
  pLayered_->setGridMap(local_map);
  LOG(INFO) << "Rending finished";
  return local_map;
}

void Costmap::Inflation(std::vector<std::vector<unsigned char>>& return_costmap)
{
  LOG(INFO) << "Start inflation";
  int grid_inscribed_dis = inscribed_radius_ / pLayered_->getResolution();
  int limit = pLayered_->getMap().size();
  grid_inscribed_dis++;
  std::vector<std::vector<bool>> seen(limit, std::vector<bool>(limit, false));
  return_costmap = std::vector<std::vector<unsigned char>>(limit, std::vector<unsigned char>(limit, 0));
  std::priority_queue<Cell> que;
  std::vector<std::vector<bool>> local_map = pLayered_->getMap();
  LOG(INFO) << "Inflation initialized";
  /** @brief convert occupied to cosmap obstacle */
  for (int i = 0; i < limit; i++) {
    for (int j = 0; j < limit; j++) {
      if (local_map[i][j]) {
        return_costmap[i][j] = LETHAL_OBSTACLE;
        que.push(Cell(i, j, i, j, 0)); /** @brief push into queue */
        seen[i][j] = true;
      }
    }
  }
  while (!que.empty()) {
    const Cell& t = que.top();
    if (t.dis != 0)
      return_costmap[t.mx][t.my] = GetCost(t);
    que.pop();
    if (t.mx > 0)
      EnQueue(t.mx - 1, t.my, t.src_x, t.src_y, seen, que, return_costmap);
    if (t.my > 0)
      EnQueue(t.mx, t.my - 1, t.src_x, t.src_y, seen, que, return_costmap);
    if (t.mx < limit - 1)
      EnQueue(t.mx + 1, t.my, t.src_x, t.src_y, seen, que, return_costmap);
    if (t.my < limit - 1)
      EnQueue(t.mx, t.my + 1, t.src_x, t.src_y, seen, que, return_costmap);
  }
}
void Costmap::UpdateCostMap(double robot_x, double robot_y, double robot_yaw)
{
  std::vector<std::vector<unsigned char>> local_map;
  double new_origin_x = robot_x - pLayered_->getSize() / 2;
  double new_origin_y = robot_y - pLayered_->getSize() / 2;
  GetLayeredMap(new_origin_x, new_origin_y);
  Inflation(local_map);
  costmap_need_ = local_map;
  return;
}
void Costmap::EnQueue(int x, int y, int src_x, int src_y, std::vector<std::vector<bool>>& seen, std::priority_queue<Cell>& q, std::vector<std::vector<unsigned char>>& cost_map)
{
  Cell t(x, y, src_x, src_y, 0);
  GetDistanceIngrid(t);
  if (t.dis * pLayered_->getResolution() > inflation_radius_)
    return; /** @brief out of range */
  unsigned char cost = GetCost(t);
  if (!seen[x][y]) {
    q.push(t);
    seen[x][y] = true;
  }
}
unsigned char Costmap::GetCellCost(double x, double y)
{
  int ox = pLayered_->getXInMap();
  int oy = pLayered_->getYInMap();
  int new_x = x / pLayered_->getResolution();
  int new_y = y / pLayered_->getResolution();
  int cost_x = new_x - ox;
  int cost_y = new_y - oy;
  if (cost_x < 0 || cost_y < 0 || cost_x >= costmap_need_.size() || cost_x >= costmap_need_.size())
    return LETHAL_OBSTACLE;
  else
    return costmap_need_[cost_x][cost_y];
}
} // end namespace costmap_2d
