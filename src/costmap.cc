#include "costmap.h"
#include "layer.h"
#include <glog/logging.h>
#include <new>
#include <vector>
namespace costmap_2d {
Costmap::Costmap(double robot_x, double robot_y, double robot_yaw, double size)
{
  LOG(INFO) << "Start initing CostMap";
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
  int grid_inscribed_dis = INSCRIBED_RADIUS / pLayered_->getResolution();
  int limit = pLayered_->getMap().size();
  grid_inscribed_dis++;
  std::vector<std::vector<bool>> seen(limit, std::vector<bool>(limit, false));
  return_costmap.resize(limit);
  for (auto it = return_costmap.begin(); it != return_costmap.end(); it++)
    it->resize(limit);
  std::queue<Cell> q; /** @brief need inflation queue */
  std::vector<std::vector<bool>> local_map = pLayered_->getMap();
  LOG(INFO) << "Inflation initialized";
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
    if (cell_data.mx > 0 && !seen[cell_data.mx - 1][cell_data.my]) {
      q.push(Cell(cell_data.mx - 1, cell_data.my, cell_data.src_x, cell_data.src_y));
      seen[cell_data.mx - 1][cell_data.my] = true;
    }
    if (cell_data.my > 0 && !seen[cell_data.mx][cell_data.my - 1]) {
      q.push(Cell(cell_data.mx, cell_data.my - 1, cell_data.src_x, cell_data.src_y));
      seen[cell_data.mx][cell_data.my - 1] = true;
    }
    if (cell_data.mx < limit - 1 && !seen[cell_data.mx + 1][cell_data.my]) {
      q.push(Cell(cell_data.mx + 1, cell_data.my, cell_data.src_x, cell_data.src_y));
      seen[cell_data.mx + 1][cell_data.my] = true;
    }
    if (cell_data.my < limit - 1 && !seen[cell_data.mx][cell_data.my + 1]) {
      q.push(Cell(cell_data.mx, cell_data.my + 1, cell_data.src_x, cell_data.src_y));
      seen[cell_data.mx][cell_data.my + 1] = true;
    }
    /** @brief choose big cost */
    return_costmap[cell_data.mx][cell_data.my] = std::max(return_costmap[cell_data.mx][cell_data.my], GetCost(cell_data.mx, cell_data.my, cell_data.src_x, cell_data.src_y));
  }
}
std::vector<std::vector<unsigned char>> Costmap::GetCostMap(double robot_x, double robot_y, double robot_yaw)
{
  std::vector<std::vector<unsigned char>> local_map;
  double new_origin_x = robot_x - pLayered_->getSize() / 2;
  double new_origin_y = robot_y - pLayered_->getSize() / 2;
  GetLayeredMap(new_origin_x, new_origin_y);
  Inflation(local_map);
  return local_map;
}
} // end namespace costmap_2d
