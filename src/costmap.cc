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
    LOG(WARNING) << "Plugin is existed";
    return false; /** @brief plugin exist*/
  }
  plugins_[name] = std::shared_ptr<Layer>(new Layer(name, robot_x, robot_y, robot_yaw, size));
  plugins_[name]->Update(gridMap, robot_x, robot_y);
  LOG(INFO) << "Plugin---" << name << "---finished";
  return true;
}
std::vector<std::vector<bool>> Costmap::GetLayeredMap(double new_origin_x, double new_origin_y)
{
  /** @brief if no map */
  LOG(INFO) << "Rending layered map...";
  pLayered_->UpdateOrigin(new_origin_x, new_origin_y);
  if (plugins_.size() == 0)
  {
    LOG(INFO) << "No plugin...return self";
    return pLayered_->getMap();
  }
  for (auto it = plugins_.begin(); it != plugins_.end(); it++) {
    it->second->UpdateOrigin(new_origin_x, new_origin_y);
  }
  /** @brief start layered */
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
} // end namespace costmap_2d
