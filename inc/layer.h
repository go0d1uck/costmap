#ifndef LAYER_H
#define LAYER_H
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#define RESOLUTION 0.05
//#yawdefine TEST
namespace costmap_2d {

class Layer {
 private:
  std::vector<std::vector<bool>> grid_map_;
  std::string name_;
  double resolution_;

 public:
  /** @brief Constructor for Layer */
  Layer(std::string name) : name_(name), resolution_(RESOLUTION) {}
  /** @brief Destructor for Layer */
  ~Layer() {}
  std::pair<int, int> getSize() const {
    return std::make_pair(grid_map_.size(),
                          grid_map_.size() > 0 ? grid_map_[0].size() : 0);
  }
  double getOriginX() const { return origin_x_; }
  double getOriginY() const { return origin_y_; }
  double getResolution() const { return resolution_; }
  int getXInMap() const { return origin_x_ / resolution_; }
  int getYInMap() const { return origin_y_ / resolution_; }
  std::string getName() const { return name_; }
  /** @brief Update layer */
  void Update(const std::vector<std::vector<bool>>& grid_map, double robot_x,
              double robot_y, double yaw);
  void ResetGridMap();
  /** @brief 当更新的坐标系超出原坐标系所给图的范围时，填充值由fill_value决定 */
  void UpdateOrigin(double new_x, double new_y, std::vector<std::vector<bool>>&,
                    bool fill_value);
  void setGridMap(std::vector<std::vector<bool>>& grid_map) {
    grid_map_ = grid_map;
  }
  std::vector<std::vector<bool>> getMap() { return grid_map_; }

 protected:
  double origin_x_;
  double origin_y_;
  double robot_yaw_;
};
}  // namespace costmap_2d

#endif /* LAYER_H */
