#ifndef LAYER_H
#define LAYER_H
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
namespace costmap_2d {

class Layer {
  private:
  double size_;
  std::vector<std::vector<bool>> grid_map_;
  std::string name_;
  double origin_x_;
  double origin_y_;
  double resolution_;

  public:
  /** @brief Constructor for Layer */
  Layer(std::string name, double robot_x, double robot_y, double robot_yaw, double size);
  /** @brief Destructor for Layer */
  ~Layer(){}
  double getSize() const { return size_; }
  double getOriginX() const { return origin_x_; }
  double getOriginY() const { return origin_y_; }
  int getResolution() const { return resolution_; }
  int getXInMap() const {return origin_x_*100/resolution_;}
  int getYInMap() const {return origin_y_*100/resolution_;}
  std::string getName() const { return name_; }
  /** @brief Update layer */
  void Update(std::vector<std::vector<bool>>& grid_map, double robot_x, double robot_y);
  void ResetGridMap();
  void UpdateOrigin(double new_x,double new_y);
  void setGridMap(std::vector<std::vector<bool>>& grid_map){grid_map_ = grid_map;}
  std::vector<std::vector<bool>> getMap(){return grid_map_;}
};
} // namespace costmap

#endif /* LAYER_H */
