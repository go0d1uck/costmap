#ifndef LAYER_H
#define LAYER_H
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
namespace costmap {

class Layer {
  private:
  double size_;
  std::vector<std::vector<bool>> gridMap_;
  std::string name_;
  double origin_x_;
  double origin_y_;

  public:
  /** @brief Constructor of Layer */
  Layer(std::vector<std::vector<bool>> gridMap, std::string name, double robot_x, double robot_y, double robot_yaw, double size);
  /** @brief Destructor of Layer */
  ~Layer();
  double getSize() const { return size_; }
  double getOriginX() const { return origin_x_; }
  double getOriginY() const { return origin_y_; }
  std::string getName() const { return name_; }
  /** @brief Update layer */
  void Update(std::vector<std::vector<bool>>& gridMap, double robot_x, double robot_y);
};
} // namespace costmap

#endif /* LAYER_H */
