#ifndef LAYER_H
#define LAYER_H
#include <memory>
#include <sstream>
#include <string>
#include <vector>
namespace costmap {

class Layer {
  private:
  int size_;
  std::vector<std::vector<bool>> gridMap_;
  std::string name_;
  int resolution_;
  double origin_x_;
  double origin_y_;

  public:
  Layer(std::vector<std::vector<bool>> gridMap, std::string name, double robot_x, double robot_y, double robot_yaw, int size);
  virtual ~Layer();
};
} // namespace costmap

#endif /* LAYER_H */
