#ifndef UT_H
#define UT_H
#include "layer.h"
#include <cmath>
#include <map>
#include <utility>
#include <vector>
#define PI acos(-1)
struct UtSensor {
  double dis;
  double theta;
  double r;
  double toward_angle;
};
class Ut : protected costmap_2d::Layer {
  private:
  double max_angle_;
  double phi_v_;
  bool delay_;
  std::map<std::pair<double, double>, double> probability_map;
  /** @brief get pos of sensor */
  void getSensorInWorld(double robot_x, double robot_y, double robot_yaw, double& sensor_x, double& sensor_y, double r, double theta);
  double Gamma(double theta);
  double Delta(double phi);
  void GetDeltas(double angle, double* dx, double* dy);
  double SensorModel(double r, double phi, double theta);
  std::pair<double, double> TransPosToWorld(const double& robot_x, const double& robot_y, const double& robot_yaw, const double& my_x, const double& my_y)
  {
    return std::make_pair(my_x * cos(robot_yaw) - my_y * sin(robot_yaw) + my_x, my_x * sin(robot_yaw) + my_y * cos(robot_yaw) + my_y);
  }

  public:
  Ut(std::string config_file_path);
  void FeedData(std::vector<UtSensor>);
  std::vector<std::vector<bool>> getGridMap(double x, double y);
  ~Ut() {};
};

#endif /* UT_H */
