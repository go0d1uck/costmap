#ifndef UT_H
#define UT_H
#include "layer.h"
#include <cmath>
#include <map>
#include <utility>
#include <vector>
#define PI acos(-1)
#define PROB_THRESHOLD 0.5
struct UtSensor {
  double max_dis;
  double theta;
  double r;
  double toward_angle;
  double fov;
  double detect_dis;
};
class Ut : protected costmap_2d::Layer {
  private:
  double max_angle_;
  double phi_v_;
  bool delay_;
  double map_size_;
  std::map<std::pair<int, int>, double> probability_map_;
  /** @brief get pos of sensor */
  std::pair<double, double> trans_coordinate(double source_x, double source_y, double theta, double o_x, double o_y)
  {
    double destination_x = source_x * cos(theta) - source_y * sin(theta) + o_x;
    double destination_y = source_x * sin(theta) + source_y * cos(theta) + o_y;
    return std::make_pair(destination_x, destination_y);
  }
  double Gamma(double theta);
  double Delta(double phi);
  void GetDeltas(double angle, double* dx, double* dy);
  double SensorModel(double r, double phi, double theta);
  std::pair<double, double> TransPosToWorld(const double& robot_x, const double& robot_y, const double& robot_yaw, const double& my_x, const double& my_y)
  {
    return std::make_pair(my_x * cos(robot_yaw) - my_y * sin(robot_yaw) + my_x, my_x * sin(robot_yaw) + my_y * cos(robot_yaw) + my_y);
  }
  std::pair<int, int> ToGridMapPos(double x, double y)
  {
    return std::make_pair(int(x / getResolution()), int(y / getResolution()));
  }
  double getPreProb(int x, int y)
  {
    if (probability_map_.find(std::make_pair(x, y)) == probability_map_.end())
      return 0.5;
    else
      return probability_map_[std::make_pair(x, y)];
  }

  public:
  Ut(std::string config_file_path);
  void FeedData(std::vector<UtSensor> data, double robot_x, double robot_y, double robot_yaw);
  std::vector<std::vector<bool>> getGridMap(double x, double y);
  ~Ut() {};
};

#endif /* UT_H */
