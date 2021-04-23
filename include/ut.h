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
  double inflate_cone_;
  std::map<std::pair<int, int>, double> probability_map_;
  std::map<std::pair<int, int>, bool> last_change_;
  /** @brief get pos of sensor */
  double NormalizeAngle(double angle)
  {
    double a = fmod(fmod(angle, 2.0 * PI) + 2.0 * PI, 2.0 * PI);
    if (a > PI)
      a -= 2.0 * PI;
    return a;
  }
  std::pair<double, double> TransCoordinate(double source_x, double source_y, double theta, double o_x, double o_y)
  {
    double destination_x = source_x * cos(theta) - source_y * sin(theta) + o_x;
    double destination_y = source_x * sin(theta) + source_y * cos(theta) + o_y;
    return std::make_pair(destination_x, destination_y);
  }
  void LocalToGlobl(const UtSensor& sensor, double local_x, double local_y, double& global_x, double& global_y, double robot_x, double robot_y, double robot_yaw)
  {
    double x = TransCoordinate(local_x, local_y, 0, sensor.detect_dis * cos(sensor.toward_angle), sensor.detect_dis * sin(sensor.toward_angle)).first;
    double y = TransCoordinate(local_x, local_y, 0, sensor.detect_dis * cos(sensor.toward_angle), sensor.detect_dis * sin(sensor.toward_angle)).second;
    global_x = TransCoordinate(x, y, robot_yaw, robot_x, robot_y).first;
    global_y = TransCoordinate(x, y, robot_yaw, robot_x, robot_y).second;
  }
  void update_cell(double ox, double oy, double ot, double r, double nx, double ny, bool clear);
  double Gamma(double theta);
  double Delta(double phi);
  double SensorModel(double r, double phi, double theta);
  void ToGridMapPos(double x, double y, int& grid_x, int& grid_y)
  {
    grid_x = int(x / getResolution()), grid_y = int(y / getResolution());
  }
  double getPreProb(int x, int y)
  {
    if (probability_map_.find(std::make_pair(x, y)) == probability_map_.end())
      return 0.5;
    else
      return probability_map_[std::make_pair(x, y)];
  }
  float area(int x1, int y1, int x2, int y2, int x3, int y3)
  {
    return fabs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
  };

  int orient2d(int Ax, int Ay, int Bx, int By, int Cx, int Cy)
  {
    return (Bx - Ax) * (Cy - Ay) - (By - Ay) * (Cx - Ax);
  };

  public:
  Ut(std::string config_file_path);
  void FeedData(std::vector<UtSensor> data, double robot_x, double robot_y, double robot_yaw);
  std::vector<std::vector<bool>> getGridMap(double x, double y);
  ~Ut() {};
};

#endif /* UT_H */
