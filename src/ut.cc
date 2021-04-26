#include "ut.h"
#include "opencv2/core/persistence.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>
#include <glog/logging.h>
#include <sstream>
#include <utility>
#include <vector>

Ut::Ut(std::string config_file_path)
    : costmap_2d::Layer("UT")
{
  LOG(INFO) << "Initial UT";
  LOG(INFO) << "Start read config";
  cv::FileStorage fs(config_file_path, cv::FileStorage::READ);
  fs["map_size_metre"] >> map_size_;
  fs["delay_times"] >> delay_times_;
  fs["use_view"] >> use_view_;
  fs.release();
  LOG(INFO) << "Set UT config ok" << use_view_;
}
double Ut::Gamma(double theta)
{
  if (fabs(theta) > max_angle_)
    return 0.0;
  else
    return 1 - pow(theta / max_angle_, 2);
}

double Ut::Delta(double phi)
{
  return 1 - (1 + tanh(2 * (phi - phi_v_))) / 2;
}

double Ut::SensorModel(double r, double phi, double theta)
{
  double lbda = Delta(phi) * Gamma(theta);
  double delta = this->getResolution();
  if (phi >= 0.0 && phi < r - 2 * delta * r)
    return (1 - lbda) * (0.5) / 100;
  else if (phi < r - delta * r)
    return lbda * 0.5 * pow((phi - (r - 2 * delta * r)) / (delta * r), 2) + (1 - lbda) * .5;
  else if (phi < r + delta * r) {
    double J = (r - phi) / (delta * r);
    return lbda * ((1 - (0.5) * pow(J, 2)) - 0.5) + 0.5;
  } else
    return 0.5;
}

void Ut::update_cell(double ox, double oy, double ot, double r, double nx, double ny, bool clear)
{
  int x, y;
  ToGridMapPos(nx, ny, x, y);
  double dx = nx - ox, dy = ny - oy;
  double theta = atan2(dy, dx) - ot;
  theta = NormalizeAngle(theta);
  double phi = sqrt(dx * dx + dy * dy);
  double sensor = 0.0;
  if (!clear)
    sensor = SensorModel(r, phi, theta);
  double prior = getPreProb(x, y);
  double prob_occ = sensor * prior;
  double prob_not = (1 - sensor) * (1 - prior);
  double new_prob = prob_occ / (prob_occ + prob_not);
  //probability_map_[std::make_pair(x, y)] = new_prob;
  if (use_view_)
    probability_map_[std::make_pair(x, y)] = 1;
}

void Ut::FeedData(std::vector<UtSensor> data, double robot_x, double robot_y, double robot_yaw)
{
  LOG(INFO) << "START UT COMPUTE";
  if(read_times_ != 0 && read_times_ % delay_times_ == 0)
  {
    probability_map_.clear();
    read_times_ = 0;
  }
  for (int i = 0; i < data.size(); i++) {
    max_angle_ = data[i].fov / 2;
    last_change_.clear();
    inflate_cone_ = 0.7;
    int have_angle = 0;
    if (data[i].detect_dis < 0.01 || data[i].detect_dis > data[i].max_dis) /** @brief ignore distance of max */
      continue;
    double target_x_local = data[i].detect_dis * cos(data[i].toward_angle);
    double target_y_local = data[i].detect_dis * sin(data[i].toward_angle);
    double sensor_x_local = 0, sensor_y_local = 0;
    double ox, oy, tx, ty;
    LocalToGlobl(data[i], sensor_x_local, sensor_y_local, ox, oy, robot_x, robot_y, robot_yaw);
    LocalToGlobl(data[i], target_x_local, target_y_local, tx, ty, robot_x, robot_y, robot_yaw);
    double dx = tx - ox, dy = ty - oy, theta = atan2(dy, dx), d = sqrt(dx * dx + dy * dy);

    // Integer Bounds of Update
    int bx0, by0, bx1, by1;

    // Triangle that will be really updated; the other cells within bounds are ignored
    // This triangle is formed by the origin and left and right sides of sonar cone
    int Ox, Oy, Ax, Ay, Bx, By, Tx, Ty;

    // Bounds includes the origin
    ToGridMapPos(ox, oy, Ox, Oy);
    ToGridMapPos(tx, ty, Tx, Ty);
    probability_map_[std::make_pair(Tx, Ty)] = 0.91;
    last_change_[std::make_pair(Tx, Ty)] = true;
    bx1 = bx0 = Ox;
    by1 = by0 = Oy;
    double mx, my;

    // Update left side of sonar cone
    mx = ox + cos(theta - max_angle_) * d * 1.2;
    my = oy + sin(theta - max_angle_) * d * 1.2;

    ToGridMapPos(mx, my, Ax, Ay);
    bx0 = std::min(bx0, Ax);
    bx1 = std::max(bx1, Ax);
    by0 = std::min(by0, Ay);
    by1 = std::max(by1, Ay);

    // Update right side of sonar cone
    mx = ox + cos(theta + max_angle_) * d * 1.2;
    my = oy + sin(theta + max_angle_) * d * 1.2;

    ToGridMapPos(mx, my, Bx, By);
    bx0 = std::min(bx0, Bx);
    bx1 = std::max(bx1, Bx);
    by0 = std::min(by0, By);
    by1 = std::max(by1, By);
    for (int x = bx0; x <= (int)bx1; x++) {
      for (int y = by0; y <= (int)by1; y++) {
        bool update_xy_cell = true;

        // Unless inflate_cone_ is set to 100 %, we update cells only within the (partially inflated) sensor cone,
        // projected on the costmap as a triangle. 0 % corresponds to just the triangle, but if your sensor fov is
        // very narrow, the covered area can become zero due to cell discretization. See wiki description for more
        // details
        if (inflate_cone_ < 1.0) {
          // Determine barycentric coordinates
          int w0 = orient2d(Ax, Ay, Bx, By, x, y);
          int w1 = orient2d(Bx, By, Ox, Oy, x, y);
          int w2 = orient2d(Ox, Oy, Ax, Ay, x, y);

          // Barycentric coordinates inside area threshold; this is not mathematically sound at all, but it works!
          float bcciath = -inflate_cone_ * area(Ax, Ay, Bx, By, Ox, Oy);
          update_xy_cell = w0 >= bcciath && w1 >= bcciath && w2 >= bcciath;
        }
        if (update_xy_cell) {
          double wx, wy;
          bool clear = false;
          last_change_[std::make_pair(x, y)] = true;
          wx = 1.0 * x * getResolution(), wy = 1.0 * y * getResolution();
          update_cell(ox, oy, theta, data[i].detect_dis, wx, wy, clear);
          have_angle++;
        }
      }
    }
    LOG(INFO) << "have_angel" << have_angle;
  }
  LOG(INFO) << "FINISH UT COMPUTE";
  read_times_++;
}
std::vector<std::vector<bool>> Ut::getGridMap(double x, double y)
{
  double min_x = x - map_size_ / 2, min_y = y - map_size_ / 2;
  double max_x = x + map_size_ / 2, max_y = y + map_size_ / 2;
  std::pair<int, int> origin, min_pos, max_pos;
  ToGridMapPos(x + map_size_ / 2, y + map_size_ / 2, origin.first, origin.second);
  ToGridMapPos(min_x, min_y, min_pos.first, min_pos.second), ToGridMapPos(max_x, max_y, max_pos.first, max_pos.second);
  std::vector<std::vector<bool>> local_map(max_pos.second - min_pos.second, std::vector<bool>(max_pos.first - min_pos.second, false));
  bool flag = false;
  for (int i = 0; i < max_pos.second - min_pos.second; i++) {
    for (int j = 0; j < max_pos.first - min_pos.first; j++) {
      int globl_x = origin.first - i, globl_y = origin.second - j;
      //if (last_change_[std::make_pair(globl_x, globl_y)])
      local_map[i][j] = (getPreProb(globl_x, globl_y) > 0.5) ? 1 : 0;
      flag = flag || local_map[i][j];
      //else
      //local_map[i][j] = 0;
    }
  }
  if (!flag) {
    LOG(ERROR) << "EMPTY MAP: " << x << " " << y;
  }
  LOG(INFO) << "Get ut map";
  return local_map;
}
