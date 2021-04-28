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
  fs["un_refresh_size"] >> un_refresh_size_;
  fs["inflate_cone"] >> inflate_cone_;
  fs["clear_dis"] >> clear_dis_;
  fs["single_frame"] >> single_frame_;
  fs.release();
  LOG(INFO) << "Set UT config ok";
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

void Ut::update_cell(double ox, double oy, double ot, double r, double nx, double ny, bool clear, int tx, int ty)
{
  double dx = nx - ox, dy = ny - oy;
  double theta = atan2(dy, dx) - ot;
  theta = NormalizeAngle(theta);
  double phi = sqrt(dx * dx + dy * dy);
  double sensor = 0.0;
  if (!clear)
    sensor = SensorModel(r, phi, theta);
  double prior = getPreProb(tx, ty);
  double prob_occ = sensor * prior;
  double prob_not = (1 - sensor) * (1 - prior);
  double new_prob = prob_occ / (prob_occ + prob_not);
  probability_map_[std::make_pair(tx, ty)] = new_prob;
  if (use_view_)
    probability_map_[std::make_pair(tx, ty)] = 1;
}

void Ut::FeedData(std::vector<UtSensor> data, double robot_x, double robot_y, double robot_yaw)
{
  LOG(INFO) << "START UT COMPUTE";
  if (read_times_ != 0 && read_times_ % delay_times_ == 0) {
    probability_map_.clear();
    read_times_ = 0;
  }
  for (int i = 0; i < data.size(); i++) {
    max_angle_ = data[i].fov / 2;
    int have_angle = 0;
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
    if (single_frame_)
      continue;
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
    /** @brief clear out-of-range data */
    //std::pair<int, int> un_refresh_min, un_refresh_max;
    //ToGridMapPos(robot_x - un_refresh_size_, robot_y - un_refresh_size_, un_refresh_min.first, un_refresh_min.second);
    //ToGridMapPos(robot_x + un_refresh_size_, robot_y + un_refresh_size_, un_refresh_max.first, un_refresh_max.second);
    //for (auto it = probability_map_.begin(); it != probability_map_.end(); it++) {
    //if (it->first.first < un_refresh_min.first || it->first.first > un_refresh_max.first || it->first.second < un_refresh_min.second || it->first.second > un_refresh_max.second) {
    //probability_map_[it->first] = 0.5;
    //}
    //}
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
          wx = 1.0 * x * getResolution(), wy = 1.0 * y * getResolution();
          if (data[i].detect_dis > data[i].max_dis)
            clear = true;
          update_cell(ox, oy, theta, data[i].detect_dis, wx, wy, clear, x, y);
          have_angle++;
        }
      }
    }
    LOG(INFO) << "have_angel: " << have_angle;
  }
  read_times_++;
  std::pair<int, int> robot_pos;
  //if (read_times_ > 10) {
  //ToGridMapPos(robot_x, robot_y, robot_pos.first, robot_pos.second);
  //for (auto it = probability_map_.begin(); it != probability_map_.end(); it++) {
  //double dis = sqrt((it->first.first - robot_pos.first) * (it->first.first - robot_pos.first) - (it->first.second - robot_pos.second) * (it->first.second - robot_pos.second)) * getResolution();
  //if (it->second > 0.5 && dis != 0 && dis < clear_dis_) {
  //it->second = 0.5;
  //}
  //}
  //}
  LOG(INFO) << "FINISH UT COMPUTE";
}
std::vector<std::vector<bool>> Ut::getGridMap(double x, double y)
{
  LOG(INFO) << "start get ut";
  std::vector<std::vector<bool>> local_map(map_size_ / getResolution(), std::vector<bool>(map_size_ / getResolution(), false));
  std::pair<int, int> origin;
  ToGridMapPos(x + map_size_ / 2, y + map_size_ / 2, origin.first, origin.second);
  LOG(INFO) << "ut pose:" << x << " " << y;
  for (int i = 0; i < local_map.size(); i++) {
    for (int j = 0; j < local_map[i].size(); j++) {
      int globl_x = origin.first - i, globl_y = origin.second - j;
      local_map[i][j] = (getPreProb(globl_x, globl_y) > 0.5) ? 1 : 0;
    }
  }
  LOG(INFO) << "get ut ok";
  return local_map;
}
