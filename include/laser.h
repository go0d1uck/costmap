#ifndef LASER_H
#define LASER_H
#include "costmap.h"
#include "layer.h"
#include <mutex>
#include <set>
#include <utility>
#include <vector>
class Laser : protected costmap_2d::Layer {
private:
    double map_size_, sensor_x_, sensor_y_, sensor_toward_angle_, max_dis_;
    std::mutex laser_map_mutex_;
    std::map<std::pair<int, int>, bool> save_map_;
    int clear_threshold_;
    int direct_[8][2] = {
        -1, 0, 1, 0, 0, -1, 0, 1, 1, 1, -1, -1, 1, -1, -1, 1
    };
    void ToGridMapPos(double x, double y, int& grid_x, int& grid_y)
    {
        grid_x = int(x / getResolution()), grid_y = int(y / getResolution());
    }
    std::pair<double, double> TransCoordinate(double source_x, double source_y, double theta, double o_x, double o_y)
    {
        double destination_x = source_x * cos(theta) - source_y * sin(theta) + o_x;
        double destination_y = source_x * sin(theta) + source_y * cos(theta) + o_y;
        return std::make_pair(destination_x, destination_y);
    }
    void LocalToGlobl(double toward_angle, double local_x, double local_y, double& global_x, double& global_y, double robot_x, double robot_y, double robot_yaw)
    {
        double x = TransCoordinate(local_x, local_y, sensor_toward_angle_, sensor_x_, sensor_y_).first;
        double y = TransCoordinate(local_x, local_y, sensor_toward_angle_, sensor_x_, sensor_y_).second;
        /** @brief robot coordinate */
        global_x = TransCoordinate(x, y, robot_yaw, robot_x, robot_y).first;
        global_y = TransCoordinate(x, y, robot_yaw, robot_x, robot_y).second;
    }

public:
    Laser(const std::string& config_file_path);
    void FeedDate(std::vector<float> ranges, std::vector<float> angles, float robot_x, float robot_y, float robot_yaw);
    std::vector<std::vector<bool>> getGridMap(double x, double y);
};

#endif /* LASER_H */
