#include "costmap.h"
#include "ut.h"
#include <cstdio>
#include <iostream>
#include <vector>
using namespace std;
template <typename DATA>
void printMap(std::vector<std::vector<DATA>> t)
{
  for (int i = 0; i < t.size(); i++) {
    for (int j = 0; j < t[i].size(); j++)
      std::cout << int(t[i][j]) << " ";
    std::cout << std::endl;
  }
  //cout << "center" << int(t[80][80])<< endl;
}
double ut_degree[] = { 1.308, 0.523, -0.523, -1.308 };
int main()
{
  //freopen("/home/antraume/costmap/test/grid_data.txt", "r", stdin);
  //freopen("~/py_project/visual_map/heatpot/gridmap1.txt", "r", stdin);
  freopen("/home/antraume/Downloads/NitroShare/UT.txt", "r", stdin);
  double x, y, yaw;
  vector<UtSensor> input;
  input.resize(4);
  for (int i = 0; i < input.size(); i++) {
    input[i].fov = 0.523;
    input[i].r = 0.26;
    input[i].max_dis = 0.8;
    input[i].theta = ut_degree[i];
    input[i].toward_angle = ut_degree[i];
  }
  vector<vector<bool>> local_map;
  costmap_2d::Costmap::getInstance("/home/antraume/costmap/costmap_config.yaml");
  Ut ut_layer("/home/antraume/costmap/ut_config.yaml");
  double a, b, c, d;
  while (cin >> a >> b >> c >> d >> x >> y >> yaw) {
    input[0].detect_dis = a / 100;
    input[1].detect_dis = b / 100;
    input[2].detect_dis = c / 100;
    input[3].detect_dis = d / 100;
    ut_layer.FeedData(input, x, y, yaw);
    local_map = ut_layer.getGridMap(x, y);
    LOG(INFO) << local_map[0].size();
    //costmap_2d::Costmap::getInstance().AddPlug(local_map, "UT", x, y, yaw);
    //printMap(costmap_2d::Costmap::getInstance().UpdateCostMap(x, y, yaw));
    printMap(local_map);
  }
  return 0;
}
