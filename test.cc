#include "costmap.h"
#include "laser.h"
#include "ut.h"
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>
using namespace std;
void split(const string& s, vector<string>& tokens, const char& delim = ' ')
{
  tokens.clear();
  size_t lastPos = s.find_first_not_of(delim, 0);
  size_t pos = s.find(delim, lastPos);
  while (lastPos != string::npos) {
    tokens.emplace_back(s.substr(lastPos, pos - lastPos));
    lastPos = s.find_first_not_of(delim, pos);
    pos = s.find(delim, lastPos);
  }
}
template <typename DATA>
void printMap(std::vector<std::vector<DATA>> t)
{
  for (int i = 0; i < t.size(); i++) {
    for (int j = 0; j < t[i].size(); j++)
      std::cout << double(t[i][j]) << " ";
    std::cout << std::endl;
  }
  //cout << "center" << int(t[80][80])<< endl;
}
void SetTestMap(std::vector<std::vector<bool>>& testMap, int x, int y, int width, int height)
{
  for (int i = max(0, x - height / 2); i < min(x + height, 201); ++i) {
    for (int j = max(0, y - width / 2); j < min(y + width / 2, 201); ++j) {
      testMap[i][j] = true;
    }
  }
}
double ut_degree[] = { 1.308, 0.523, -0.523, -1.308 };
int main()
{
  //freopen("/home/antraume/costmap/test/grid_data.txt", "r", stdin);
  //freopen("~/py_project/visual_map/heatpot/gridmap1.txt", "r", stdin);
  //freopen("/home/antraume/Downloads/NitroShare/square.txt", "r", stdin);
  freopen("/home/antraume/Downloads/NitroShare/data/lidar_data-512-2.txt", "r", stdin);
  //double x, y, yaw;
  //vector<UtSensor> input;
  //input.resize(4);
  //for (int i = 0; i < input.size(); i++) {
  //input[i].fov = 0.523;
  //input[i].r = 0.26;
  //input[i].max_dis = 2;
  //input[i].theta = ut_degree[i];
  //input[i].toward_angle = ut_degree[i];
  //}
  //vector<vector<bool>> local_map;
  //costmap_2d::Costmap::getInstance("/home/antraume/costmap/costmap_config.yaml");
  //Ut ut_layer("/home/antraume/costmap/ut_config.yaml");
  //double a, b, c, d;
  //int i = 0;
  //while (cin >> a >> b >> c >> d >> x >> y >> yaw) {
  //LOG(INFO) << "index:" << i;
  //input[0].detect_dis = a / 100;
  //input[1].detect_dis = b / 100;
  //input[2].detect_dis = c / 100;
  //input[3].detect_dis = d / 100;
  //ut_layer.FeedData(input, x, y, yaw);
  //local_map = ut_layer.getGridMap(x, y);
  //cout << x << " " << y << endl;
  //printMap(local_map);
  //i++;
  //costmap_2d::Costmap::getInstance().AddPlug(local_map, "UT", x, y, yaw);
  //printMap(costmap_2d::Costmap::getInstance().UpdateCostMap(x, y, yaw));
  //}
  //costmap_2d::Costmap::getInstance("/home/antraume/costmap/costmap_config.yaml");
  //freopen("/home/antraume/costmap/test/grid_data.txt", "r", stdin);
  //vector<vector<bool>> local_map;
  //local_map.resize(100);
  //for(int i = 0;i < local_map.size();i++)
  //{
  //local_map[i].resize(100,0);
  //for(int j = i;j < 100-i;j++) local_map[i][j] = 1;
  //}
  //printMap(local_map);
  //costmap_2d::Costmap::getInstance().AddPlug(local_map, "test", 0, 0, 0);
  //printMap(costmap_2d::Costmap::getInstance().UpdateCostMap(0, 0, 0));
  //int x,y;
  //for(double i = -5;i <= 5;i+=0.05)
  //{
  //for(double j = -5;j <= 5;j+=0.05)
  //{
  //cout << (costmap_2d::Costmap::getInstance().GetCellCost(i,j,x,y)==255?1:0) << " ";
  //}
  //cout << endl;
  //}
  // static map test
  costmap_2d::Costmap::getInstance("/home/antraume/costmap/costmap_config.yaml");
  auto static_map = vector<vector<bool>>(100, vector<bool>(100, 0));
  for (int i = 40; i < 60; i++)
    for (int j = 40; j < 60; j++)
      static_map[i][j] = 1;
  costmap_2d::Costmap::getInstance().AddPlug(static_map, "test", 0, 0, 0);
  std::cout << "test" << std::endl;
  printMap(costmap_2d::Costmap::getInstance().UpdateCostMap(0, 0, 0.78));
  std::cout << "test" << std::endl;
  return 0;
}
