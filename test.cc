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
int main()
{
  double x, y, yaw;
  string name;
  //freopen("/home/antraume/costmap/test/grid_data.txt", "r", stdin);
  freopen("/home/antraume/Python_study/heatpot/gridmap1.txt", "r", stdin);
  costmap_2d::Costmap::getInstance("/home/antraume/costmap/costmap_config.yaml");
  Ut ut_layer("/home/antraume/costmap/ut_config.yaml");
  while (cin >> name >> x >> y >> yaw) {
    vector<vector<bool>> input(160, vector<bool>(160, false));
    for (int i = 0; i < input.size(); i++) {
      for (int j = 0; j < input.size(); j++) {
        int t;
        cin >> t;
        input[i][j] = bool(t);
      }
    }
    cout << name << " " << x << " " << y << " " << yaw << endl;
    costmap_2d::Costmap::getInstance().AddPlug(input, "RGBD", x, y, yaw);
    printMap(costmap_2d::Costmap::getInstance().UpdateCostMap(x, y, yaw));
  }
  return 0;
}
