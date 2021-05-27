#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include "include/costmap.h"
#include "include/laser.h"
#include "include/ut.h"
void split(const std::string& s, std::vector<std::string>* tokens,
           const char& delim = ' ') {
  tokens->clear();
  size_t lastPos = s.find_first_not_of(delim, 0);
  size_t pos = s.find(delim, lastPos);
  while (lastPos != std::string::npos) {
    tokens->emplace_back(s.substr(lastPos, pos - lastPos));
    lastPos = s.find_first_not_of(delim, pos);
    pos = s.find(delim, lastPos);
  }
}
template <typename DATA>
void printMap(const std::vector<std::vector<DATA>>& t) {
  for (int i = 0; i < t.size(); i++) {
    for (int j = 0; j < t[i].size(); j++) std::cout << double(t[i][j]) << " ";
    std::cout << std::endl;
  }
  // cout << "center" << int(t[80][80])<< endl;
}
void SetTestMap(std::vector<std::vector<bool>>* testMap, int x, int y,
                int width, int height) {
  for (int i = std::max(0, x - height / 2); i < std::min(x + height, 201);
       ++i) {
    for (int j = std::max(0, y - width / 2); j < std::min(y + width / 2, 201);
         ++j) {
      (*testMap)[i][j] = true;
    }
  }
}
double ut_degree[] = {1.308, 0.523, -0.523, -1.308};
int main() {
  // freopen("/home/antraume/costmap/test/grid_data.txt", "r", stdin);
  // freopen("~/py_project/visual_map/heatpot/gridmap1.txt", "r", stdin);
  // freopen("/home/antraume/Downloads/NitroShare/square.txt", "r", stdin);
  freopen("/home/antraume/Downloads/NitroShare/data/lidar_data-512-2.txt", "r",
          stdin);
  costmap_2d::Costmap::getInstance(
      "/home/antraume/costmap/costmap_config.yaml");
  auto static_map =
      std::vector<std::vector<bool>>(100, std::vector<bool>(100, 0));
  for (int i = 40; i < 60; i++)
    for (int j = 40; j < 60; j++) static_map[i][j] = 1;
  costmap_2d::Costmap::getInstance().AddPlug(static_map, "test", 0, 0, 0);
  std::cout << "test" << std::endl;
  printMap(costmap_2d::Costmap::getInstance().UpdateCostMap(0, 0, 0.78));
  std::cout << "test" << std::endl;
  return 0;
}
