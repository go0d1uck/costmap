#include "costmap.h"
#include <iostream>
#include <vector>
void printMap(std::vector<std::vector<bool>> t)
{
  for (int i = 0; i < t.size(); i++) {
    for (int j = 0; j < t[i].size(); j++)
      std::cout << t[i][j];
    std::cout << std::endl;
  }
}
int main()
{
  std::vector<std::vector<bool>> input = { { 0, 1, 0 }, { 1, 1, 0 }, { 0, 1, 1 } };
  std::vector<std::vector<bool>> input2 = { { 1, 1, 0 }, { 0, 0, 1 }, { 0, 0, 0 } };
  costmap_2d::Costmap my_costmap(0.0, 0.0, 0.0, 0.15);
  my_costmap.AddPlug(input, "mytest", 0.0, 0.0, 0.0, 0.15);
  my_costmap.AddPlug(input2, "input2", 0.0, 0.0, 0.0, 0.15);
  my_costmap.GetLayeredMap(0.0, 0.0);
  printMap(my_costmap.GetLayeredMap(0, 0));
  return 0;
}
