#include "costmap.h"
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
}
int main()
{
  double map_size = 8,x,y,yaw;
  string name;
  freopen("/home/antraume/costmap/test/grid_data.txt","r",stdin);
  costmap_2d::Costmap my_costmap(0.0, 0.0, 0.0, map_size);
  while(cin >> name >> x >> y >> yaw)
  {
    vector<vector<bool>> input(160,vector<bool>(160,false));
    for(int i = 0;i < input.size();i++)
    {
      for(int j = 0;j < input.size();j++)
      {
        int t;
        cin >> t;
        input[i][j] = bool(t);
      }
    }
    cout << name << " " << x << " " << y << " " << yaw << endl;
    my_costmap.AddPlug(input, "RGBD", x, y, yaw, map_size);
    printMap(my_costmap.GetCostMap(x,y,yaw));
    //printMap(my_costmap.GetLayeredMap(x,y));
  }
  return 0;
}
