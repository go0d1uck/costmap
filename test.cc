/**
* @brief: fuck
*
* @param: int a
*       : int b
*       : int c
*
* @return: int
*/
int getName(int a,int b,int c)
{
  return a+b+c;
}
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
  //cout << "center" << int(t[80][80])<< endl;
}
int main()
{
  double map_size = 8,x,y,yaw;
  string name;
  freopen("/home/antraume/costmap/test/grid_data.txt","r",stdin);
  costmap_2d::Costmap my_costmap(0.0, 0.0, 0.0, map_size,"/home/antraume/costmap/costmap.conf");
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
    double ox = x - map_size/2;
    double oy = y - map_size/2;
    my_costmap.UpdateCostMap(x,y,yaw);
    std::cout << int(my_costmap.GetCellCost(1.25,1.0));
    //printMap(my_costmap.GetLayeredMap(ox,oy));
  }
  return 0;
}
