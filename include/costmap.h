#ifndef COSTMAP_H
#define COSTMAP_H
#include "layer.h"
#include <atomic>
#include <string>
#include <unordered_map>
#include <memory>
#include <thread>
namespace costmap {

class Costmap {
  private:
    std::atomic<int> plugin;
  public:
  Costmap();
  virtual ~Costmap();
};
}

#endif /* COSTMAP_H */
