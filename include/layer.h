#ifndef LAYER_H
#define LAYER_H
namespace costmap {

class Layer {
  private:
  int size;
  

  public:
  Layer ();
  virtual ~Layer ();
};
} // namespace costmap

#endif /* LAYER_H */
