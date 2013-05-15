#define YELLOW 1
#define BLUE 2
#define RED 3

struct Brick {
  std::string timestamp;
  int color;
  double x_coord;
  double y_coord;
  double angle;
  double width;
  double height;
};