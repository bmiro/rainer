#ifndef MAP_N
#define MAP

#define CLEAN 0
#define DIRTY 1
#define OBSTACLE 2

enum state {CLEAN, DIRTY, OBSTACLE}

struct Coor {
  int x, y;
}

struct cell {
  state s;
  Point2D realCenter;
}

class Map {
private:
  int xmax;
  int ymax;
  cell m[x][y];
  Coor wai; /* Where am I */
  
  Coor nextCell();
  Coor nearCell(state s);
  bool culDeSac();
  
  Map() { };
  
public:
  
  Map(int cellsize, int x, int y);
  
  void init(state s);
  
  void setInitPosition(Point2D p, Coor c);
  
  void markCleaned(Point2D p);
  void markObstacled(Point2D p);
  void markDirty(Point2D p);
  
  bool sameCell(Point2D p1, Point2D p2);
  
  bool isCleaned(Point2D p);
  
  bool isCenter(Point2D p);
  bool isClean();
  bool isAbsolutelyClean();
  
  Point2D getNextPoint();
  
  Point2D getFirstObstacle();
  Point2D getNextObstacle();
  bool lastObstacle(Point2D p);
  
  
}
#endif