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
  cell m[][];
  Coor robotCoor;
  
  isDirty(Coor c) { return m[c.x][c.y].state == DIRTY; }
  isClean(Coor c) { return m[c.x][c.y].state == CLEAN; }
  isObstacle(Coor c){ return m[c.x][c.y].state == OBSTACLE; }
  
  
public:
  Map(sizex, int sizey, Coor robotCoor, Point2D robotPoint);
  
  changeOrigen(Coor newCoorCenter, Point2D newPointCenter);
  
  void setRobotPos(Coor c);
  Coor getNextPos(state s);
  Coor whichCell(Point2D p);
  Point2D getRealXY(Coor c);  
  
  mark(Coor c, state s);
  
  bool isClean();
  bool isAbsolutelyClean();

}
#endif