#ifndef MAP_N
#define MAP

#include "common_rainer.h"

enum State {CLEAN, DIRTY, OBSTACLE};

struct Coor {
  int x, y;
};

struct Cell {
  State state;
  Point2D realCenter;
};

class RainerMap {
private:
  Cell **m;
  Coor robotCoor;
	int maxX, maxY;
  
  bool isDirty(Coor c) { return m[c.x][c.y].state == DIRTY; }
  bool isClean(Coor c) { return m[c.x][c.y].state == CLEAN; }
  bool isObstacle(Coor c) { return m[c.x][c.y].state == OBSTACLE; }
  
  RainerMap() { };
  
public:
  RainerMap(int sizex, int sizey, double cellEdge, Coor robotCoor, Point2D robotPoint);
  
  //void changeOrigen(Coor newCoorCenter, Point2D newPointCenter);
  
  void setRobotPos(Coor c);
  Coor getNextPos(State s);
  //Coor whichCell(Point2D p);
  Point2D getRealXY(Coor c);  
  
  void mark(Coor c, State s);
  
  bool isClean();
  bool isAbsolutelyClean();
};
#endif
