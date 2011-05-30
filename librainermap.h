#ifndef MAP_N
#define MAP

#include <stdio.h>
#include "stdlib.h"
#include "lib2d.h"

#define OBS_CHAR '#'
#define CLN_CHAR ' '
#define DTY_CHAR '='
#define RBT_CHAR '@'

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
  Cell **m; /* Mapa bi-dimensional l'inicialitzara i alocarà el constructor */
  Coor robotCoor;
  int xmax, ymax;
  
  bool isDirty(Coor c) { return m[c.x][c.y].state == DIRTY; }
  bool isClean(Coor c) { return m[c.x][c.y].state == CLEAN; }
  bool isObstacle(Coor c) { return m[c.x][c.y].state == OBSTACLE; }
  
  char charOf(int x, int y);
  
  RainerMap() { };
  
public:
  RainerMap(int sizex, int sizey, double cellEdge, Coor robotCoor, Point2D robotPoint);
    
  void setRobotPos(Coor c);
  Coor getNextPos(State s);
  Point2D getRealXY(Coor c);  
  
  void mark(Coor c, State s);
  
  bool isClean();
  bool isAbsolutelyClean();
  
  void printMap();
  
};
#endif
