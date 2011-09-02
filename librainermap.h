#ifndef MAP_H
#define MAP_H

#include <stdio.h>
#include "Aria.h"

#include "stdlib.h"
#include "lib2d.h"
#include "common_rainer.h"

#define OBS_CHAR '#'
#define CLN_CHAR ' '
#define DTY_CHAR '='
#define RBT_CHAR '@'
#define NULL_COOR -1

enum State {CLEAN, DIRTY, OBSTACLE};

struct Cell {
  State state;
  Point2D realCenter;
};

class RainerMap {
private:
  Cell **m; /* Mapa bi-dimensional l'inicialitzara i alocarà el constructor */
  Coor robotCell;
  int xmax, ymax;
  double ce;
  
  bool isDirty(Coor c) { return m[c.x][c.y].state == DIRTY; }
  bool isClean(Coor c) { return m[c.x][c.y].state == CLEAN; }
  bool isObstacle(Coor c) { return m[c.x][c.y].state == OBSTACLE; }
  
  char charOf(int x, int y);
    
public:
  RainerMap(int sizex=8, int sizey=8, double cellEdge=500, int rcx=0, int rcy=0,
                                                           double rpx=0.0, double rpy=0.0);
    
  void setRobotPos(Coor c);
  
  Coor getNextPos(State s, double x, double y);
  Coor zigZag();
  
  Point2D getRealXY(Coor c);
  Coor getCellCoor(double px, double py, double th);
  
  void mark(Coor c, State s);
  void markAs(State orig, State final);
  
  bool isClean();
  bool isAbsolutelyClean();
  
  void printMap();
  
};
#endif
