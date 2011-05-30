#include "librainermap.h"

RainerMap::RainerMap (int sizex, int sizey, double cellEdge, Coor rc, Point2D robotPoint) {
  xmax = sizex; 
  ymax = sizey;
  robotCoor.x = rc.x;
  robotCoor.y = rc.y;

  m = new Cell*[sizex];
  for (int x = 0; x < xmax; x++) {
    m[x] = new Cell[sizey];
  }
  
  for (int i = 0; i < xmax; i++) {
    for (int j = 0; j < ymax; j++) {
      m[i][j].state = DIRTY;
      m[i][j].realCenter.x = robotPoint.x + (i - robotCoor.x) * cellEdge;
      m[i][j].realCenter.y = robotPoint.y + (j - robotCoor.y) * cellEdge;
    }
  }
}
  
void RainerMap::setRobotPos(Coor c) {
  robotCoor.x = c.x;
  robotCoor.y = c.y;
}

Coor RainerMap::getNextPos(State s) {
  Coor c; 

  if (robotCoor.x < xmax) {
    c.x = robotCoor.x + 1;
    c.y = robotCoor.y;
  } else if (robotCoor.y < ymax) {
    c.x = 0;
    c.y = robotCoor.y + 1;
  } else {
    c.x = 0;
    c.y = 0;
  }

  return c;

}

Point2D RainerMap::getRealXY(Coor c) {

  Point2D p;

  p.x = m[c.x][c.y].realCenter.x;
  p.y = m[c.x][c.y].realCenter.y;

  return p;
}

void RainerMap::mark(Coor c, State s) {
  m[c.x][c.y].state = s;
}

bool RainerMap::isClean() {
  for (int i = 0; i < xmax; i++) {
    for (int j = 0; j < ymax; j++) {
      if (m[i][j].state == DIRTY) {
        return false;
      }
    }
  }
  return true;
}

bool RainerMap::isAbsolutelyClean() {
  for (int i = 0; i < xmax; i++) {
    for (int j = 0; j < ymax; j++) {
      if (m[i][j].state != CLEAN) {
        return false;
      }
    }
  }
  return true;
}

char RainerMap::charOf(int x, int y) {
  if (robotCoor.x == x and robotCoor.y == y) {
    return RBT_CHAR;
  }  
  
  switch (m[x][y].state) {
    case OBSTACLE: return OBS_CHAR;
    case CLEAN:    return CLN_CHAR;
    case DIRTY:    return DTY_CHAR;
  }
}

void RainerMap::printMap() {
  for (int y = ymax; y ; y--) {
    printf("%d |", y);
    for (int x = 0; x < xmax; x++) {
        printf("%c", charOf(x, y));
    }
    printf("\n");
  }
  printf("   --------\n");
  printf("   ");
  for (int x = 0; x < xmax; x++) {
      printf("%d", x);
  }
  printf("\n");
}