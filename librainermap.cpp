#include "librainermap.h"

RainerMap::RainerMap (int sizex, int sizey, double cellEdge, Coor rc, Point2D robotPoint) {
  xmax = sizex; /* Recorda de 0 a 7! */  
  ymax = sizey;
  robotCell.x = rc.x;
  robotCell.y = rc.y;

  m = new Cell*[sizex];
  for (int x = 0; x < xmax; x++) {
    m[x] = new Cell[sizey];
  }
  
  for (int i = 0; i < xmax; i++) {
    for (int j = 0; j < ymax; j++) {
      m[i][j].state = DIRTY;
      m[i][j].realCenter.x = robotPoint.x + (i - robotCell.x) * cellEdge;
      m[i][j].realCenter.y = robotPoint.y + (j - robotCell.y) * cellEdge;
    }
  }
}
  
void RainerMap::setRobotPos(Coor c) {
  robotCell.x = c.x;
  robotCell.y = c.y;
}

Coor RainerMap::zigZagHorizontal() {
  Coor c; 
  
  if (robotCell.x < xmax-1) {
    c.x = robotCell.x + 1;
    c.y = robotCell.y;
  } else if (robotCell.x == (xmax-1) and robotCell.y < (ymax-1)) {
    c.x = 0;
    c.y = robotCell.y + 1;
  } else if (robotCell.x == (xmax-1) and robotCell.y == (ymax-1)) {
    c.x = 0;
    c.y = 0;
  }

  printf("\n%d-%d\n", c.x, c.y);

  return c;
}

Coor RainerMap::getNextPos(State s) {
  return zigZagHorizontal();
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
  if (robotCell.x == x and robotCell.y == y) {
    return RBT_CHAR;
  }  
  
  switch (m[x][y].state) {
    case OBSTACLE: return OBS_CHAR;
    case CLEAN:    return CLN_CHAR;
    case DIRTY:    return DTY_CHAR;
  }
  return 0;
}

void RainerMap::printMap() {
  for (int y = ymax -1; y+1; y--) {
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