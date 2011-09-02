#include "librainermap.h"

RainerMap::RainerMap (int sizex, int sizey, double cellEdge, int rcx, int rcy,
                                                             double rpx, double rpy) {
  xmax = sizex; /* Recorda de 0 a 7! */  
  ymax = sizey;
  Coor c;
  
  c.x = rcx;
  c.y = rcy;
  
  setRobotPos(c);
  
  ce = cellEdge;
  m = new Cell*[sizex];
  for (int x = 0; x < xmax; x++) {
    m[x] = new Cell[sizey];
  }
  
  for (int i = 0; i < xmax; i++) {
    for (int j = 0; j < ymax; j++) {
      m[i][j].state = DIRTY;
      m[i][j].realCenter.x = rpx + (i - robotCell.x) * cellEdge;
      m[i][j].realCenter.y = rpy + (j - robotCell.y) * cellEdge;
    }
  }
  
}
  
void RainerMap::setRobotPos(Coor c) {
  robotCell.x = c.x;
  robotCell.y = c.y;
}

Coor RainerMap::zigZag() {
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

  printf("Robot esta a %d-%d\n", c.x, c.y);

  return c;
}

Coor RainerMap::getNextPos(State s, double x, double y) {
  Coor nearCell;
  double d, dmin;
  
  dmin = DBL_MAX;
  for (int i = 0; i < xmax; i++) {
    for (int j = 0; j < ymax; j++) {
      if (m[i][j].state == s) {
	d = ArMath::distanceBetween(m[i][j].realCenter.x, m[i][j].realCenter.y, x, y);
	if (d < dmin) {
	  dmin = d;
	  nearCell.x = i;
	  nearCell.y = j;
	}
      }
    }
  }
  if (dmin == DBL_MAX) {
    printf("No queden celles amb aquest estat\n\n");
    nearCell.x = NULL_COOR;
    nearCell.y = NULL_COOR;
  }
  
  printf("Estic a %d %d i el lloc mes proper es %d %d\n",
         robotCell.x, robotCell.y, nearCell.x, nearCell.y);
  
  return nearCell;
 
}

Point2D RainerMap::getRealXY(Coor c) {

  Point2D p;

  p.x = m[c.x][c.y].realCenter.x;
  p.y = m[c.x][c.y].realCenter.y;

  return p;
}

Coor RainerMap::getCellCoor(double px, double py, double th) {
  double d;
  Coor c;
  
  for(int i=0; i<xmax; i++) {
    for (int j=0; j<ymax; j++) {
      d = ArMath::distanceBetween(m[i][j].realCenter.x, m[i][j].realCenter.y, px, py);
      if (d < th) {
	c.x = i;
	c.y = j;
	return c;
      }
    }
  }
  c.x = NULL_COOR;
  c.y = NULL_COOR;
  return c;  
}


void RainerMap::mark(Coor c, State s) {
  m[c.x][c.y].state = s;
}

void RainerMap::markAs(State orig, State final) {
  for (int i = 0; i < xmax; i++) {
    for (int j = 0; j < ymax; j++) {
      if (m[i][j].state == orig) {
	m[i][j].state = final;
      }
    }
  }
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
  printf("\n");
  for (int y = ymax -1; y+1; y--) {
    printf("%d |", y);
    for (int x = 0; x < xmax; x++) {
        printf("%c", charOf(x, y));
    }
    printf("|\n");
  }
  printf("   --------\n");
  printf("   ");
  for (int x = 0; x < xmax; x++) {
      printf("%d", x);
  }
  printf("\n");
}
