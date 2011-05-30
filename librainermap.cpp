#include "librainermap.h"

RainerMap::RainerMap (int sizex, int sizey, double cellEdge, Coor rc, Point2D robotPoint) {
	maxX = sizex; 
	maxY = sizey;
	robotCoor.x = rc.x;
	robotCoor.y = rc.y;
	
	for (int i = 0; i < maxX; i++) {
		for (int j = 0; j < maxY; j++) {
			m[i][j].state = DIRTY;		
			m[i][j].realCenter.x = robotPoint.x + (i - robotCoor.x) * cellEdge;
			m[i][j].realCenter.y = robotPoint.y + (j - robotCoor.y) * cellEdge;
		}
	}
}

//void RainerMap::changeOrigen(Coor newCoorCenter, Point2D newPointCenter) {}
  
void RainerMap::setRobotPos(Coor c) {
	robotCoor.x = c.x;
	robotCoor.y = c.y;
}

Coor RainerMap::getNextPos(State s) {
	Coor c; 

	if (robotCoor.x < maxX) {
		c.x = robotCoor.x + 1;
		c.y = robotCoor.y;
	} else if (robotCoor.y < maxY) {
		c.x = 0;
		c.y = robotCoor.y + 1;
	} else {
		c.x = 0;
		c.y = 0;	
	}

	return c;
	
}

//Coor RainerMap::whichCell(Point2D p) {}

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
	for (int i = 0; i < maxX; i++) {
		for (int j = 0; j < maxY; j++) {
			if (m[i][j].state = DIRTY) {
				return false;			
			} 	
		}	
	}
	return true;
}

bool RainerMap::isAbsolutelyClean() {
	for (int i = 0; i < maxX; i++) {
		for (int j = 0; j < maxY; j++) {
			if (m[i][j].state != CLEAN) {
				return false;			
			} 	
		}	
	}
	return true;
}
