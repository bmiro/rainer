#include "librainer.h"

Rainer::Rainer(string filename) {

  TactRainer tact(filename);
  exec = &tact.ar;
 
}

void Rainer::cleanArea() {
  Point2D  p;
  Coor c;
  Coor robotCoor;

  /* La primera passada simbolitza la neteja general,
   les seguentes intentes d'anar allà on s'han detectat obstacles. */
  for (int i = 0; i < 2; i++) {
    while (!tact.mp->isClean()) {
      c = tact.mp->getNextPos(DIRTY, tact.ar.getX(), tact.ar.getY());
      p = tact.mp->getRealXY(c);
      
      printf("Vaig a netejar la cel.la a (%f, %f)\n", p.x, p.y);
      if (tact.goGoal(p, tact.getCe())){ //Revisar el segon paràmetre
	tact.mp->mark(c, CLEAN);
      } else {
	tact.mp->mark(c, OBSTACLE);
      }
      
      tact.mp->setRobotPos(c);
      tact.mp->printMap();
    }  
    tact.mp->markAs(OBSTACLE, DIRTY);
  }
  
  printf("\n\n\nMapa net!\n\n\n");
  
}