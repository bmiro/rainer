#include "librainer.h"

Rainer::Rainer(string filename) {

  TactRainer tact(filename);

  exec = &tact.ar;
}

void Rainer::cleanArea(int xs, int ys, double ce, Coor robotCoor) {
  Point2D robotPoint, p;
  Coor c;

  robotPoint.x = exec->getX();
  robotPoint.y = exec->getY();
  
  RainerMap mp(xs, ys, ce, robotCoor, robotPoint); 
  mp.printMap();
  mp.mark(robotCoor, CLEAN);  
  while (!mp.isClean()) {
    c = mp.getNextPos(CLEAN);
    p = mp.getRealXY(c);
    printf("Vaig a netejar la cel.la a (%f, %f)\n", p.x, p.y);
    tact.goGoal(p, ce); //Revisar el segon paràmetre
    mp.mark(c, CLEAN);
    mp.setRobotPos(c);
    mp.printMap();
  }
}

/* Segueix un contorn a distància D, en tornar passar per un punt on ha estat
* la funció acaba i retorna el mapa de l'objecte */
// bool followCountourn(ArRobot *rbt, double d) {
//   s = findObject(d);
//   as = getSensorAngle(s);
//   
//   wise = (as <= 90.0) ? CLOCKWISE : COUNTERCWISE;
//   
//   /* Orientam  */
//   
//   if (wise == CLOCKWISE) {
//     a = 90.0 - as;
//   } else {
//     a = 90.0 + as;
//   }
// 
//   
//   
// } 
