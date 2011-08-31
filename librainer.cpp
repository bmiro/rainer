#include "librainer.h"

void Rainer::provaF() {
//   printf("\n\nProoovaaaaaaaaaaaaaaaaaaaaaaa\n\n");
//   Coor c;
//   c.x = 0;
//   c.y = 0;
//   mp.mark(c, DIRTY);
}

Rainer::Rainer(string filename, int xs, int ys, double ce, Coor robotCoor) :
  myTaskCB(this, &Rainer::provaF) {

  Point2D robotPoint;
    
  TactRainer tact(filename);
  exec = &tact.ar;
  
  robotPoint.x = 0.0;
  robotPoint.y = 0.0;
  RainerMap mp(xs, ys, ce, robotCoor, robotPoint); 

   exec->addUserTask("Prova", 0, &myTaskCB);

}

void Rainer::cleanArea() {
  Point2D p;
  Coor c;

  mp.printMap();
//   mp.mark(robotCoor, CLEAN);  
  while (!mp.isClean()) {
    c = mp.getNextPos(CLEAN);
    p = mp.getRealXY(c);
    
    printf("Vaig a netejar la cel.la a (%f, %f)\n", p.x, p.y);
    if (tact.goGoal(p, cellEdge)){ //Revisar el segon paràmetre
      mp.mark(c, CLEAN);
    } else {
      mp.mark(c, OBSTACLE);
    }
    
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
