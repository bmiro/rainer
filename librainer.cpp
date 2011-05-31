#include "librainer.h"

TarctRainer::  TarctRainer(double pthHeading, double pthOnPoint, double pmaxDist, double pimpactDist,
  double pblindTime, double pnumSonarFront, double pnumFirstSonar, double pnumLastSonar,
  double pslowVel, double pnormalVel, double *psonarWeight, double *pbehaviourWeight,
  time_t pTimeObstacledTh, double pDistObstacledTh, int pDlephantMem) {
  
  TactRainer tact(pthHeading, pthOnPoint, pmaxDist, pimpactDist,
                  pblindTime, pnumSonarFront, pnumFirstSonar, pnumLastSonar,
                  pslowVel, pnormalVel, *psonarWeight, *pbehaviourWeight,
                  pTimeObstacledTh, pDistObstacledTh, pDlephantMem);
                  
  exec = tact.&ar;
}

void Rainer::init(int *argc, char **argv) {
  tact.initArRobot(argc, argv); 
}

/* Fa que el robot es mogui fins que es troba a una distancia menor de th. */
int Rainer::findObject(double vel, double th) {
  ArSensorReading *sensor;	
  double xRob, yRob, xObs, yObs, d;
  
  ar.setVel(vel); 
  while (true) {
    xRob = ar.getX();
    yRob = ar.getY(); 
    for (int i = numFirstSonar; i < numLastSonar; i++) {
      sensor = ar.getSonarReading(i);
      xObs = sensor->getX();
      yObs = sensor->getY(); 
      d = ArMath::distanceBetween(xRob, yRob, xObs, yObs);
      if (d < th) {
        ar.setVel(0); 
        return 1;
      }
    }
  }
}

/* El robot es mou fins trobar un objecte. Llavors calcula el vector de 
repulsió per tal d'esquivar l'objecte. Es robot es mou per l'entorn fins 
que es pitja la tecla Esc */
void Rainer::wander() {
  double vel, th, th_dmin, alpha;
  ArKeyHandler keyHandler;
  bool impactAlert;
  Vect2D vro;

  Aria::setKeyHandler(&keyHandler);
  ar.attachKeyHandler(&keyHandler);

  vel = normalVel;
  th = maxDist;
  th_dmin = impactDist;

  alpha = 0.0;

  while (1) {
    //Avançam fins a trobar un objecte
    findObject(vel, th);  

    //Calculam el vector de repulsió i l'angle de gir
    vro = obstacleRepulsion(th, th_dmin, &impactAlert);
    alpha = ArMath::atan2(vro.y, vro.x);

    //Orientam el robot cap a la direcció que ha de seguir		
    ar.setHeading(alpha);
    while (!ar.isHeadingDone(thHeading)) {
      ar.setVel(0);
    }
  }
}

void Rainer::cleanArea(int xs, int ys, double ce, Coor robotCoor) {
  Point2D robotPoint, p;
  Coor c;

  robotPoint.x = ar.getX();
  robotPoint.y = ar.getY();
  
  RainerMap mp(xs, ys, ce, robotCoor, robotPoint); 
  mp.printMap();
  mp.mark(robotCoor, CLEAN);  
  while (!mp.isClean()) {
    c = mp.getNextPos(CLEAN);
    p = mp.getRealXY(c);
    printf("\nVaig al punt real: %f-%f\n", p.x, p.y);
    goGoal(p);
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
