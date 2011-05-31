#include "librainer.h"

Rainer::  Rainer(double pthHeading, double pthOnPoint, double pmaxDist, double pimpactDist,
  double pblindTime, double pnumSonarFront, double pnumFirstSonar, double pnumLastSonar,
  double pslowVel, double pnormalVel, double *psonarWeight, double *pbehaviourWeight,
  time_t pTimeObstacledTh, double pDistObstacledTh, int pDlephantMem) {
  
  thHeading = pthHeading;
  thOnPoint = pthOnPoint;
  maxDist = pmaxDist;
  impactDist = pimpactDist;
  
  blindTime = pblindTime;
  
  numSonarFront = pnumSonarFront;
  numFirstSonar = pnumFirstSonar;
  numLastSonar = pnumLastSonar;
  
  normalVel = pnormalVel;
  slowVel = pslowVel;
  
  timeObstacledTh = pTimeObstacledTh;
  distObstacledTh = pDistObstacledTh;
    
  elephantMem = pDlephantMem;
  
  sonarWeight = new double[(int)pnumSonarFront];
  behaviourWeight = new double[2];
  
  sonarWeight = psonarWeight;
  behaviourWeight = pbehaviourWeight;
}

int Rainer::initArRobot(int *argc, char **argv) {
  Aria::init();
  ArSimpleConnector connector(argc, argv);
  if (!connector.parseArgs() || *argc > 1) {
    connector.logOptions();
    exit(1);
  }
  if (!connector.connectRobot(&ar)) {
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    return 1;
  }
  ar.comInt(ArCommands::SOUNDTOG, 1);
  ar.comInt(ArCommands::ENABLE, 1); //Habilitar els motors
  ar.runAsync(false);
}

Vect2D Rainer::goalAttraction(Point2D goal) {
    Vect2D va(ar.getX(), ar.getY(), goal.x, goal.y)
    return va.norm();   
  }

Vect2D Rainer::obstacleRepulsion(double th, double th_dmin, bool *impactAlert) {
  double mod_vObs [numSensor];
  double di = 0.0;
  Vect2D vObs [numSensor];
  Vect2D vRep (0.0, 0.0);
  ArSensorReading *sensor;
  
  for (int i = firstSensor; i <= lastSensor; i++) {
    sensor = ar.getSonarReading(i);
    di = sensor.getRange();
    if (di < th) {
      vObs[i].setXY(s.getSonarDX, s.getSonarDY);
      mod_vObs[i] = CALC_MOD_VOBS(maxDist, di);
    } else {
      vObs[i].setZero();
      mod_vObs[i] = 0.0;
    }
    vObs += sonarWeight[i] * vRep[i];
  }
  
  *impactAlert = (dmin < th_dmin);

  return vObs.norm();
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

/* Fa que el robot vagi al punt pnt evitant els obstaques que troba. 
 * Retorna:
 *   False si es considera que no es pot arribar al punt 
 *   True si ha pogut arribar al punt
 */
bool Rainer::goGoal(Point2D pnt) {
  double alpha, vel;
  double d;
  bool impactAlert;
  bool canAccess;
  Point2D hereP;
   /* Vector Repulsio Obstacle, Vector Atraccio objectiu, Vector Director */
  Vect2D vro, va, vd;
    
  Trace trace (elephantMem, distObstacledTh, timeObstacledTh);

  d = DBL_MAX;
  canAccess = true;
  while ((d >= thOnPoint) and canAccess) {
    d = ArMath::distanceBetween(ar.getX(), ar.getY(), pnt.x, pnt.y);
    
    vro = obstacleRepulsion(maxDist, impactDist, &impactAlert);
    va = goalAttraction(pnt);
    
    if (impactAlert) {
      /* Col.lisió imminent, sols tenim amb compte el vector de repulsió*/
      vd.x = vro.x;
      vd.y = vro.y;
      vel = slowVel;
      printf("Alerta de col.lisió imminet!!\n");
    } else {
      /* Recalculam el vector director del moviment del robot */
      vd = va * behaviourWeight[BH_GOAL] + vro * behaviourWeight[BH_OBSTACLE];
      
      vel = normalVel;
    }
    
    alpha = ArMath::atan2(vd.y, vd.x);
        
    ar.setHeading(alpha);
    while (!ar.isHeadingDone(thHeading)) {
      ar.setVel(0);
    }   

    ar.setVel(vel);
    ArUtil::sleep(blindTime);

    if (!vro.isZero()) {
      hereP.setXY(ar.getX(), ar.getY());
      trace.add(hereP);
    }
    
    canAccess = not trace.isInnaccessible();   
  }
  return canAccess;
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
