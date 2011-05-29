#include "libtactrainer.h"

Vect2D goalAttraction(ArRobot *rbt, Punt2D goal) {
  double vx, vy, d;
  Vect2D va; /* vector atracció */
  
  d = ArMath::distanceBetween(rbt->getX(), rbt->getY(), goal.x, goal.y);
  vx = goal.x - rbt->getX();
  vy = goal.y - rbt->getY();
  
  va.x = vx / d;
  va.y = vy / d;
  
  return va;
}

Vect2D obstacleRepulsion(ArRobot *rbt, double th, double th_dmin, int firstSonar, int lastSonar, 
                         double sonarWeight[], bool *impactAlert) {
  ArSensorReading *sensor;
  double vx, vy, vux, vuy, xObs, yObs, xRob, yRob, modV;
  double d;
  double dmin = DBL_MAX;
  Vect2D vr;
    
  vr.x = 0.0;
  vr.y = 0.0;
  
  xRob = rbt->getX();
  yRob = rbt->getY();
  
  for (int i = firstSonar; i < lastSonar; i++) {
    sensor = rbt->getSonarReading(i);
    xObs = sensor->getX();
    yObs = sensor->getY(); 
    vx = xObs - xRob; 
    vy = yObs - yRob;
    d = ArMath::distanceBetween(xRob, yRob, xObs, yObs);
    
    if (d < dmin) {
      dmin = d;
    }
    
    if (d < th) {
      modV = (th - d) / th;
      /* Normalitzam el vector generat per aquest sensor */
      vux = vx / modV; //TODO alerta possible divisió per 0 que ha de ser controlada
      vux = vy / modV;
      /* Poderam i acumulam al vector de repulsió */
      vr.x += sonarWeight[i] * vx;
      vr.y += sonarWeight[i] * vy;
    }
  }
  /* Feim que sigui un vector negatiu */
  vr.x *= -1;
  vr.y *= -1;

  *impactAlert = (dmin < th_dmin);

  return vr;
}

/* Retorna distancia a que s'ha quedat del punt */
double goGoal(ArRobot *rbt, map<string, double> param, double *sonarWeight, Punt2D pnt) {
  double alpha;
  double d;
  bool impactAlert;
  Vect2D vro, va, vd; /* Vector Repulsio Obstacle, Vector Atraccio objectiu, Vector Director */
  
  d = DBL_MAX;
  while (d >= param["thOnPoint"]) {
    d = ArMath::distanceBetween(rbt->getX(), rbt->getY(), pnt.x, pnt.y);
        
    vro = obstacleRepulsion(rbt, param["maxDist"], param["impactDist"], param["numFirstSensor"],
                            param["numLastSensor"], sonarWeight, &impactAlert);
    va = goalAttraction(rbt, pnt);
    
    if (impactAlert) {
      /* Col.lisió imminent, sols tenim amb compte el vector de repulsió*/
      vd.x = vro.x;
      vd.y = vro.y;
    } else {
      /* Recalculam el vector director del moviment del robot */
      vd.x = param["weightGoalAttraction"] * va.x + param["weightObstacleRepulsion"] * vro.x;
      vd.y = param["weightGoalAttraction"] * va.y + param["weightObstacleRepulsion"] * vro.y;
    }
    
    alpha = ArMath::atan2(vd.y, vd.x);
        
    rbt->setHeading(alpha);
    while (!rbt->isHeadingDone(param["thHeading"])) {
      rbt->setVel(0);
    }   

    rbt->setVel(param["normalVel"]);
    ArUtil::sleep(param["blindTime"]);
  }
  return ArMath::distanceBetween(rbt->getX(), rbt->getY(), pnt.x, pnt.y);
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