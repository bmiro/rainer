#include "librainer.h"

Rainer::Rainer(double pthHeading, double pthOnPoint, double pmaxDist, double pimpactDist,
  double pblindTime, double pnumSonarFront, double pnumFirstSonar, double pnumLastSonar,
  double pnormalVel, double *psonarWeight, double *pbehaviourWeight) {
  
  thHeading = pthHeading;
  thOnPoint = pthOnPoint;
  maxDist = pmaxDist;
  impactDist = pimpactDist;
  
  blindTime = pblindTime;
  
  numSonarFront = pnumSonarFront;
  numFirstSonar = pnumFirstSonar;
  numLastSonar = pnumLastSonar;
  
  normalVel = pnormalVel;
  
  sonarWeight = (double *)malloc(pnumSonarFront*sizeof(double));
  behaviourWeight = (double *)malloc(2*sizeof(double));
  
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

Vect2D Rainer::goalAttraction(Punt2D goal) {
    double vx, vy, d;
    Vect2D va; /* vector atracció */
    
    d = ArMath::distanceBetween(ar.getX(), ar.getY(), goal.x, goal.y);
    vx = goal.x - ar.getX();
    vy = goal.y - ar.getY();
    
    va.x = vx / d;
    va.y = vy / d;
    
    return va;
  }

Vect2D Rainer::obstacleRepulsion(double th, double th_dmin, bool *impactAlert) {
  ArSensorReading *sensor;
  double vx, vy, vux, vuy, xObs, yObs, xRob, yRob, modV;
  double d;
  double dmin = DBL_MAX;
  Vect2D vr;
    
  vr.x = 0.0;
  vr.y = 0.0;
  
  xRob = ar.getX();
  yRob = ar.getY();
  
  for (int i = numFirstSonar; i < numLastSonar; i++) {
    sensor = ar.getSonarReading(i);
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
double Rainer::goGoal(Punt2D pnt) {
  double alpha;
  double d;
  bool impactAlert;
  Vect2D vro, va, vd; /* Vector Repulsio Obstacle, Vector Atraccio objectiu, Vector Director */
  
  printf("%f \n %f \n %f \n %f \n %f \n %f \n %f \n %f \n %f \n ", thHeading, thOnPoint, maxDist, impactDist, blindTime
  , numSonarFront, numFirstSonar, numLastSonar, normalVel);
  
  
  d = DBL_MAX;
  while (d >= thOnPoint) {
    d = ArMath::distanceBetween(ar.getX(), ar.getY(), pnt.x, pnt.y);
        
    vro = obstacleRepulsion(maxDist, impactDist, &impactAlert);
    va = goalAttraction(pnt);
    
    if (impactAlert) {
      /* Col.lisió imminent, sols tenim amb compte el vector de repulsió*/
      vd.x = vro.x;
      vd.y = vro.y;
    } else {
      /* Recalculam el vector director del moviment del robot */
      vd.x = behaviourWeight[BH_GOAL] * va.x + behaviourWeight[BH_OBSTACLE] * vro.x;
      vd.y = behaviourWeight[BH_GOAL] * va.y + behaviourWeight[BH_OBSTACLE] * vro.y;
    }
    
    alpha = ArMath::atan2(vd.y, vd.x);
        
    ar.setHeading(alpha);
    while (!ar.isHeadingDone(thHeading)) {
      ar.setVel(0);
    }   

    ar.setVel(normalVel);
    ArUtil::sleep(blindTime);
  }
  return ArMath::distanceBetween(ar.getX(), ar.getY(), pnt.x, pnt.y);
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