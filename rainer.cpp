/* Seguiment de trajectories, exercici 1 de robòtica mòbil */

#include "Aria.h"
#include <stdio.h>
#include <float.h>

#define FRONT_SENS_NUM 8

#define MAX_DIST 500.0
#define TH_HEADING 90
#define TH_ON_POINT 100.0
#define T_BLIND 250

#define GO_GOAL 0
#define AVOID_IMPACT 1

double sonarWeight [] = {0.1, 0.1, 0.1, 0.2, 0.2, 0.1, 0.1, 0.1};
double behaviorWeight [] = {0.3, 0.7};

struct Punt2D {
  double x;
  double y;
};

struct Vect2D {
  double x;
  double y;
};


Vect2D repulsioObstacle(ArRobot *rbt, double th) {
  ArSensorReading *sensor;
  double vx, vy, vux, vuy, xObs, yObs, xRob, yRob, modV;
  Vect2D vr;
  
  vr.x = 0.0;
  vr.y = 0.0;
  
  xRob = rbt->getX();
  yRob = rbt->getY();
  
  for (int i = 0; i < FRONT_SENS_NUM; i++) {
    sensor = rbt->getSonarReading(i);
    xObs = sensor->getX();
    yObs = sensor->getY(); 
    vx = xObs - xRob; 
    vy = yObs - yRob;
    modV = ArMath::distanceBetween(xRob, yRob, xObs, yObs);
    if (modV < th) {
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

  return vr;
}

Vect2D atraccioObjectiu(ArRobot *rbt, Punt2D goal) {
  double vx, vy, d;
  Vect2D va; /* vector atracció */
  
  d = ArMath::distanceBetween(rbt->getX(), rbt->getY(), goal.x, goal.y);
  vx = goal.x - rbt->getX();
  vy = goal.y - rbt->getY();
  
  va.x = vx / d;
  va.y = vy / d;
  
  return va;
}

void anarPunt(ArRobot *rbt, Punt2D pnt) {
  double alpha;
  double d;
  Vect2D vro, va, vd; /* Vector Repulsio Obstacle, Vector Atraccio objectiu, Vector Director */
  
  d = DBL_MAX;
  while (d >= TH_ON_POINT) {
    d = ArMath::distanceBetween(rbt->getX(), rbt->getY(), pnt.x, pnt.y);
        
    va = atraccioObjectiu(rbt, pnt);
    
    vro = repulsioObstacle(rbt, MAX_DIST);

    /* Recalculam el vector director del moviment del robot */
    vd.x = behaviorWeight[GO_GOAL] * va.x + behaviorWeight[AVOID_IMPACT] * vro.x;
    vd.y = behaviorWeight[GO_GOAL] * va.y + behaviorWeight[AVOID_IMPACT] * vro.y;
    
    alpha = ArMath::atan2(vd.y, vd.x);
        
    rbt->setHeading(alpha);
    while (!rbt->isHeadingDone(TH_HEADING)) {
      rbt->setVel(0);
    }   

    rbt->setVel(150);
    ArUtil::sleep(T_BLIND);
  }
}

int main(int argc, char **argv) {
  ArRobot robot3;
  //Declarar la tasca, dins dels constructor ja s'afegeig la tasca
  
  Punt2D punts[] = {{0.0, 0.0}, {0.0, 1000.0}, {1000.0, 0.0}, {1000.0, 1000.0}};

  Aria::init();
  ArSimpleConnector connector(&argc, argv);
  if (!connector.parseArgs() || argc > 1) {
    connector.logOptions();
    exit(1);
  }
  if (!connector.connectRobot(&robot3)) {
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    return 1;
  }
  robot3.comInt(ArCommands::SOUNDTOG, 1);
  robot3.comInt(ArCommands::ENABLE, 1); //Habilitar els motors
  robot3.runAsync(false);
  while (robot3.isRunning()) {
    //Aqui va el vostre programa
    for (int i = 0; i < 4 /*punts.Length*/; i++) {
      printf("Vaig al punt %d\n", i);
      printf("X: %f, Y: %f\n\n", robot3.getX(), robot3.getY());
      anarPunt(&robot3, punts[i]);
    }
  }
  robot3.stopRunning();
  Aria::shutdown();
  return 0;
}


