/* Seguiment de trajectories, exercici 1 de robòtica mòbil */

#include "Aria.h"
#include <stdio.h>
#include <float.h>

#define TH_IMPACT 500.0
#define MAX_DIST 500.0
#define TH_HEADING 90
#define TH_ON_POINT 100.0
#define T_BLIND 250

double sonarWeight [] = {0.1, 0.1, 0.1, 0.2, 0.2, 0.1, 0.1, 0.1};

struct Punt2D {
  double x;
  double y;
};

void repulsioObstacle(ArRobot *rbt, double *x, double *y, double th) {
  ArSensorReading *sensor;
  double vx, vy, ax, ay, vux, vuy, xObs, yObs, xRob, yRob, modV;
  xRob = rbt->getX();
  yRob = rbt->getY();
  
  for (int i = 0; i < rbt->getNumSonar(); i++) {
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
      ax += sonarWeight[i] * vx;
      ay += sonarWeight[i] * vy;
    }
  }
  /* Restam vector de repulsió */
  *x -= ax;
  *y -= ay; 
}

void anarPunt(ArRobot *rbt, Punt2D pnt) {
  double alpha;
  double vx, vy, d;
  
  d = DBL_MAX;
  while (d >= TH_ON_POINT) {
    d = ArMath::distanceBetween(rbt->getX(), rbt->getY(), pnt.x, pnt.y);
    
    printf("%f\n", d);
    
    /* Calculam el vector original */
    vx = pnt.x - rbt->getX();
    vy = pnt.y - rbt->getY();
    
    /* Recalculam el vector segons els obstacles */
    repulsioObstacle(rbt, &vx, &vy, MAX_DIST);
    
    alpha = ArMath::atan2(vy, vx);
    
    printf("X: %f, Y: %f\n", rbt->getX(), rbt->getY());
    
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


