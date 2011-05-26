/* Seguiment de trajectories, exercici 1 de robòtica mòbil */

#include "Aria.h"
#include <stdio.h>
#include <float.h>

struct Punt2D {
  float x;
  float y;
};

void anarPunt(ArRobot *rbt, Punt2D pnt) {
  double alpha;
  double dx, dy, d;
  
  d = DBL_MAX;
  while (d >= 5.0) {
    d = ArMath::distanceBetween(rbt->getX(), rbt->getY(), pnt.x, pnt.y);
    
    printf("%f\n", d);
    
    dx = pnt.x - rbt->getX();
    dy = pnt.y - rbt->getY();
    
    alpha = ArMath::atan2(dy, dx);
    
    printf("X: %f, Y: %f\n", rbt->getX(), rbt->getY());
    
    rbt->setHeading(alpha);
    while (!rbt->isHeadingDone(90)) {
      rbt->setVel(0);
    }   

    rbt->setVel(150);
    ArUtil::sleep(500);
  }
}

int main(int argc, char **argv) {
  ArRobot robot3;
  //Declarar la tasca, dins dels constructor ja s'afegeig la tasca
  
  Punt2D punts[] = {{0.0, 0.0}, {0.0, 100.0}, {100.0, 0.0}, {100.0, 100.0}};

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


