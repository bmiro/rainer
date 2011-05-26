/* Seguiment de trajectories, exercici 1 de robòtica mòbil */

#include "Aria.h"

struct Punt2D() {
  x float;
  y float;
};

void anarPunt(ArRobot *rbt, Punt2D pnt) {
  
}

int main(int argc, char **argv) {
  ArRobot robot3;
  //Declarar la tasca, dins dels constructor ja s'afegeig la tasca
  
  Punt2D punts = {{0.0, 0.0}, {0.0, 1000.0}, {1000.0, 0.0}, {1000.0, 1000.0}}

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
    for (i=0; i < punts.Length; i++) {
      anarPunt(punts[i])
    }
  }
  robot3.stopRunning();
  Aria::shutdown();
  return 0;
}


