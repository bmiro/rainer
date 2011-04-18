/*
   Programa d'exemple de connexió amb el robot.
   A més a més, s'ha afegit una tasca periodica. Veure updatingTask.cpp
*/

#include "Aria.h"

int main(int argc, char **argv) {

  ArRobot robot3;

  //Declarar la tasca, dins dels constructor ja s'afegeig la tasca

  Aria::init();
 
  
  ArSimpleConnector connector(&argc, argv);

  //Testejar els arguments del programa
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
    robot3.move(1000.0);
  }

  robot3.stopRunning();
  Aria::shutdown();
  return 0;
}

