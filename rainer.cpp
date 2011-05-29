/* Seguiment de trajectories, exercici 1 de robòtica mòbil */

#include "Aria.h"
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <map>
#include <string>
#include <fstream>
#include <iostream>

#include "libtactrainer.h"

map<string, double> param;

using namespace std;

double sonarWeight [N_FRONT_SONARS];
double behaviorWeight [BEHAVIOR_FATORS];

bool loadGlobalParams(string filename) {
  string line, id, value;
  char key[MAX_PARAM_LINE];
  size_t pos;
  
  ifstream f (filename.c_str());
  
  if (!f.is_open()) return false;
  
  while (f.good()) {
    getline(f, line);
    if (!line.empty()) {
      pos = line.find(" ");
      id = line.substr(0, pos);
      value = line.substr(pos);
      param[id] = strtod(value.c_str(), NULL);
    }
  }
  f.close();
  
  for (int i = param["numFirstSensor"]; i < param["numLastSensor"]; i++) {
    snprintf(key, sizeof(key), "%s%d", "weightSonar", i);
    sonarWeight[i] = param[key];
  }
  
  return true;
}

int main(int argc, char **argv) {
  ArRobot robot3;
  //Declarar la tasca, dins dels constructor ja s'afegeig la tasca
  
  if (!loadGlobalParams(FILE_PATH)) {
    printf("No s'han pogut carregar els paràmetres de configuració.\n");
    return 1;
  }
  
  map<string, double>::iterator curr,end;
  for (curr = param.begin(); curr != param.end(); curr++) {
      cout << (*curr).first << " " << (*curr).second << endl;
  }

  Punt2D punts[] = {{0.0, 0.0}, {0.0, 5000.0}, {5000.0, 0.0}, {5000.0, 5000.0}};

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
      goGoal(&robot3, param, sonarWeight, punts[i]);
    }
  }
  robot3.stopRunning();
  Aria::shutdown();
  return 0;
}


