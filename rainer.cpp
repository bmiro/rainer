/* Seguiment de trajectories, exercici 1 de robòtica mòbil */

#include "Aria.h"
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <map>
#include <string>
#include <fstream>
#include <iostream>

#include "librainer.h"
#include "map.h"

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
  
  for (int i = param["numFirstSonar"]; i < param["numLastSonar"]; i++) {
    snprintf(key, sizeof(key), "%s%d", "weightSonar", i);
    sonarWeight[i] = param[key];
  }
  
  behaviorWeight[0] = param["weightGoalAttraction"];
  behaviorWeight[1] = param["weightObstacleRepulsion"];
  
  return true;
}

int main(int argc, char **argv) {
    
  if (!loadGlobalParams(FILE_PATH)) {
    printf("No s'han pogut carregar els paràmetres de configuració.\n");
    return 1;
  }
  
  map<string, double>::iterator curr,end;
  for (curr = param.begin(); curr != param.end(); curr++) {
      cout << (*curr).first << " " << (*curr).second << endl;
  }

  Rainer rainer(param["thHeading"], param["thOnPoint"], param["maxDist"], param["impactDist"],
  param["blindTime"], param["numSonarFront"], param["numFirstSonar"], param["numLastSonar"],
  param["slowVel"], param["normalVel"], sonarWeight, behaviorWeight);

  Punt2D punts[] = {{0.0, 0.0}, {0.0, 5000.0}, {5000.0, 0.0}, {5000.0, 5000.0}};

  rainer.initArRobot(&argc, argv);
  while (rainer.ar.isRunning()) {
    ;
    ;
  }
  rainer.ar.stopRunning();
  Aria::shutdown();
  return 0;
}


