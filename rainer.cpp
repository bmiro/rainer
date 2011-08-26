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
#include "lib2d.h"

map<string, double> param;

using namespace std;

double sonarWeight [N_FRONT_SONARS];
double behaviorWeight [BEHAVIOR_FATORS];

/* Rutina local per la lectura dels paràmetres del fitxer */
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
                param["blindTime"], param["numSonar"], param["numFirstSonar"], param["numLastSonar"],
                param["slowVel"], param["normalVel"], sonarWeight, behaviorWeight,
                param["timeObstacledTh"], param["distObstacledTh"], param["elephantMem"]);                
                
  Point2D punts[4];
  punts[0].setXY(0.0, 0.0);
  punts[1].setXY(0.0, 5000.0);
  punts[2].setXY(5000.0, 0.0);
  punts[3].setXY(5000.0, 5000.0);

  cout << "Hola here\n";
  
  rainer.tact.init(&argc, argv);
  while (rainer.tact.ar.isRunning()) { //TODO mes elegant amb un .exec->isRunning pero no rula joder ostia puta
    //rainer.cleanArea(param["areaXsize"], param["areaYsize"], param["cellEdge"], cr);
    rainer.tact.goGoal(punts[1]);
    rainer.tact.goGoal(punts[2]);
    /*rainer.goGoal(punts[3]);
    rainer.goGoal(punts[0]);*/
  }

  cout << "Goodbye!";
  rainer.exec->stopRunning();
  Aria::shutdown();
  return 0;
}
