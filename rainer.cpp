/* Seguiment de trajectories, exercici 1 de robòtica mòbil */

#include "Aria.h"
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <map>
#include <string>
#include <fstream>
#include <iostream>

using namespace std;

#define FILE_PATH "config.cfg"
#define MAX_PARAM_NAME 25
#define MAX_PARAM_LINE 80
#define DELIM ' '
#define COMMENT '#'

double sonarWeight [];
double behaviorWeight [2];

map<string, double> macro;

struct Punt2D {
  double x;
  double y;
};

struct Vect2D {
  double x;
  double y;
};

bool loadGlobalParams(string filename) {
  string line, id, value;
  size_t pos;
  
  ifstream f (filename.c_str());
  
  if (!f.is_open()) return false;
  
  while (f.good()) {
    getline(f, line);
    if (!line.empty()) {
      pos = line.find(" ");
      id = line.substr(0, pos);
      value = line.substr(pos);
      macro[id] = strtod(value.c_str(), NULL);
    }
  }
  f.close();
  return true;
}

Vect2D repulsioObstacle(ArRobot *rbt, double th, double th_dmin , bool *impactAlert) {
  ArSensorReading *sensor;
  double vx, vy, vux, vuy, xObs, yObs, xRob, yRob, modV;
  double d;
  double dmin = DBL_MAX;
  Vect2D vr;
    
  vr.x = 0.0;
  vr.y = 0.0;
  
  xRob = rbt->getX();
  yRob = rbt->getY();
  
  for (int i = macro["numFirstSensor"]; i < macro["numLastSensor"]; i++) {
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
  bool impactAlert;
  Vect2D vro, va, vd; /* Vector Repulsio Obstacle, Vector Atraccio objectiu, Vector Director */
  
  d = DBL_MAX;
  while (d >= macro["thOnPoint"]) {
    d = ArMath::distanceBetween(rbt->getX(), rbt->getY(), pnt.x, pnt.y);
        
    vro = repulsioObstacle(rbt, macro["maxDist"], macro["impactDist"], &impactAlert);
    va = atraccioObjectiu(rbt, pnt);
    
    if (impactAlert) {
      vd.x = vro.x;
      vd.y = vro.y;
    } else {
      /* Recalculam el vector director del moviment del robot */
      vd.x = macro["weightGoalAttraction"] * va.x + macro["weightObstacleRepulsion"] * vro.x;
      vd.y = macro["weightGoalAttraction"] * va.y + macro["weightObstacleRepulsion"] * vro.y;
    }
    
    alpha = ArMath::atan2(vd.y, vd.x);
        
    rbt->setHeading(alpha);
    while (!rbt->isHeadingDone(macro["thHeading"])) {
      rbt->setVel(0);
    }   

    rbt->setVel(150);
    ArUtil::sleep(macro["blindTime"]);
  }
}

int main(int argc, char **argv) {
  ArRobot robot3;
  //Declarar la tasca, dins dels constructor ja s'afegeig la tasca
  
  if (!loadGlobalParams(FILE_PATH)) {
    printf("No s'han pogut carregar els paràmetres de configuració.\n");
    return 1;
  }
  
  map<string, double>::iterator curr,end;
  for (curr = macro.begin(); curr != macro.end(); curr++) {
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
      anarPunt(&robot3, punts[i]);
    }
  }
  robot3.stopRunning();
  Aria::shutdown();
  return 0;
}


