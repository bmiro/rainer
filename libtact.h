#ifndef LIBTACTRAINER_H
#define LIBTACTRAINER_H

#include <math.h>
#include <map>
#include <string>
#include <fstream>
#include <iostream>

#include "Aria.h"

#include "common_rainer.h"
#include "lib2d.h"
#include "libtrace.h"
#include "librainermap.h"

#define BH_GOAL 0
#define BH_OBSTACLE 1

#define CALC_MOD_VOBS(md, d) ((md - d)/md)

class TactRainer {
private:
  bool loadGlobalParams(string filename);
  
  /* Paràmetres del robot*/
  double thHeading;
  double thObsHeading;
  double dr;
  double thOnPoint;
  double maxDist;
  double impactDist;
  
  double blindTime;
  
  int numSonar;
  int numFirstSonar;
  int numLastFrontSonar;
  int numLastSonar;
  
  double slowVel;
  double normalVel;
  
  time_t timeObstacledTh;
  double distObstacledTh;
  int elephantMem;
  
  double *sonarWeight;
  double *behaviourWeight;  
  
  map<string, double> param;
  
  double currHeading;
  
  double cellEdge;
  int areaXsize;
  int areaYsize;
  int robotXinit;
  int robotYinit;
  
  ArFunctorC<TactRainer> routeTask;
  ArFunctorC<TactRainer> seenTask;
  
public:
  ArRobot ar; /* Robot de l'aria */
  RainerMap *mp;

  
  //TactRainer();
  TactRainer(string filename=FILE_PATH);
  
  int init(int *argc, char **argv);
  
  void ivSeen();
  void routeDone();
 
  double getThHeading(double alpha);
  double getThOnPoint();
  double getMaxDist();
  double getImpactDist();
  
  double getBlindTime();
  
  int getNumSonar();
  int getNumFirstSonar();
  int getNumLastSonar();
  
  double getSlowVel();
  double getNormalVel();
  
  Vect2D goalAttraction(Point2D goal);
  Vect2D obstacleRepulsion(double th, double th_dmin,
				     bool *obstacle, bool *impactAlert, Point2D *nearObstaclePoint);
  
  int findObject(double vel, double th);
  void wander();

  bool goGoal(Point2D pnt, double obsRadius);
  
    /* Getters de parametres del mapa, no intrinsecament lligats amb el tactic */
  double getCrX();
  double getCrY();
  int getSizeX();
  int getSizeY();
  double getCe();
};
#endif
