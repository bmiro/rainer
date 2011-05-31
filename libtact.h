#ifndef LIBTACTRAINER_H
#define LIBTACTRAINER_H

#include <math.h>
#include "Aria.h"

#include "common_rainer.h"
#include "lib2d.h"
#include "libtrace.h"

#define BH_GOAL 0
#define BH_OBSTACLE 1

#define CALC_MOD_VOBS(md, d) ((md - d)/md)

class TactRainer {
private:
  /* Paràmetres del robot*/
  double thHeading;
  double thOnPoint;
  double maxDist;
  double impactDist;
  
  double blindTime;
  
  int numSonar;
  int numFirstSonar;
  int numLastSonar;
  
  double slowVel;
  double normalVel;
  
  time_t timeObstacledTh;
  double distObstacledTh;
  int elephantMem;
  
  double *sonarWeight;
  double *behaviourWeight;  
  
public:
  ArRobot ar; /* Robot de l'aria */
  
  //TactRainer();
  TactRainer(double pthHeading=0.0, double pthOnPoint=0.0, double pmaxDist=0.0, double pimpactDist=0.0,
  double pblindTime=0.0, double pnumSonar=0.0, double pnumFirstSonar=0.0, double pnumLastSonar=0.0,
  double pslowVel=0.0, double pnormalVel=0.0, double *psonarWeight=NULL, double *pbehaviourWeight=NULL,
  time_t pTimeObstacledTh=0, double pDistObstacledTh=0.0, int pDlephantMem=0);
  
  int init(int *argc, char **argv);
 
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
  Vect2D obstacleRepulsion(double th, double th_dmin, bool *impactAlert);
  
  bool goGoal(Point2D pnt);
};
#endif
