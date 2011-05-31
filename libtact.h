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
  
  TactRainer();
  TactRainer(double pthHeading, double pthOnPoint, double pmaxDist, double pimpactDist,
  double pblindTime, double pnumSonar, double pnumFirstSonar, double pnumLastSonar,
  double pslowVel, double pnormalVel, double *psonarWeight, double *pbehaviourWeight,
  time_t pTimeObstacledTh, double pDistObstacledTh, int pDlephantMem);
  
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
