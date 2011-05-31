#ifndef LIBTACTRAINER_H
#define LIBTACTRAINER_H

#include <float.h>

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
  /* Paràmetres del robot*/
  double thHeading;
  double thOnPoint;
  double maxDist;
  double impactDist;
  
  double blindTime;
  
  double numSonarFront;
  double numFirstSonar;
  double numLastSonar;
  
  double slowVel;
  double normalVel;
  
  time_t timeObstacledTh;
  double distObstacledTh;
  int elephantMem;
  
  double *sonarWeight;
  double *behaviourWeight;
  
  Vect2D goalAttraction(Point2D goal);
  Vect2D obstacleRepulsion(double th, double th_dmin, bool *impactAlert);
   
  Rainer() { };
  
public:
  ArRobot ar; /* Robot de l'aria */
  
  TactRainer(double pthHeading, double pthOnPoint, double pmaxDist, double pimpactDist,
  double pblindTime, double pnumSonarFront, double pnumFirstSonar, double pnumLastSonar,
  double pslowVel, double pnormalVel, double *psonarWeight, double *pbehaviourWeight,
  time_t pTimeObstacledTh, double pDistObstacledTh, int pDlephantMem);
  
  int init(int *argc, char **argv);
 
  int findObject(double vel, double th); 
  void wander();
  bool goGoal(Point2D pnt);

  void cleanArea(int xs, int zs, double ce, Coor robotCoor);
   
};
#endif
