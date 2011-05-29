#ifndef LIBRAINER_H
#define LIBRAINER_H

#include "Aria.h"
#include "common_rainer.h"

#define BH_GOAL 0
#define BH_OBSTACLE 1

class Rainer {
private:
  double thHeading;
  double thOnPoint;
  double maxDist;
  double impactDist;
  
  double blindTime;
  
  int numSonarFront;
  int numFirstSonar;
  int numLastSonar;
  
  double normalVel;
  
  double *sonarWeight;
  double *behaviourWeight;
  
  Rainer() { };
  
public:
  ArRobot ar; /* Robot de l'aria */
  
  Rainer(double pthHeading, double pthOnPoint, double pmaxDist, double pimpactDist,
  double pblindTime, int pnumSonarFront, int pnumFirstSonar, int pnumLastSonar,
  double pnormalVel, double *psonarWeight, double *pbehaviourWeight);
  
  int initArRobot(int *argc, char **argv);
  
  double getThHeading() {return thHeading;}
  double getThOnPoint() {return thOnPoint;}
  double getMaxDist() {return maxDist;}
  double getImpactDist() {return impactDist;}
  
  double getBlindTime() {return blindTime;}
  
  int getNumSonarFront() {return numSonarFront;}
  int getNumFirstSonar() {return numFirstSonar;}
  int getNumLastSonar() {return numLastSonar;}
  
  double getNormalVel() {return normalVel;}
  
  double getSonarWeight(int s) {return sonarWeight[s];}
  double getBehaviorWeight(int behaviour) {return sonarWeight[behaviour];}
  
  Vect2D goalAttraction(Punt2D goal);
  Vect2D obstacleRepulsion(double th, double th_dmin, bool *impactAlert);
  double goGoal(Punt2D pnt);
   
};
#endif