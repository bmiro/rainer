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
  
  double numSonarFront;
  double numFirstSonar;
  double numLastSonar;
  
	double slowVel;
  double normalVel;
  
  double *sonarWeight;
  double *behaviourWeight;
  
  Rainer() { };
  
public:
  ArRobot ar; /* Robot de l'aria */
  
  Rainer(double pthHeading, double pthOnPoint, double pmaxDist, double pimpactDist,
  double pblindTime, double pnumSonarFront, double pnumFirstSonar, double pnumLastSonar,
  double pslowVel, double pnormalVel, double *psonarWeight, double *pbehaviourWeight);
  
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
  
  Vect2D goalAttraction(Point2D goal);
  Vect2D obstacleRepulsion(double th, double th_dmin, bool *impactAlert);
	int findObject(double vel, double th); 
	void wander();
  double goGoal(Point2D pnt);
   
};
#endif
