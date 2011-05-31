#ifndef LIBRAINER_H
#define LIBRAINER_H

#include <float.h>

#include "Aria.h"

#include "common_rainer.h"
#include "lib2d.h"
#include "libtrace.h"
#include "librainermap.h"
#include "libtactrainer.h"

class Rainer {
private:   
  Rainer() { };
  
public:
  TactRainer tact; /* Robot de l'aria */
  ArRobot *exec;
  
  Rainer(double pthHeading, double pthOnPoint, double pmaxDist, double pimpactDist,
  double pblindTime, double pnumSonarFront, double pnumFirstSonar, double pnumLastSonar,
  double pslowVel, double pnormalVel, double *psonarWeight, double *pbehaviourWeight,
  time_t pTimeObstacledTh, double pDistObstacledTh, int pDlephantMem);
  
  int findObject(double vel, double th); 
  void wander();

	void cleanArea(int xs, int zs, double ce, Coor robotCoor);
   
};
#endif
