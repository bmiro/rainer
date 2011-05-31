#ifndef LIBRAINER_H
#define LIBRAINER_H

#include <math.h>
#include "Aria.h"

#include "common_rainer.h"
#include "lib2d.h"
#include "libtact.h"
#include "libtrace.h"
#include "librainermap.h"

class Rainer {
private: 
  
public:
  TactRainer tact; /* Accés a nivell tàctic */
  ArRobot *exec; /* Acceés al nivell executiu (punter a ArRobot) */
  
  Rainer(double pthHeading, double pthOnPoint, double pmaxDist, double pimpactDist,
  double pblindTime, double pnumSonar, double pnumFirstSonar, double pnumLastSonar,
  double pslowVel, double pnormalVel, double *psonarWeight, double *pbehaviourWeight,
  time_t pTimeObstacledTh, double pDistObstacledTh, int pDlephantMem);
       
  void wander();
  int findObject(double vel, double th);
  void cleanArea(int xs, int zs, double ce, Coor robotCoor);
   
};
#endif
