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
  
  ArFunctorC<Rainer> myTaskCB;
  
  double cellEdge;
  
public:
  TactRainer tact; /* Accés a nivell tàctic */
  ArRobot *exec; /* Acceés al nivell executiu (punter a ArRobot) */
  
  RainerMap mp;

  Rainer(string filename, int xs, int zs, double ce, Coor robotCoor);
  
  void provaF(void);
  void cleanArea();
   
};
#endif
