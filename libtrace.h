#ifndef LIBTRACE_H
#define LIBTRACE_H

#include <time.h> 
#include <math.h>
#include <stdio.h>

#include "lib2d.h"

class Trace {
private:
  struct traceItem {
    time_t t;
    Point2D p;
  };
  traceItem *t;

  int lastIn;

  int memSize;
  double distanceTh;
  time_t timeTh;
  
  Trace () { };
  
public:
  Trace(int pMemSize, double pDistanceTh, time_t pTimeTh);
  
  bool isInnaccessible();
  bool reset();
  bool add(Point2D p);

};
#endif