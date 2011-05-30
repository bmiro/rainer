#ifndef LIBTRACE_H
#define LIBTRACE_H

#include <time.h> 
#include "lib2d.h"

class Trace {
private:
  
  struct traceItem {
    time_t t;
    Point2D p;
  }
  
  int memSize;
  double distanceth;
  time_t timeTh;
  
public:
  Trace(int pMemSize, double pDistanceTh, time_t pTimeTh);
  
  bool isInaccessible();
  bool resetTrace();
  bool addToTrace(Point2D p);

}
#endif