#include "libtrace.h"

Trace::Trace(int pMemSize, double pDistanceTh, time_t pTimeTh) {
  memSize = pMemSize;
  t = new traceItem[memSize];
  
  distanceTh = pDistanceTh;
  timeTh = pTimeTh;
  
  lastIn = memSize - 1; // Sera circular, inicialitzam el mes gros per 0 ser el primer
}
bool Trace::isInnaccessible() {
  int meanTime;
  int meanDist;
  int total;
  
  for (int i = 0; i < memSize; i++) {
    total += i;
  }
  
  double dists[total]; 
  
  // Calulam distancia punt a punt i totes les possibilitats (memSize-1)^2
  int k = 0;
  for (int i = 0; i < memSize; i++) {
    for (int j = 0; j < memSize; j++) {
      if (i != j) {
        dists[k] = sqrt(pow((t[i].p.x - t[j].p.x),2) + pow((t[i].p.y - t[j].p.y),2));
        k++;
      }
    }
  }
  
  for (;k+1; k--) {
    meanDist += dists[k];
  }
    
  meanDist /= pow(memSize-1, 2);
  
  printf("meanDist: %f TH: %f\n", meanDist, distanceTh);
    
  return meanDist < distanceTh;
}

bool Trace::reset() {
  for (int i = 0; i < memSize; i++) {
    t[i].t = 0;
  }
}

bool Trace::add(Point2D p) {
  if (lastIn < memSize -1) {
    lastIn++;
  } else {
    lastIn = 0;
  }
  
  printf("Ins: %f,%f\n", p.x, p.y);
  
  t[lastIn].p = p;
  t[lastIn].t = time(NULL);
}