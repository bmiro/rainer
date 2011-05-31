#include "libtrace.h"

Trace::Trace(int pMemSize, double pDistanceTh, time_t pTimeTh) {
  memSize = pMemSize;
  t = new traceItem[memSize];
  
  distanceTh = pDistanceTh;
  
  printf("lbtDth: %f\n", distanceTh);
  
  timeTh = pTimeTh;
  
  lastIn = memSize - 1; // Sera circular, inicialitzam el mes gros per 0 ser el primer
}
bool Trace::isInnaccessible() {
  int meanTime;
  int meanDist;
  int total;
  int k;

  printf("isInnaDth: %f\n", distanceTh);

  total = 0;
  for (int i = 0; i < memSize; i++) {
    total += i;
  }
  
  double dists[total]; 
  
  // Calulam distancia punt a punt i totes les possibilitats (memSize-1)^2
  k = 0;
  for (int i = 0; i < memSize -1; i++) {
    for (int j = i+1; j < memSize; j++) {
      dists[k] = sqrt(pow((t[i].p.x - t[j].p.x),2) + pow((t[i].p.y - t[j].p.y),2));
      k++;
    }
  }
  
  printf("total %d, k %d", total, k);
  
  for (k = 0; k < total; k++) {
    meanDist += dists[k];
  }
    
  //meanDist /= pow(memSize-1, 2);
  
  printf("meanDist: %f TH: %f\n", meanDist, distanceTh);
    
  return meanDist < distanceTh;
}

bool Trace::reset() {
  for (int i = 0; i < memSize; i++) {
    t[i].t = 0;
  }
}

bool Trace::add(Point2D p) {
  
  //TODO nomes ficar si ha passat thershold de temps
  if (lastIn < memSize -1) {
    lastIn++;
  } else {
    lastIn = 0;
  }
  
  printf("Ins: %f,%f\n", p.x, p.y);
  
  t[lastIn].p = p;
  t[lastIn].t = time(NULL);
}