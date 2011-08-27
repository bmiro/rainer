/* Seguiment de trajectories, exercici 1 de robòtica mòbil */

#include "Aria.h"
#include <stdio.h>
#include <stdlib.h>
#include <float.h>

#include "librainer.h"
#include "lib2d.h"

using namespace std;

double sonarWeight [N_FRONT_SONARS];
double behaviorWeight [BEHAVIOR_FATORS];

int main(int argc, char **argv) {
 
//   map<string, double>::iterator curr,end;
//   for (curr = param.begin(); curr != param.end(); curr++) {
//       cout << (*curr).first << " " << (*curr).second << endl;
//   }

  Rainer rainer(FILE_PATH);                
                
  Point2D punts[4];
  punts[0].setXY(0.0, 0.0);
  punts[1].setXY(0.0, 5000.0);
  punts[2].setXY(5000.0, 0.0);
  punts[3].setXY(5000.0, 5000.0);

  cout << "Hola here\n";
  
  rainer.tact.init(&argc, argv);
  while (rainer.tact.ar.isRunning()) { //TODO mes elegant amb un .exec->isRunning pero no rula joder ostia puta
    //rainer.cleanArea(param["areaXsize"], param["areaYsize"], param["cellEdge"], cr);
    rainer.tact.goGoal(punts[3]);
    rainer.tact.goGoal(punts[2]);
    /*rainer.goGoal(punts[3]);
    rainer.goGoal(punts[0]);*/
  }

  cout << "Goodbye!";
  rainer.exec->stopRunning();
  Aria::shutdown();
  return 0;
}
