/* Seguiment de trajectories, exercici 1 de robòtica mòbil */

#include "Aria.h"
#include <stdio.h>
#include <stdlib.h>
#include <float.h>

#include "librainer.h"
#include "librainermap.h"
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

  /**************************
  *      Àrea a netejar     *
  *      _ _ _ _ _ _ _ _    *
  *   7 |_|_|_|_|_|_|_|_|   *
  *   6 |_|_|_|_|_|_|_|_|   *
  *   5 |_|_|_|_|_|_|_|_|   *
  *   4 |_|_|_|_|_|_|_|_|   *
  *   3 |_|_|_|>|_|_|_|_|   *
  *   2 |_|_|_|_|_|_|_|_|   *
  *   1 |_|_|_|_|_|_|_|_|   *
  *   0 |_|_|_|_|_|_|_|_|   *
  *      0 1 2 3 4 5 6 7    *
  ***************************/

  Coor cr;
  cr.x = 3;
  cr.y = 3;
  
  /*              
  Point2D punts[4];
  punts[0].setXY(0.0, 0.0);
  punts[1].setXY(0.0, 5000.0);
  punts[2].setXY(5000.0, 0.0);
  punts[3].setXY(5000.0, 5000.0);*/

  cout << "Hola here\n";
  
  rainer.tact.init(&argc, argv);
  while (rainer.tact.ar.isRunning()) { //TODO mes elegant amb un .exec->isRunning pero no rula joder ostia puta
  rainer.cleanArea(8, 8, 500.0, cr);
//     rainer.tact.goGoal(punts[1]);
//     rainer.tact.goGoal(punts[2]);
//     rainer.tact.goGoal(punts[3]);
//     rainer.tact.goGoal(punts[0]);
//    rainer.tact.wander();
  }

  cout << "Goodbye!";
  rainer.exec->stopRunning();
  Aria::shutdown();
  return 0;
}
