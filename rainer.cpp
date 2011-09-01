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

  Rainer rainer(FILE_PATH);

  rainer.tact.init(&argc, argv);
   
   Point2D punts[4];
   punts[0].setXY(0.0, 0.0);
   punts[1].setXY(0.0, 5000.0);
   punts[2].setXY(5000.0, 0.0);
   punts[3].setXY(5000.0, 5000.0);

  while (rainer.tact.ar.isRunning()) { 
    rainer.cleanArea();
   
//   if (!rainer.tact.goGoal(punts[1], 1000.0)) {
//     printf("No puc arribar a l'objecte");
//   } else {
//     printf("He arribat a %f %f", rainer.tact.ar.getX(), rainer.tact.ar.getY());
//   }
//   rainer.tact.goGoal(punts[2], 1000.0);
//   rainer.tact.goGoal(punts[3], 1000.0);
//   rainer.tact.goGoal(punts[0], 1000.0);
   
//   rainer.tact.wander();
        
   rainer.tact.ar.setVel(0);
   ArUtil::sleep(10000000);
   
  }

  cout << "Goodbye!";
  rainer.exec->stopRunning();
  Aria::shutdown();
  return 0;
}
