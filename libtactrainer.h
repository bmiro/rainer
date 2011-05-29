#include <map>
#include <string>
#include "Aria.h"

#include "common_rainer.h"

#define BEH_GOAL_ATTR 0
#define BEH_OBST_REPL 1

using namespace std;

//void findObject(ArRobot rbt, double vel, double th);
//void followContourn(ArRobot rbt, double vel, double d);
Vect2D goalAttraction(ArRobot *rbt, Punt2D goal);
Vect2D obstacleRepulsion(ArRobot *rbt, double th, double th_dmin, int firstSonar, int lastSonar, 
                        double sonarWeight[], bool *impactAlert);
double goGoal(ArRobot *rbt, map<string, double> param, double *sonarWeight, Punt2D pnt);
  
