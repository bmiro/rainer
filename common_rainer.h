#include <map>
#include <string>

#define CLOCKWISE 0
#define COUNTERCWISE 1

#define FILE_PATH "config.cfg"
#define MAX_PARAM_NAME 25
#define MAX_PARAM_LINE 80
#define N_FRONT_SONARS 8
#define BEHAVIOR_FATORS 2
#define DELIM ' '
#define COMMENT '#'

#define DEBUG 1

using namespace std;

#ifndef COOR_STRUCT
#define COOR_STRUCT
struct Coor {
  int x, y;
};
#endif


//map<string, double> param;
