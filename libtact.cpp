#include "libtact.h"

TactRainer::TactRainer(string filename) {
  
  char key[MAX_PARAM_LINE];

  if (!loadGlobalParams(filename)) {
    printf("No s'han pogut carregar els paràmetres de configuració.\n");
    //return 1; Aixecar escepcio
  } else {
    printf("Parametres llegits del fitxer %s", (char *)filename.c_str());
  }
  
  thHeading = param["thHeading"];
  thOnPoint = param["thOnPoint"];
  maxDist = param["maxDist"];
  impactDist = param["impactDist"];
  
  blindTime = param["blindTime"];
  
  numSonar = (int)param["numSonar"];
  numFirstSonar = (int)param["numFirstSonar"];
  numLastSonar = (int)param["numLastSonar"];
  
  normalVel = param["normalVel"];
  slowVel = param["slowVel"];
  
  timeObstacledTh = (time_t)param["timeObstacledTh"];
  distObstacledTh = param["distObstacledTh"];
    
  elephantMem = (int)param["elephantMem"];
  
  sonarWeight = new double[numSonar];
  behaviourWeight = new double[2];
  
  for (int i = param["numFirstSonar"]; i < param["numLastSonar"]; i++) {
    snprintf(key, sizeof(key), "%s%d", "weightSonar", i);
    sonarWeight[i] = param[key];
  }
  
  behaviourWeight[0] = param["weightGoalAttraction"];
  behaviourWeight[1] = param["weightObstacleRepulsion"];
    
}

/* Rutina local per la lectura dels paràmetres del fitxer */
bool TactRainer::loadGlobalParams(string filename) {
  string line, id, value;
  size_t pos;
  
  ifstream f (filename.c_str());
  
  if (!f.is_open()) return false;
  
  while (f.good()) {
    getline(f, line);
    if (!line.empty()) {
      pos = line.find(" ");
      id = line.substr(0, pos);
      value = line.substr(pos);
      param[id] = strtod(value.c_str(), NULL);
    }
  }
  f.close();
  
  return true;
}

int TactRainer::init(int *argc, char **argv) {
  Aria::init();
  
  printf("TactInit!\n");
  
  for (int i = 0; i < 4; i++) {
    printf("%f\n", sonarWeight[i]);
  }
  
  
  ArSimpleConnector connector(argc, argv);
  if (!connector.parseArgs() || *argc > 1) {
    connector.logOptions();
    exit(1);
  }
  if (!connector.connectRobot(&ar)) {
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    return 1;
  }
 
  ar.comInt(ArCommands::SOUNDTOG, 1);
  ar.comInt(ArCommands::ENABLE, 1); //Habilitar els motors
  ar.runAsync(false);
  return 0;
}

Vect2D TactRainer::goalAttraction(Point2D goal) {
  Vect2D va(ar.getX(), ar.getY(), goal.x, goal.y);
  return va.norm();   
}

Vect2D TactRainer::obstacleRepulsion(double th, double th_dmin, bool *impactAlert) {
  double mod_vObs [numSonar]; //TODO revisar perque s'ha de emprar
  double di = 0.0;
  double dmin = DBL_MAX;
  Vect2D vObs [numSonar]; 
  Vect2D vRep (0.0, 0.0);
  ArSensorReading *sensor;
  
  for (int i = numFirstSonar; i <= numLastSonar; i++) {
    sensor = ar.getSonarReading(i);
    di = (double)sensor->getRange();
    
    if (di < dmin) {
      dmin = di;
    }

    if (di < th) {
      vObs[i].setXY(sensor->getSensorDX(), sensor->getSensorDY());
      mod_vObs[i] = CALC_MOD_VOBS(maxDist, di);
    } else {
      vObs[i].setZero();
      mod_vObs[i] = 0.0;
    }
        
    vRep += vObs[i] * sonarWeight[i];
  }
  
  *impactAlert = (dmin < th_dmin);
  
  return vRep.norm();
}

double TactRainer::getThHeading(double alpha) {
  return thHeading; //TODO en funcio de alpha
}

double TactRainer::getThOnPoint() {
  return thOnPoint;
}
double TactRainer::getMaxDist() {
  return maxDist;
}

double TactRainer::getImpactDist() {
  return impactDist;
}

double TactRainer::getBlindTime() {
  return blindTime;
}

int TactRainer::getNumSonar() {
  return numSonar;
}

int TactRainer::getNumFirstSonar() {
  return numFirstSonar;
}
int TactRainer::getNumLastSonar() {
  return numLastSonar;
}

double TactRainer::getSlowVel() {
  return slowVel;
}
double TactRainer::getNormalVel() {
  return normalVel;
}

/* Fa que el robot vagi al punt pnt evitant els obstaques que troba. 
 * Retorna:
 *   False si es considera que no es pot arribar al punt 
 *   True si ha pogut arribar al punt
 */
bool TactRainer::goGoal(Point2D pnt) {
  double alpha, vel;
  double d;
  bool impactAlert;
  bool canAccess;
  Point2D hereP;
   /* Vector Repulsio Obstacle, Vector Atraccio objectiu, Vector Director */
  Vect2D vro, va, vd;
    
  //Trace trace (elephantMem, distObstacledTh, timeObstacledTh); // TODO

  printf("Vaig al punt %f, %f\n", pnt.x, pnt.y);
  
  d = DBL_MAX;
  canAccess = true;
  while ((d >= thOnPoint) and canAccess) {
    
    printf("aaaaaaaaaaa");
    
    d = ArMath::distanceBetween(ar.getX(), ar.getY(), pnt.x, pnt.y);
    
    printf("bbbbbbbbb");

    
    vro = obstacleRepulsion(maxDist, impactDist, &impactAlert);
    
    printf("ccccccccc");
    
    va = goalAttraction(pnt);
    
    if (impactAlert) {
      /* Col.lisió imminent, sols tenim amb compte el vector de repulsió*/
      vd.x = vro.x;
      vd.y = vro.y;
      vel = slowVel;
      printf("Alerta de col.lisió imminet!!\n");
    } else {
      /* Recalculam el vector director del moviment del robot */
      vd = (va * behaviourWeight[BH_GOAL]) + (vro * behaviourWeight[BH_OBSTACLE]);
      
      vel = normalVel;
    }
    
    printf("zzzzzzzzzzzzzzzzzzz");
    
    alpha = ArMath::atan2(vd.y, vd.x);
        
    ar.setHeading(alpha);
    while (!ar.isHeadingDone(thHeading)) {
      ar.setVel(0);
    }   

    ar.setVel(vel);
    ArUtil::sleep(blindTime);

    if (!vro.isZero()) {
      hereP.setXY(ar.getX(), ar.getY());
      //trace.add(hereP);// TODO
    }
    
    //canAccess = not trace.isInnaccessible(); // TODO   
  }
  return canAccess;
}

