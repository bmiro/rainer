#include "libtact.h"

TactRainer::TactRainer(string filename) :
  routeTask(this, &TactRainer::routeDone),
  seenTask(this, &TactRainer::ivSeen) 
  {
  
  char key[MAX_PARAM_LINE];

  ar.addUserTask("routedone", 1, &routeTask);
  ar.addUserTask("ivseen", 2, &seenTask);
  
  if (!loadGlobalParams(filename)) {
    printf("No s'han pogut carregar els paràmetres de configuració.\n");
  } else {
    printf("Parametres llegits del fitxer %s", (char *)filename.c_str());
  }
  
  thHeading = param["thHeading"];
  thObsHeading = param["thObsHeading"];
  thOnPoint = param["thOnPoint"];
  maxDist = param["maxDist"];
  impactDist = param["impactDist"];
  
  blindTime = param["blindTime"];
  
  numSonar = (int)param["numSonar"];
  numFirstSonar = (int)param["numFirstSonar"];
  numLastFrontSonar = (int)param["numLastFrontSonar"];
  numLastSonar = (int)param["numLastSonar"];
  
  normalVel = param["normalVel"];
  slowVel = param["slowVel"];
  
  timeObstacledTh = (time_t)param["timeObstacledTh"];
  distObstacledTh = param["distObstacledTh"];
  
  dr = param["dr"];
    
  elephantMem = (int)param["elephantMem"];
  
  sonarWeight = new double[numSonar];
  behaviourWeight = new double[2];
  
  for (int i = param["numFirstSonar"]; i < param["numLastSonar"]; i++) {
    snprintf(key, sizeof(key), "%s%d", "weightSonar", i);
    sonarWeight[i] = param[key];
  }
  
  behaviourWeight[0] = param["weightGoalAttraction"];
  behaviourWeight[1] = param["weightObstacleRepulsion"];
  
  cellEdge = param["cellEdge"];
  areaXsize = param["areaXsize"];
  areaYsize = param["areaYsize"];
  robotXinit = param["robotXinit"];
  robotYinit = param["robotYinit"];
  
  mp = new RainerMap(areaXsize, areaYsize, cellEdge, robotXinit, robotYinit, 0.0, 0.0);

}

void TactRainer::ivSeen() {
  double di;
  ArSensorReading *sensor;
  Coor c;

  for (int i = numFirstSonar; i <= numLastSonar; i++) {
    sensor = ar.getSonarReading(i);
    di = (double)sensor->getRange();
    if (di < getMaxDist()) {
      c = mp->getCellCoor(sensor->getX(), sensor->getY(), cellEdge*0.7);
    }
  }
  
  //Escriure resultats a mapa de bits o fitxer
  
}

/* Va marcant els punts per on ja s'ha passat */
void TactRainer::routeDone() {
  Coor c;
  
  /* Marcam per on hem passat */
  c = mp->getCellCoor(ar.getX(), ar.getY(), cellEdge/2);
  if (c.x != NULL_COOR) {
    mp->mark(c, CLEAN);
    mp->setRobotPos(c);
  }
  
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
  ar.comInt(ArCommands::ENABLE, 1);
  ar.runAsync(false);
  return 0;
}

Vect2D TactRainer::goalAttraction(Point2D goal) {
  Vect2D va(goal.x, goal.y, ar.getX(), ar.getY());  
  return va.norm();   
}

Vect2D TactRainer::obstacleRepulsion(double th, double th_dmin,
				     bool *obstacle=NULL, bool *impactAlert=NULL,
				     Point2D *nearObstaclePoint=NULL) {
  double mod_vObs [numSonar];
  double di = 0.0;
  double dmin = DBL_MAX;
  Vect2D vObs [numSonar]; 
  Vect2D vRep (0.0, 0.0);
  ArSensorReading *sensor;
  
  if (nearObstaclePoint != NULL) {
    nearObstaclePoint->x = DBL_MAX;
    nearObstaclePoint->y = DBL_MAX;
  }
  if (obstacle != NULL) {
    *obstacle = false;
  }
  
  /* Calcul de la repulsió dels sensors de davant */
  for (int i = numFirstSonar; i <= numLastSonar; i++) {
    sensor = ar.getSonarReading(i);
    di = (double)sensor->getRange();
    
    if (di < th) {
      if (obstacle != NULL) {
	*obstacle = true;
      }
      vObs[i].setXY(sensor->getX() - sensor->getXTaken(),
                    sensor->getY() - sensor->getYTaken());
      vObs[i] *= -1;
      mod_vObs[i] = CALC_MOD_VOBS(maxDist, di);
       /* Convertim el vector en un que tingui el mòdul (maxdist - di)/ maxdist */
      vObs[i] = vObs[i].norm(mod_vObs[i]);
    } else {
      vObs[i].setZero();
    }
        
    vRep += vObs[i] * sonarWeight[i];
    
    /* Guarda la més petita de totes les distàncies */
    /* Si es el sensor que detecta la distància mínima es té doblement amb compte */
    if (di < dmin) {
      dmin = di;
      if (nearObstaclePoint != NULL) {
	nearObstaclePoint->x = sensor->getX();
	nearObstaclePoint->y = sensor->getY();
      }
      vRep += vObs[i] * sonarWeight[i];
    }
        
  }  
  if (impactAlert != NULL) {
    *impactAlert = (dmin < th_dmin);
  }
  
  return vRep.norm();
}

double TactRainer::getThHeading(double alpha) {
  return thHeading;
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

/* Fa que el robot es mogui fins que es troba a una distancia menor de th. */
int TactRainer::findObject(double vel, double th) {
  ArSensorReading *sensor;	
  double d;
  
  ar.setVel(vel); 
  
  while (true) {
    for (int i = numFirstSonar; i <= numLastFrontSonar; i++) {
      sensor = ar.getSonarReading(i);
      d = (double)sensor->getRange();
            
      if (d < th) {
	return 1;
      }
    }
  }
}

/* El robot es mou fins trobar un objecte. Llavors calcula el vector de 
repulsió per tal d'esquivar l'objecte. Es robot es mou per l'entorn fins 
que es pitja la tecla Esc */
void TactRainer::wander() {
  double vel, th, th_dmin, alpha;
  ArKeyHandler keyHandler;
  Vect2D vro;

  Aria::setKeyHandler(&keyHandler);
  ar.attachKeyHandler(&keyHandler);

  vel = getNormalVel();
  th = getMaxDist();
  th_dmin = getImpactDist();

  alpha = 0.0;

  while (1) {
    //Avançam fins a trobar un objecte
    findObject(vel, th);
    
    //Ens aturam
    ar.setVel(0);

    //Calculam el vector de repulsió i l'angle de gir
    vro = obstacleRepulsion(th, th_dmin, NULL, NULL);
    alpha = ArMath::atan2(vro.y, vro.x);
    
    //Orientam el robot cap a la direcció que ha de seguir		
    ar.setHeading(alpha);
    currHeading = alpha;
    while (!ar.isHeadingDone(thHeading)) {
      ArUtil::sleep(blindTime);
    }
  }
}

/* Fa que el robot vagi al punt pnt evitant els obstaques que troba. 
 * Retorna:
 *   False si es considera que no es pot arribar al punt 
 *   True si ha pogut arribar al punt
 */
bool TactRainer::goGoal(Point2D pnt, double obsRadius=100.0) {
  double alpha, vel;
  double d, e;
  double heading;
  bool impactAlert, obstacle;
  bool canAccess;
  Point2D hereP;
  Point2D nearObstaclePoint;
  
   /* Vector Repulsió Obstacle, Vector Atraccio objectiu, Vector Director */
  Vect2D vro, va, vd;
      
  d = DBL_MAX;
  e = DBL_MAX;
  canAccess = true;
  while ((d >= thOnPoint) and canAccess) { 
    d = ArMath::distanceBetween(ar.getX(), ar.getY(), pnt.x, pnt.y);
    
    va = goalAttraction(pnt);
    vro = obstacleRepulsion(maxDist, impactDist, &obstacle,
                           &impactAlert, &nearObstaclePoint);
        
    if (obstacle) {
      /* Miram si el punt on anam i l'obstacle detectat estan suficientment
         junts com per considerar el punt inaccessible */
      e = ArMath::distanceBetween(nearObstaclePoint.x, nearObstaclePoint.y, pnt.x, pnt.y);
      if (e < obsRadius) {
        return false;
      }
    }
    if (impactAlert) {
      /* Col.lisió imminent, sols tenim amb compte el vector de repulsió */
      vd.x = vro.x;
      vd.y = vro.y;
      vel = slowVel;
      printf("Alerta de col.lisió imminet!!\n");
    } else {      
      /* Recalculam el vector director del moviment del robot */
      vd = (va * behaviourWeight[BH_GOAL]) + (vro * behaviourWeight[BH_OBSTACLE]);
      vel = normalVel;
    }
        
    alpha = ArMath::atan2(vd.y, vd.x);
    
    ar.setHeading(alpha);
    currHeading = alpha;
        
    if (obstacle) {
      heading = thObsHeading;
    } else {
      heading = getThHeading(alpha);
    }
    while (!ar.isHeadingDone(heading)) {
      ar.setVel(0);
    }

    ar.setVel(vel);
    ArUtil::sleep(blindTime);
    
  }
  return canAccess;
}

double TactRainer::getCrX() {
  return robotXinit;
} 

double TactRainer::getCrY() {
  return robotYinit;
}

int TactRainer::getSizeX() {
  return areaXsize;
}

int TactRainer::getSizeY() {
  return areaYsize;
}

double TactRainer::getCe() {
  return cellEdge;
}
  
