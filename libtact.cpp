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
  thHeadingObstacled = param["thHeadingObstacled"];
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
  Vect2D va(goal.x, goal.y, ar.getX(), ar.getY());  
  return va.norm();   
}

Vect2D TactRainer::obstacleRepulsion(double th, double th_dmin,
				     bool *obstacle, bool *impactAlert, Coor *nearObstacleCoor) {
  double mod_vObs [numSonar];
  double di = 0.0;
  int nearSens; /* Sensor que ha detectat distància mínima */
  double dmin = DBL_MAX;
  Vect2D vObs [numSonar]; 
  Vect2D vRep (0.0, 0.0);
  ArSensorReading *sensor;
  
  *nearObstacleCoor->x = DBL_MAX;
  *nearObstacleCoor->y = DBL_MAX;
  *obstacle = false;
  /* Calcul de la repulsió dels sensors de davant */
  for (int i = numFirstSonar; i <= numLastSonar; i++) {
    sensor = ar.getSonarReading(i);
    di = (double)sensor->getRange();
    
    if (di < th) {
      *obstacle = true;
      vObs[i].setXY(sensor->getX() - sensor->getXTaken(), sensor->getY() - sensor->getYTaken());
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
      nearSens = i;
      *nearObstacleCoor->x = sensor->getX();
      *nearObxtacleCoor->y = sensor->getY();
      vRep += vObs[i] * sonarWeight[i];
    }
        
  }  
  
  *impactAlert = (dmin < th_dmin);
  
  return vRep.norm(); //TODO Es correcte normalitzar?
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

/* Fa que el robot es mogui fins que es troba a una distancia menor de th. */
int TactRainer::findObject(double vel, double th) {
  ArSensorReading *sensor;	
  double xRob, yRob, xObs, yObs, d;
  
  ar.setVel(vel); 
  
  while (true) {
    for (int i = numFirstSonar; i <= numLastFrontSonar; i++) {
      sensor = ar.getSonarReading(i);
      d = (double)sensor->getRange();
            
      if (d < th) {
	printf("Obesrvant el sensor %d\n", i);
	printf("Posicio robot %f %f\n", ar.getX(), ar.getY());
	printf("getXY: %f, %f\n", sensor->getX(), sensor->getY());
	printf("getXYTaken: %f, %f\n", sensor->getXTaken(), sensor->getYTaken());
	printf("getSensorDXY: %f, %f\n", sensor->getSensorDX(), sensor->getSensorDY());
	printf("vector al sensor %f, %f\n", sensor->getX() - sensor->getSensorX(), sensor->getY() - sensor->getSensorY());
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
  bool impactAlert, obstacle;
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
    printf("He trobat un objecte, m'atur.\n");
    
    //Ens aturam
    ar.setVel(0);

    //Calculam el vector de repulsió i l'angle de gir
    vro = obstacleRepulsion(th, th_dmin, &obstacle, &impactAlert);
    alpha = ArMath::atan2(vro.y, vro.x);
    printf("Reorientació a (%f, %f) que suposa un angle %f\n",vro.x, vro.y, alpha);
    
    //Orientam el robot cap a la direcció que ha de seguir		
    ar.setHeading(alpha);
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
bool TactRainer::goGoal(Point2D pnt, double obsRadius) {
  double alpha, vel;
  double d;
  double heading;
  bool impactAlert, obstacle;
  bool canAccess;
  Point2D hereP;
  
   /* Vector Repulsió Obstacle, Vector Atraccio objectiu, Vector Director */
  Vect2D vro, va, vd;
    
  //Trace trace (elephantMem, distObstacledTh, timeObstacledTh); // TODO

  printf("M'encamin al punt (%f, %f)\n", pnt.x, pnt.y);
  
  d = DBL_MAX;
  canAccess = true;
  while ((d >= thOnPoint) and canAccess) { 
    
    if (DEBUG) { /* El primer cop hi poden haver valors no calculats, don't worry */
      printf("A distància %f (%f de llindar)\n", d, thOnPoint);
      printf("(%f, %f) -> (%f, %f)\n", ar.getX(), ar.getY(), pnt.x, pnt.y);
      printf("VA: (%f, %f)\n", va.x, va.y);
      printf("VO: (%f, %f)\n", vro.x, vro.y);
      printf("VD: (%f, %f)\n", vd.x, vd.y);
      printf("alpha: %f\n\n", alpha);
    }
    
    d = ArMath::distanceBetween(ar.getX(), ar.getY(), pnt.x, pnt.y);
    
    va = goalAttraction(pnt);
    vro = obstacleRepulsion(maxDist, impactDist, &obstacle, &impactAlert);
        
    if (obstacle) {
      /* Miram si el punt on anam i l'obstacle detectat estan suficientment
	junts com per considerar el punt inaccessible */
	
      
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
    
    /* Si la diferencia entre angles fos molt petita es faria sense avançar */
    if (fabs(alpha) > dr) {
      if (obstacle) {
	printf("Obstacled TH\n");
	heading = thHeadingObstacled;
      } else {
	heading = getThHeading(alpha);
      }
      while (!ar.isHeadingDone(heading)) {
	ar.setVel(0);
      }
    }

    ar.setVel(vel);
    ArUtil::sleep(blindTime);

    /* Si esteim detectant algun obstacle memoritzam la posicio */
    if (!vro.isZero()) {
      hereP.setXY(ar.getX(), ar.getY());
      //trace.add(hereP);// TODO
    }
    
    //canAccess = not trace.isInnaccessible(); // TODO   
  }
  return canAccess;
}

