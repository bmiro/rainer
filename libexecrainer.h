class Rainer {
private:
  double thHeading;
  double thOnPoint;
  double maxDist;
  double impactDist;
  
  double blindTime;
  
  int numSensFront;
  int numFirstSensro;
  int numLastSensor;
  
  double normalVel;
  
  double sonarWeight[];
  double behaviourWeight[];
  
public:
  ArRobot ar; /* Robot de l'aria */
  
  void initArobot();
  
  double getThHeading();
  double thOnPoint();
  double maxDist();
  double impactDist();
  
  double blindTime();
  
  int numSensFront();
  int numFirstSonar();
  int numLastSonar();
  
  double normalVel();
  
  double getSonarWeight(int s);
  double getBehaviorWeight(int behaviour);
}