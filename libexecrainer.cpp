void initArRobot(int argc, char **argv) {
  Aria::init();
  ArSimpleConnector connector(&argc, argv);
  if (!connector.parseArgs() || argc > 1) {
    connector.logOptions();
    exit(1);
  }
  if (!connector.connectRobot(&ar)) {
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    return 1;
  }
  robot3.comInt(ArCommands::SOUNDTOG, 1);
  robot3.comInt(ArCommands::ENABLE, 1); //Habilitar els motors
  robot3.runAsync(false);
}

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