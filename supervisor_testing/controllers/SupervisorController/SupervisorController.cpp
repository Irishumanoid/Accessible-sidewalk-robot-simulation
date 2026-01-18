#include <webots/Supervisor.hpp>

#define TIME_STEP 32

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {

  Supervisor *robot = new Supervisor(); // create Supervisor instance

  // [CODE PLACEHOLDER 1]
  Node *bb8Node = robot->getFromDef("BB-8");
  Field *translationField = bb8Node->getField("translation");

  Node *rootNode = robot->getRoot();
  Field *childrenField = rootNode->getField("children");

  childrenField->importMFNodeFromString(-1, "DEF BALL Ball { translation 0 1 1 }");
  Node *ballNode = robot->getFromDef("BALL");
  Field *colorField = ballNode->getField("color");

  int i = 0;
  while (robot->step(TIME_STEP) != -1) {
    // [CODE PLACEHOLDER 2]
    if (i == 0) {
      const double newValue[3] = {2.5, 0, 0};
      translationField->setSFVec3f(newValue);
    }

    if (i == 10)
      bb8Node->remove();

    if (i == 20)
      childrenField->importMFNodeFromString(-1, "Nao { translation 2.5 0 0.334 }");

    const double *position = ballNode->getPosition();
    std::cout << "Ball position: " << position[0] << " " << position[1] << " " << position[2] << std::endl;

    if (position[2] < 0.2) {
      const double redColor[3] = {1, 0, 0};
      colorField->setSFColor(redColor);
    }

    i++;
  }

  delete robot;

  return 0;
}