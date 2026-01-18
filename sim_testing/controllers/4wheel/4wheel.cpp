#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#define TIME_STEP 64
#define DEFAULT_VEL 3.5
#define THRESH 950
using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Motor *wheels[4];
  char wheelNames[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  DistanceSensor *ds[2];
  char dsNames[2][10] = {"ds_right", "ds_left"};
  
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheelNames[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  for (int i = 0; i < 2; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }

  int timeStep = (int)robot->getBasicTimeStep();
  int counter = 0;
  while (robot->step(timeStep) != -1) {
    double dsVals[2];
    for (int i = 0; i < 2; i++) {
      dsVals[i] = ds[i]->getValue();
    }
    
    double leftVels = DEFAULT_VEL;
    double rightVels = DEFAULT_VEL;
    if (counter > 0) {
      leftVels *= -1;
      counter--;
    } else {
      if (dsVals[0] < THRESH || dsVals[1] < THRESH) {
        counter = 40;
      }
    }

    wheels[0]->setVelocity(leftVels);
    wheels[1]->setVelocity(rightVels);
    wheels[2]->setVelocity(leftVels);
    wheels[3]->setVelocity(rightVels);
   };

  delete robot;
  return 0;
}
