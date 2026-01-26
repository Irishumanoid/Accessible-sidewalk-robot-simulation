#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 64
#define DEFAULT_VEL 5
#define THRESH 200
using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Motor *wheels[4];
  char wheelNames[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  DistanceSensor *ds[2];
  char dsNames[2][10] = {"ds_right", "ds_left"};

  Camera camera = Camera("camera");
  camera.enable(TIME_STEP);

  
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
      leftVels = DEFAULT_VEL;
      rightVels = DEFAULT_VEL / 2;
      counter--;
    } else {
      if (dsVals[0] > THRESH || dsVals[1] > THRESH) {
        counter = 5;
      }
    }
    
    printf("dist sensor vals: %.2f and %.2f\n", dsVals[0], dsVals[1]);
    printf("left wheel velocities: %.2f, %.2f\n", wheels[0]->getVelocity(), wheels[2]->getVelocity());
    printf("right wheel velocities: %.2f, %.2f\n", wheels[1]->getVelocity(), wheels[3]->getVelocity());

    wheels[0]->setVelocity(leftVels);
    wheels[1]->setVelocity(rightVels);
    wheels[2]->setVelocity(leftVels);
    wheels[3]->setVelocity(rightVels);
   };

  delete robot;
  return 0;
}
