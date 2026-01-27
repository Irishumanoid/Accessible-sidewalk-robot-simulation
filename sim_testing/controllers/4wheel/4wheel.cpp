#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <cmath>

#define TIME_STEP 64
#define DEFAULT_VEL 5
#define SCALE_FACTOR 500.0
#define THRESH 400
using namespace webots;

enum class State {
  FORWARD,
  TURN_LEFT,
  TURN_RIGHT
};

enum class NavState {
  OBST_AVOID,
  NAV_TO_POINT,
};

State state = State::FORWARD;
NavState navState = NavState::NAV_TO_POINT;
int turnCounter = 0;


void logData(DistanceSensor *ds[2], Motor *wheels[4], GPS *gps, InertialUnit *imu) {
  printf("dist sensor vals: %.2f and %.2f\n", ds[0]->getValue(), ds[1]->getValue());
  printf("left wheel velocities: %.2f, %.2f\n", wheels[0]->getVelocity(), wheels[2]->getVelocity());
  printf("right wheel velocities: %.2f, %.2f\n", wheels[1]->getVelocity(), wheels[3]->getVelocity());

  const double *gps_coords = gps->getValues();
  printf("gps: (%.2f m, %.2f m, %.2f)\n", gps_coords[0], gps_coords[1], gps_coords[2]);
  printf("imu yaw (deg): %.2f\n", imu->getRollPitchYaw()[2] * 180 / 3.14159);
}

void navToPoint(Robot* robot, Motor *wheels[4], GPS *gps, InertialUnit *imu, DistanceSensor *ds[2], double target_x, double target_y, double rot_thresh, double pos_thresh) {
  const double *cur_pos = gps->getValues();
  double cur_x = cur_pos[0];
  double cur_y = cur_pos[1];
  double kP_turn = 5.0;
  double target_angle = std::atan2(target_y - cur_y, target_x - cur_x);

  // turn, then drive straight
  while (robot->step(TIME_STEP) != -1) {
    logData(ds, wheels, gps, imu);
    double cur_yaw = imu->getRollPitchYaw()[2];
    double error = target_angle - cur_yaw;
    printf("target yaw: %.2f\n", target_angle);

    while (error > M_PI) error -= 2 * M_PI;
    while (error < -M_PI) error += 2 * M_PI;
    if (std::abs(error) < rot_thresh) break;

    double turn_speed = kP_turn * error;
    for (int i = 0; i < 4; i++) {
      wheels[i]->setVelocity(turn_speed * (i % 2 == 0 ? -1 : 1));
    }
  }

  while (robot->step(TIME_STEP) != -1) {
    logData(ds, wheels, gps, imu);
    cur_pos = gps->getValues();
    cur_x = cur_pos[0];
    cur_y = cur_pos[1];
    double error = std::sqrt(std::pow(target_x - cur_x, 2) + std::pow(target_y - cur_y, 2));
    if (std::abs(error) < pos_thresh) break;

    for (int i = 0; i < 4; i++) {
      wheels[i]->setVelocity(DEFAULT_VEL);
    }
  }

  for (int i = 0; i < 4; i++) {
    wheels[i]->setVelocity(0.0);
  }
}


int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Motor *wheels[4];
  char wheelNames[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  DistanceSensor *ds[2];
  char dsNames[2][10] = {"ds_right", "ds_left"};

  Camera *camera = robot->getCamera("camera");
  camera->enable(TIME_STEP);
  GPS *gps = robot->getGPS("gps_main");
  gps->enable(TIME_STEP);
  InertialUnit *imu = robot->getInertialUnit("inertial unit");
  imu->enable(TIME_STEP);

  
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheelNames[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  for (int i = 0; i < 2; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }
  robot->step(TIME_STEP);

  if (navState == NavState::OBST_AVOID) {
    while (robot->step(TIME_STEP) != -1) {
    double dsVals[2];
    for (int i = 0; i < 2; i++) {
      dsVals[i] = ds[i]->getValue();
    }
    
    double leftVels = DEFAULT_VEL;
    double rightVels = DEFAULT_VEL;

    if (state == State::FORWARD) {
      if (dsVals[0] > THRESH && dsVals[1] > THRESH) {
        state = (rand() % 2 == 0) ? State::TURN_LEFT : State::TURN_RIGHT;
        turnCounter = 40;
      } else if (dsVals[0] > THRESH) {
        state = State::TURN_LEFT;
        turnCounter = 30;
      } else if (dsVals[1] > THRESH) {
        state = State::TURN_RIGHT;
        turnCounter = 30;
      }
    }

    switch (state) {
      case State::FORWARD:
        leftVels = DEFAULT_VEL;
        rightVels = DEFAULT_VEL;
        break;
      case State::TURN_LEFT:
        leftVels = DEFAULT_VEL / 4;
        rightVels = DEFAULT_VEL;
        turnCounter--;
        break;
      case State::TURN_RIGHT:
        leftVels = DEFAULT_VEL;
        rightVels = DEFAULT_VEL / 4;
        turnCounter--;
        break;
    }

    if (turnCounter <= 0) {
      state = State::FORWARD;
    }

    wheels[0]->setVelocity(leftVels);
    wheels[1]->setVelocity(rightVels);
    wheels[2]->setVelocity(leftVels);
    wheels[3]->setVelocity(rightVels);
    logData(ds, wheels, gps, imu);
    };
  } else {
    navToPoint(robot, wheels, gps, imu, ds, 50.0, -120.0, 0.5, 5);
  }

  delete robot;
  return 0;
}
