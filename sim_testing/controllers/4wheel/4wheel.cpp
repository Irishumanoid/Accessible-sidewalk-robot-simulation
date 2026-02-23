#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include "webots/Emitter.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <map>
#include <vector>
#include <tuple>
#include <limits>
#include <queue>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <string>
#include <fstream>
#include <iostream>

#include "json.hpp"
using json = nlohmann::json;

#define TIME_STEP 64
#define DEFAULT_VEL 5
#define MAX_VEL 10
#define SCALE_FACTOR 500.0
#define THRESH 400
#define TRANSLATION_ERR_THRESH 5
#define RAD_CONV 3.14159/180
#define EARTH_RAD 6378137
#define LAT_CENTER 47.686
#define OFFSET_X 44.7
#define OFFSET_Y -122
#define TURN_PENALTY 5
#define DEBUG 0

using namespace webots;

enum class State { FORWARD, TURN_LEFT, TURN_RIGHT };
enum class NavState { OBST_AVOID, NAV_TO_POINT };

State state = State::FORWARD;
NavState nav_state = NavState::NAV_TO_POINT;
const double kP_turn = 5.0;
const double kD_turn = 2.1;
int turn_counter = 0;

struct Coord { double x, y; };

struct PathNode {
  long long id;
  Coord loc;
  bool issue;
};

struct Edge {
  long long to;
  double cost;
};


double heuristic(const Coord &a, const Coord &b) {
  double dx = (b.x - a.x) * RAD_CONV * EARTH_RAD * std::cos(RAD_CONV * LAT_CENTER);
  double dy = (b.y - a.y) * RAD_CONV * EARTH_RAD;
  return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}

void logData(DistanceSensor *ds[4], Motor *wheels[4], GPS *gps, InertialUnit *imu) {
  printf("dist sensor vals: (%.2f, %.2f, %.2f, %.2f) \n", ds[0]->getValue(), ds[1]->getValue(), ds[2]->getValue(), ds[3]->getValue());
  const double *gps_coords = gps->getValues();
  printf("gps: (%.2f m, %.2f m, %.2f)\n", gps_coords[0], gps_coords[1], gps_coords[2]);
  printf("imu yaw (deg): %.2f\n", imu->getRollPitchYaw()[2] * 180 / 3.14159);
}

double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

void stopWheels(Motor *wheels[4]) {
  for (int i = 0; i < 4; i++) {
    wheels[i]->setVelocity(0.0);
  }
}
// TODO last mile navigation to go on grass (bottom 20% of camera should be grass or sidewalk)

void navToPoint(Robot* robot, Motor *wheels[4], Camera *camera, GPS *gps, InertialUnit *imu,
                DistanceSensor *ds[4], double target_x, double target_y,
                double rot_thresh, double pos_thresh) {

  enum class NavMode { TURN_TO_TARGET, MOVE_TO_TARGET, AVOID_OBSTACLE };
  NavMode mode = NavMode::TURN_TO_TARGET;

  int turn_counter = 0;
  int move_counter = 0;
  State avoid_state = State::FORWARD;
  double crosswalk_target_angle = 0;

  double last_err = 0;
  while (robot->step(TIME_STEP) != -1) {
    #if DEBUG
      logData(ds, wheels, gps, imu);
    #endif

    double dsVals[4];
    for (int i = 0; i < 4; i++) dsVals[i] = ds[i]->getValue();

    const double *cur_pos = gps->getValues();
    double cur_x = cur_pos[0];
    double cur_y = cur_pos[1];
    double cur_yaw = imu->getRollPitchYaw()[2];

    double dx = target_x - cur_x;
    double dy = target_y - cur_y;
    double dist_err = std::sqrt(dx * dx + dy * dy);


    const CameraRecognitionObject *detected_objs = camera->getRecognitionObjects();
    for (int i = 0; i < camera->getRecognitionNumberOfObjects(); i++) {
      const CameraRecognitionObject &obj = detected_objs[i];
      const double *position = obj.position;
      double distance = std::sqrt(std::pow(position[0], 2) + std::pow(position[1], 2));
      //printf("obj name: %s at distance %.2f \n", obj.model, distance);
    }

    // look at bottom 20% of screen to find orientation of robot relative to grass
    int scan_y = 0.8 * camera->getHeight();
    int width = camera->getWidth();
    int grass_loc_x = -1;
    for (int i = 0; i < width; i++) {
      const auto image = camera->getImage();
      int r = Camera::imageGetRed(image, width, i, scan_y);
      int g = Camera::imageGetGreen(image, width, i, scan_y);
      int b = Camera::imageGetBlue(image, width, i, scan_y);

      bool isGrass = g > r + 20 && g > b + 20;
      if (isGrass) {
        grass_loc_x = i;
      }
    }
    if (grass_loc_x != -1 && (grass_loc_x - width / 2) != 0) {

    }


    #if DEBUG
    printf("**************************\n");
    #endif

    bool obstacle = false;
    for (double val : dsVals) {
      if (val > THRESH) obstacle = true;
    }
    if (obstacle && mode != NavMode::AVOID_OBSTACLE) {
      mode = NavMode::AVOID_OBSTACLE;

      if (dsVals[1] > THRESH && dsVals[2] > THRESH) {
        if (dsVals[0] > THRESH && dsVals[3] < THRESH) {
          avoid_state = State::TURN_RIGHT;
          turn_counter = 20;
        } else if (dsVals[0] < THRESH && dsVals[3] > THRESH) {
          avoid_state = State::TURN_LEFT;
          turn_counter = 20;
        } else {
          avoid_state = (rand() % 2 == 0) ? State::TURN_LEFT : State::TURN_RIGHT;
          turn_counter = 40;
        }
      } else if (dsVals[0] > THRESH || dsVals[1] > THRESH) {
        avoid_state = State::TURN_RIGHT;
        turn_counter = 30;
      } else {
        avoid_state = State::TURN_LEFT;
        turn_counter = 30;
      }
      move_counter = 5;
    }

    switch (mode) {
      case NavMode::TURN_TO_TARGET: {
        #if DEBUG
          printf("MODE: TURN_TO_TARGET\n");
        #endif
        double target_angle = std::atan2(dy, dx);
        double err = target_angle - cur_yaw;
        while (err > M_PI) err -= 2 * M_PI;
        while (err < -M_PI) err += 2 * M_PI;

        if (std::abs(err) < rot_thresh) {
          stopWheels(wheels);
          mode = NavMode::MOVE_TO_TARGET;
          break;
        }

        double turn_speed = clamp(kP_turn * err + kD_turn * (err - last_err) / (TIME_STEP / 1000.0), -MAX_VEL, MAX_VEL);
        last_err = err;
        for (int i = 0; i < 4; i++) {
          wheels[i]->setVelocity(turn_speed * (i % 2 == 0 ? -1 : 1));
        }
        break;
      }
      case NavMode::MOVE_TO_TARGET: {
        #if DEBUG
          printf("MODE: MOVE_TO_TARGET\n");
        #endif
        if (dist_err < pos_thresh) {
          stopWheels(wheels);
          return;
        }

        for (int i = 0; i < 4; i++)
          wheels[i]->setVelocity(DEFAULT_VEL);
        break;
      }
      case NavMode::AVOID_OBSTACLE: {
        #if DEBUG
          printf("MODE: AVOID_OBSTACLE\n");
        #endif
        double leftVels = DEFAULT_VEL;
        double rightVels = DEFAULT_VEL;

        if (turn_counter > 0) {
          if (avoid_state == State::TURN_LEFT) {
            leftVels = DEFAULT_VEL / 4;
            rightVels = DEFAULT_VEL;
          } else {
            leftVels = DEFAULT_VEL;
            rightVels = DEFAULT_VEL / 4;
          }
          turn_counter--;
        } else if (move_counter > 0) {
          move_counter--;
        } else {
          mode = NavMode::TURN_TO_TARGET;
          break;
        }

        for (int i = 0; i < 4; i++) {
          wheels[i]->setVelocity(i % 2 == 0 ? leftVels : rightVels);
        }
        break;
      }
    }
  }

  stopWheels(wheels);
}

const std::unordered_map<long long, PathNode> getPathNodes(const std::string &filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) throw std::runtime_error("Could not open json file");

  json j; file >> j;
  std::unordered_map<long long, PathNode> nodes;
  const json &json_nodes = j.at("nodes");

  for (auto it = json_nodes.begin(); it != json_nodes.end(); ++it) {
    long long id = std::stoll(it.key());
    const json &n = it.value();

    PathNode node;
    node.loc.x = n.at("x").get<double>() + OFFSET_X;
    node.loc.y = n.at("y").get<double>() + OFFSET_Y;
    node.issue = n.at("issue").get<bool>();

    nodes.emplace(id, node);
  }
  return nodes;
}

const std::unordered_map<long long, std::vector<Edge>> getEdges(const std::string &filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) throw std::runtime_error("Could not open json file");

  json j; file >> j;
  std::unordered_map<long long, std::vector<Edge>> edges;
  const json &json_edges = j.at("adjacency");

  for (auto it = json_edges.begin(); it != json_edges.end(); ++it) {
    long long id = std::stoll(it.key());
    const json &n = it.value();

    std::vector<Edge> id_edges;
    for (const auto &edge_end : n) {
      long long to = edge_end["to"].get<long long>();
      double cost = edge_end["cost"].get<double>();
      id_edges.emplace_back(Edge{to, cost});
    }
    edges[id] = id_edges;
  }
  return edges;
}

std::vector<Coord> generateAStarPath(
    const std::unordered_map<long long, PathNode> &nodes,
    const std::unordered_map<long long, std::vector<Edge>> &edges,
    Coord start_loc, Coord target_loc) {

  auto closest = [&](Coord c) {
    long long best = -1; double best_dist = std::numeric_limits<double>::max();
    for (const auto &[id, n] : nodes) {
      double d = heuristic(n.loc, c);
      if (d < best_dist) { best_dist = d; best = id; }
    }
    return best;
  };

  long long start = closest(start_loc);
  long long goal = closest(target_loc);

  using State = std::pair<double, long long>; // cost, node id
  std::priority_queue<State, std::vector<State>, std::greater<State>> open;
  std::unordered_map<long long, double> g;
  std::unordered_map<long long, long long> came_from;
  std::unordered_set<long long> closed;

  g[start] = 0.0;
  open.push({heuristic(nodes.at(start).loc, nodes.at(goal).loc), start});

  while (!open.empty()) {
    auto [f, current] = open.top(); open.pop();
    if (closed.count(current)) continue;
    if (current == goal) break;

    closed.insert(current);

    for (const auto &e : edges.at(current)) {
      if (closed.count(e.to)) continue;

      PathNode nodeFrom = nodes.at(current);
      PathNode nodeTo = nodes.at(e.to);
      double cur_yaw = std::atan2(nodeTo.loc.y - nodeFrom.loc.y, nodeTo.loc.x - nodeFrom.loc.x);

      double prev_yaw = cur_yaw;
      if (came_from.count(current)) {
        PathNode prev = nodes.at(came_from[current]);
        prev_yaw = std::atan2(nodeFrom.loc.y - prev.loc.y, nodeFrom.loc.x - prev.loc.x);
      }

      double d_theta = cur_yaw - prev_yaw;
      while (d_theta > M_PI) d_theta -= 2 * M_PI;
      while (d_theta < -M_PI) d_theta += 2 * M_PI;
      d_theta = fabs(d_theta);

      double cost = heuristic(nodeFrom.loc, nodeTo.loc);
      if (d_theta > M_PI / 4) cost *= TURN_PENALTY;

      double tentative = g[current] + cost;
      if (!g.count(e.to) || tentative < g[e.to]) {
        came_from[e.to] = current;
        g[e.to] = tentative;

        double fscore = tentative + heuristic(nodes.at(e.to).loc, nodes.at(goal).loc);
        open.push({fscore, e.to});
      }
    }
  }

  std::vector<Coord> path;
  for (long long cur = goal; cur != start; cur = came_from[cur]) {
    path.push_back(nodes.at(cur).loc);
  }
  path.push_back(nodes.at(start).loc);
  std::reverse(path.begin(), path.end());
  return path;
}

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Motor *wheels[4];
  char wheelNames[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  DistanceSensor *ds[4];
  char dsNames[4][13] = {"ds_far_left", "ds_left", "ds_right", "ds_far_right"};

  Camera *camera = robot->getCamera("camera");
  camera->enable(TIME_STEP);
  camera->recognitionEnable(TIME_STEP);

  GPS *gps = robot->getGPS("gps_main");
  gps->enable(TIME_STEP);
  InertialUnit *imu = robot->getInertialUnit("inertial-unit");
  imu->enable(TIME_STEP);

  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheelNames[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }

  for (int i = 0; i < 4; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }
  robot->step(TIME_STEP);

  if (nav_state == NavState::OBST_AVOID) {
    while (robot->step(TIME_STEP) != -1) {
      double dsVals[4];
      for (int i = 0; i < 2; i++) dsVals[i] = ds[i]->getValue();

      double leftVels = DEFAULT_VEL, rightVels = DEFAULT_VEL;

      if (state == State::FORWARD) {
        if (dsVals[1] > THRESH && dsVals[2] > THRESH) {
          state = (rand() % 2 == 0) ? State::TURN_LEFT : State::TURN_RIGHT;
          turn_counter = 40;
        } else if (dsVals[1] > THRESH) {
          state = State::TURN_LEFT; turn_counter = 30;
        } else if (dsVals[2] > THRESH) {
          state = State::TURN_RIGHT; turn_counter = 30;
        }
      }

      switch (state) {
        case State::FORWARD: leftVels = rightVels = DEFAULT_VEL; break;
        case State::TURN_LEFT: leftVels = DEFAULT_VEL / 4; rightVels = DEFAULT_VEL; turn_counter--; break;
        case State::TURN_RIGHT: leftVels = DEFAULT_VEL; rightVels = DEFAULT_VEL / 4; turn_counter--; break;
      }

      if (turn_counter <= 0) state = State::FORWARD;

      wheels[0]->setVelocity(leftVels);
      wheels[1]->setVelocity(rightVels);
      wheels[2]->setVelocity(leftVels);
      wheels[3]->setVelocity(rightVels);
      #if DEBUG
        logData(ds, wheels, gps, imu);
      #endif
    }
  } else {
    std::string path = "/Users/irislitiu/Webots-sims/adjacency_map.json";
    const auto path_nodes = getPathNodes(path);
    const auto edges = getEdges(path);
    const Coord start = {38.6+OFFSET_X, -66.1+OFFSET_Y};
    const Coord end = {48.0+OFFSET_X, -20.3+OFFSET_Y};
    std::vector<Coord> fullPath = generateAStarPath(path_nodes, edges, start, end);

    Emitter *emitter = robot->getEmitter("emitter");
    for (Coord &c : fullPath) {
      printf("coord (%.2f, %.2f)\n", c.x, c.y);
      emitter->send(&c, sizeof(Coord));
    }

    const char* test_data = "test msg";
    emitter->send(test_data, strlen(test_data) + 1);

    printf("end coord is: x=%.2f, y=%.2f\n", fullPath.back().x, fullPath.back().y);
    for (int i = 0; i < fullPath.size(); i++) {
      Coord c = fullPath.at(i);
      printf("currently at iteration %.2d\n", i);
      double distTolerance = i == fullPath.size() - 1 ? 2.0 : 2.0;
      navToPoint(robot, wheels, camera, gps, imu, ds, c.x, c.y, 0.01, distTolerance);
    }
  }

  delete robot;
  return 0;
}
