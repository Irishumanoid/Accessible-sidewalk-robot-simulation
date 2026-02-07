#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>

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
#define SCALE_FACTOR 500.0
#define THRESH 400
#define TRANSLATION_ERR_THRESH 5
#define RAD_CONV 3.14159/180
#define EARTH_RAD 6378137
#define LAT_CENTER 47.686
#define OFFSET_X 44.7
#define OFFSET_Y -122

using namespace webots;

enum class State { FORWARD, TURN_LEFT, TURN_RIGHT };
enum class NavState { OBST_AVOID, NAV_TO_POINT };

State state = State::FORWARD;
NavState navState = NavState::NAV_TO_POINT;
int turnCounter = 0;

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
  //return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));  
}

void logData(DistanceSensor *ds[2], Motor *wheels[4], GPS *gps, InertialUnit *imu) {
  /*printf("dist sensor vals: %.2f and %.2f\n", ds[0]->getValue(), ds[1]->getValue());
  printf("left wheel velocities: %.2f, %.2f\n", wheels[0]->getVelocity(), wheels[2]->getVelocity());
  printf("right wheel velocities: %.2f, %.2f\n", wheels[1]->getVelocity(), wheels[3]->getVelocity());*/

  const double *gps_coords = gps->getValues();
  //printf("gps: (%.2f m, %.2f m, %.2f)\n", gps_coords[0], gps_coords[1], gps_coords[2]);
  //printf("imu yaw (deg): %.2f\n", imu->getRollPitchYaw()[2] * 180 / 3.14159);
}

void navToPoint(Robot* robot, Motor *wheels[4], GPS *gps, InertialUnit *imu,
                DistanceSensor *ds[2], double target_x, double target_y,
                double rot_thresh, double pos_thresh) {

  const double *cur_pos = gps->getValues();
  double cur_x = cur_pos[0], cur_y = cur_pos[1];
  double kP_turn = 5.0;
  double target_angle = std::atan2(target_y - cur_y, target_x - cur_x);

  // turn towards target
  while (robot->step(TIME_STEP) != -1) {
    logData(ds, wheels, gps, imu);
    double cur_yaw = imu->getRollPitchYaw()[2];
    double error = target_angle - cur_yaw;

    while (error > M_PI) error -= 2 * M_PI;
    while (error < -M_PI) error += 2 * M_PI;
    if (std::abs(error) < rot_thresh) break;

    double turn_speed = kP_turn * error;
    for (int i = 0; i < 4; i++) {
      wheels[i]->setVelocity(turn_speed * (i % 2 == 0 ? -1 : 1));
    }
  }

  bool close_to_thresh = false;
  bool interrupt = false;
  int moveCounter = 0;
  double dsVals[2];
  while (robot->step(TIME_STEP) != -1) {
    logData(ds, wheels, gps, imu);
    for (int i = 0; i < 2; i++) dsVals[i] = ds[i]->getValue();
    // manual path override for obstacle avoidance
    if (state == State::FORWARD && (dsVals[0] > THRESH || dsVals[1] > THRESH)) {
      interrupt = true;
      if (dsVals[0] > THRESH && dsVals[1] > THRESH) {
        state = (rand() % 2 == 0) ? State::TURN_LEFT : State::TURN_RIGHT;
        turnCounter = 40;
      } else if (dsVals[0] > THRESH) {
        state = State::TURN_LEFT; turnCounter = 30;
      } else if (dsVals[1] > THRESH) {
        state = State::TURN_RIGHT; turnCounter = 30;
      }
    }
    // if interrupt ever becomes true, call navToPoint again after diverting robot so it reaches the correct endpoint
    if (interrupt && (turnCounter > 0 || moveCounter > 0)) {
      double leftVels = DEFAULT_VEL, rightVels = DEFAULT_VEL;
      switch (state) {
        case State::FORWARD: leftVels = rightVels = DEFAULT_VEL; break;
        case State::TURN_LEFT: leftVels = DEFAULT_VEL / 4; rightVels = DEFAULT_VEL; turnCounter--; break;
        case State::TURN_RIGHT: leftVels = DEFAULT_VEL; rightVels = DEFAULT_VEL / 4; turnCounter--; break;
      }
      if (turnCounter <= 0) {
        state = State::FORWARD;
        moveCounter = 20;
      }
      if (moveCounter > 0) moveCounter--;
      wheels[0]->setVelocity(leftVels);
      wheels[1]->setVelocity(rightVels);
      wheels[2]->setVelocity(leftVels);
      wheels[3]->setVelocity(rightVels);
    } else if (!interrupt) {
      cur_pos = gps->getValues();
      cur_x = cur_pos[0]; cur_y = cur_pos[1];
      double error = std::sqrt(std::pow(target_x - cur_x, 2) + std::pow(target_y - cur_y, 2));
      if (error < pos_thresh) break;
      if (error < pos_thresh * 2) close_to_thresh = true;
      if (close_to_thresh && error > pos_thresh * 2) {
        printf("overshot end location, stopped at (%.2f, %.2f)", cur_x, cur_y);
        break;
      }

      for (int i = 0; i < 4; i++) {
        wheels[i]->setVelocity(DEFAULT_VEL);
      }
    } else {
      // reschedule navToPoint
      return navToPoint(robot, wheels, gps, imu, ds, target_x, target_y, rot_thresh, pos_thresh);
    }
  }

  for (int i = 0; i < 4; i++) wheels[i]->setVelocity(0.0);
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
    node.loc.x = n.at("x").get<double>()+OFFSET_X;
    node.loc.y = n.at("y").get<double>()+OFFSET_Y;
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

      double tentative = g[current] + e.cost;
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
  DistanceSensor *ds[2];
  char dsNames[2][10] = {"ds_right", "ds_left"};

  Camera *camera = robot->getCamera("camera");
  camera->enable(TIME_STEP);
  GPS *gps = robot->getGPS("gps_main");
  gps->enable(TIME_STEP);
  InertialUnit *imu = robot->getInertialUnit("inertial-unit");
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
      for (int i = 0; i < 2; i++) dsVals[i] = ds[i]->getValue();

      double leftVels = DEFAULT_VEL, rightVels = DEFAULT_VEL;

      if (state == State::FORWARD) {
        if (dsVals[0] > THRESH && dsVals[1] > THRESH) {
          state = (rand() % 2 == 0) ? State::TURN_LEFT : State::TURN_RIGHT;
          turnCounter = 40;
        } else if (dsVals[0] > THRESH) {
          state = State::TURN_LEFT; turnCounter = 30;
        } else if (dsVals[1] > THRESH) {
          state = State::TURN_RIGHT; turnCounter = 30;
        }
      }

      switch (state) {
        case State::FORWARD: leftVels = rightVels = DEFAULT_VEL; break;
        case State::TURN_LEFT: leftVels = DEFAULT_VEL / 4; rightVels = DEFAULT_VEL; turnCounter--; break;
        case State::TURN_RIGHT: leftVels = DEFAULT_VEL; rightVels = DEFAULT_VEL / 4; turnCounter--; break;
      }

      if (turnCounter <= 0) state = State::FORWARD;

      wheels[0]->setVelocity(leftVels);
      wheels[1]->setVelocity(rightVels);
      wheels[2]->setVelocity(leftVels);
      wheels[3]->setVelocity(rightVels);
      logData(ds, wheels, gps, imu);
    }
  } else {
    std::string path = "/Users/irislitiu/Webots-sims/adjacency_map.json";
    const auto path_nodes = getPathNodes(path);
    const auto edges = getEdges(path);
    const Coord start = {38.6+OFFSET_X, -66.1+OFFSET_Y};
    const Coord end = {15.8+OFFSET_X, -53.4+OFFSET_Y};
    std::vector<Coord> fullPath = generateAStarPath(path_nodes, edges, start, end);
    for (Coord &c : fullPath) {
      printf("coord (%.2f, %.2f)\n", c.x, c.y);
    }
    printf("end coord is: x=%.2f, y=%.2f\n", fullPath.back().x, fullPath.back().y);
    for (int i = 0; i < fullPath.size(); i++) {
      Coord c = fullPath.at(i);
      printf("currently at iteration %.2d\n", i);
      navToPoint(robot, wheels, gps, imu, ds, c.x, c.y, 0.05, 1);
    }
  }

  delete robot;
  return 0;
}
