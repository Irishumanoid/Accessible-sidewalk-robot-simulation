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
#define PI 3.14159
#define RAD_CONV 3.14159/180
#define EARTH_RAD 6378137
#define LAT_CENTER 47.686
#define OFFSET_X 44.7
#define OFFSET_Y -122
#define TURN_PENALTY 5
#define DEBUG 0
#define NONE -1
#define NUM_CONSEQ_DETECTIONS 5

using namespace webots;

enum class State { FORWARD, TURN_LEFT, TURN_RIGHT };
enum class NavState { OBST_AVOID, NAV_TO_POINT };

State state = State::FORWARD;
NavState nav_state = NavState::NAV_TO_POINT;
const double kP_turn = 5.0;
const double kD_turn = 2.1;
int turn_counter = 0;
int r_bounds[2] = {135, 165}, g_bounds[2] = {125, 145}, b_bounds[2] = {115, 130};

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

double angle_diff(double a1, double a2) {
  return atan2(sin(a1 - a2), cos(a1 - a2));
}

void stop_wheels(Motor *wheels[4]) {
  for (int i = 0; i < 4; i++) {
    wheels[i]->setVelocity(0.0);
  }
}

bool grass_pixel(int r, int g, int b) {
  return (g > r + 20) && (g > b + 20);
}

bool sidewalk_pixel(int r, int g, int b) {
  return r >= r_bounds[0] && r <= r_bounds[1] &&
          g >= g_bounds[0] && g <= g_bounds[1] &&
          b >= b_bounds[0] && b <= b_bounds[1];
}

void navToPoint(Robot* robot, Motor *wheels[4], Camera *camera, GPS *gps, InertialUnit *imu,
                DistanceSensor *ds[4], double target_x, double target_y,
                double rot_thresh, double pos_thresh) {

  enum class NavMode { TURN_TO_TARGET, MOVE_TO_TARGET, AVOID_OBSTACLE, ALIGN_TO_SIDEWALK, NAV_ON_SIDEWALK };
  NavMode mode = NavMode::TURN_TO_TARGET;

  int turn_counter = 0;
  int move_counter = 0;
  State avoid_state = State::FORWARD;
  double last_err = 0.0;

  const int MAX_STEPS = 5000;
  int step_counter = 0;

  double sidewalk_heading = 0.0;
  bool have_sidewalk_heading = false;
  bool is_bottom_sidewalk = false;
  while (robot->step(TIME_STEP) != -1 && step_counter < MAX_STEPS) {
    step_counter++;

    double dsVals[4];
    for (int i = 0; i < 4; i++) {
      dsVals[i] = ds[i]->getValue();
    }

    const double *cur_pos = gps->getValues();
    double cur_x = cur_pos[0];
    double cur_y = cur_pos[1];
    double cur_yaw = imu->getRollPitchYaw()[2];

    double dx = target_x - cur_x;
    double dy = target_y - cur_y;
    double target_angle = atan2(dy, dx);
    double dist_err = std::sqrt(dx * dx + dy * dy);

    if (camera) {
      const unsigned char *image = camera->getImage();
      if (image) {
        int width = camera->getWidth();
        int height = camera->getHeight();
        double bottom_row[3] = {0.0, 0.0, 0.0};

        for (int i = 0; i < width; i++) {
          bottom_row[0] += Camera::imageGetRed(image, width, i, height - 1);
          bottom_row[1] += Camera::imageGetGreen(image, width, i, height - 1);
          bottom_row[2] += Camera::imageGetBlue(image, width, i, height - 1);
        }
        for (int i = 0; i < 3; i++) bottom_row[i] /= width;        
        is_bottom_sidewalk = sidewalk_pixel(bottom_row[0], bottom_row[1], bottom_row[2]);

        if (is_bottom_sidewalk) {
          std::cout << "paved surface detected" << std::endl;
          // detect boundary between grass and sidewalk
          std::vector<double> xs, ys;
          for (int y = 0.8 * height; y < height; y++) {
            for (int x = 1; x < width; x++) {
              int r = Camera::imageGetRed(image, width, x, y);
              int g = Camera::imageGetGreen(image, width, x, y);
              int b = Camera::imageGetBlue(image, width, x, y);

              int r_prev = Camera::imageGetRed(image, width, x-1, y);
              int g_prev = Camera::imageGetGreen(image, width, x-1, y);
              int b_prev = Camera::imageGetBlue(image, width, x-1, y);

              bool is_grass = grass_pixel(r, g, b);
              bool was_grass = grass_pixel(r_prev, g_prev, b_prev);
              bool is_sidewalk = sidewalk_pixel(r, g, b);
              bool was_sidewalk = sidewalk_pixel(r_prev, g_prev, b_prev);

              if ((is_grass && was_sidewalk) || (was_grass && is_sidewalk)) {
                xs.push_back(x);
                ys.push_back(y);
                break;
              }
            }
          }

          if (xs.size() > 10) {
            double dx_edge = xs.back() - xs.front();
            double dy_edge = ys.back() - ys.front();
            double edge_angle = atan2(dy_edge, dx_edge);

            double a1 = edge_angle;
            double a2 = a1 + M_PI;
            a1 = atan2(sin(a1), cos(a1));
            a2 = atan2(sin(a2), cos(a2));

            if (!have_sidewalk_heading) {
              sidewalk_heading =
                fabs(angle_diff(a1, target_angle)) <
                fabs(angle_diff(a2, target_angle)) ? a1 : a2;
              have_sidewalk_heading = true;
            } else {
              if (fabs(angle_diff(a2, sidewalk_heading)) <
                  fabs(angle_diff(a1, sidewalk_heading))) {
                sidewalk_heading = a2;
              } else {
                sidewalk_heading = a1;
              }
            }
            
            if (mode != NavMode::ALIGN_TO_SIDEWALK && mode != NavMode::NAV_ON_SIDEWALK) {
              mode = NavMode::ALIGN_TO_SIDEWALK;
            }
          }
        } else {
          have_sidewalk_heading = false;
        }
      }
    }

    bool obstacle = false;
    for (double val : dsVals) if (val > THRESH) obstacle = true;

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

    double leftVels = 0.0, rightVels = 0.0;
    switch (mode) {
      case NavMode::TURN_TO_TARGET: {
        std::cout << "nav mode: TURN_TO_TARGET" << std::endl;
        double err = target_angle - cur_yaw;
        while (err > M_PI) err -= 2 * M_PI;
        while (err < -M_PI) err += 2 * M_PI;

        if (std::abs(err) < rot_thresh) {
          stop_wheels(wheels);
          mode = NavMode::MOVE_TO_TARGET;
          break;
        }

        double turn_speed = clamp(kP_turn * err + kD_turn * (err - last_err) / (TIME_STEP / 1000.0), -MAX_VEL, MAX_VEL);
        last_err = err;
        leftVels = -turn_speed;
        rightVels = turn_speed;
        break;
      }

      case NavMode::MOVE_TO_TARGET: {
        std::cout << "nav mode: MOVE_TO_TARGET" << std::endl;
        if (dist_err < pos_thresh) {
          stop_wheels(wheels);
          return;
        }
        leftVels = rightVels = DEFAULT_VEL;
        break;
      }

      case NavMode::AVOID_OBSTACLE: {
        std::cout << "nav mode: AVOID_OBSTACLE" << std::endl;
        if (turn_counter > 0) {
          if (avoid_state == State::TURN_LEFT) { leftVels = DEFAULT_VEL / 4; rightVels = DEFAULT_VEL; }
          else { leftVels = DEFAULT_VEL; rightVels = DEFAULT_VEL / 4; }
          turn_counter--;
        } else if (move_counter > 0) {
          leftVels = rightVels = DEFAULT_VEL;
          move_counter--;
        } else {
          mode = NavMode::TURN_TO_TARGET;
        }
        break;
      }

      case NavMode::ALIGN_TO_SIDEWALK: {
        std::cout << "nav mode: ALIGN_TO_SIDEWALK" << std::endl;
        double heading_error = angle_diff(sidewalk_heading, cur_yaw);
        printf("sidewalk heading: %.6f, cur_yaw: %.6f, error: %.6f\n", sidewalk_heading, cur_yaw, std::abs(heading_error));

        if (std::abs(heading_error) >= rot_thresh) {
          double turn_speed = clamp(kP_turn * heading_error, -MAX_VEL, MAX_VEL);
          leftVels = -turn_speed;
          rightVels = turn_speed;
        } else {
          mode = NavMode::NAV_ON_SIDEWALK;
        }
        break;
      }

      case NavMode::NAV_ON_SIDEWALK: {
        std::cout << "nav mode: NAV_ON_SIDEWALK" << std::endl;
        double progress = cos(sidewalk_heading - target_angle);
        if (progress < 0.01) {
          stop_wheels(wheels);
          mode = NavMode::TURN_TO_TARGET;
        } else {
          leftVels = rightVels = DEFAULT_VEL * clamp(progress, 0.3, 1.0);
        }
        break;
      }
    }

    for (int i = 0; i < 4; i++) {
      wheels[i]->setVelocity(i % 2 == 0 ? leftVels : rightVels);
    }
  }

  stop_wheels(wheels);
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
      for (int i = 0; i < 4; i++) dsVals[i] = ds[i]->getValue();

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
    const Coord end = {15.8+OFFSET_X, -54.3+OFFSET_Y}; // far: (48.0, -20.3), close: (15.8, -54.3)
    std::vector<Coord> fullPath = generateAStarPath(path_nodes, edges, start, end);

    Emitter *emitter = robot->getEmitter("emitter");
    for (Coord &c : fullPath) {
      printf("coord (%.2f, %.2f)\n", c.x, c.y);
      emitter->send(&c, sizeof(Coord));
    }

    printf("end coord is: x=%.2f, y=%.2f\n", fullPath.back().x, fullPath.back().y);
    for (int i = 0; i < fullPath.size(); i++) {
      Coord c = fullPath.at(i);
      printf("currently at iteration %.2d\n", i);
      double distTolerance = i == fullPath.size() - 1 ? 2.0 : 2.0;
      navToPoint(robot, wheels, camera, gps, imu, ds, c.x, c.y, 0.01, distTolerance);
    }
    std::string message = "finished path";
    emitter->send(message.c_str(), message.length()+1);
  }

  delete robot;
  return 0;
}