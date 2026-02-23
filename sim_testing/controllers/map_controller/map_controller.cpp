#include <webots/Supervisor.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstring>
#include <map>
#include <unordered_map>
#include <webots/Receiver.hpp>

#define TIME_STEP 64
#define COORD_THRESH 0.05
#define OFFSET_X 44.7
#define OFFSET_Y -122
using namespace webots;

struct Coord { double x, y; };

const std::unordered_map<std::string, std::string> MARKER_TEMPLATES = {
  { "SurfaceProblem", "SurfaceProblemMarker" },
  { "NoCurbRamp", "NoCurbRampMarker" },
  { "NoSidewalk", "NoSidewalkMarker" }
};

Node *createMarker(
  Supervisor *robot,
  Field *children,
  const std::string &type,
  int id,
  double x,
  double y,
  double z
) {
  std::string defName = type + "_MARKER_" + std::to_string(id);
  Node *existingNode = robot->getFromDef(defName);
  if (existingNode) {
    return nullptr;
  }

  std::string nodeString =
    "DEF " + defName + " " + MARKER_TEMPLATES.at(type) + " {\n"
    "  translation " + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + "\n"
    "  name \"" + defName + "\"\n}";

  children->importMFNodeFromString(-1, nodeString);
  Node *node = robot->getFromDef(defName);
  if (!node) {
    std::cerr << "Failed to create marker: " << defName << '\n';
  }
  return node;
}

Node *createPole(
  Supervisor *robot,
  Field *children,
  bool isIssue,
  std::string id,
  double x,
  double y
) {
  std::string defName = "POLE_MARKER_" + id;
  Node *existingNode = robot->getFromDef(defName);
  if (existingNode) {
    return nullptr;
  }

  std::string nodeString = "DEF " + defName + " PoleMarker {\n"
  "  baseColor " + std::to_string(isIssue ? 0.664134 : 0.0) + " " + std::to_string(isIssue ? 0.0552834 : 0.576852) + " " + std::to_string(isIssue ? 0.0 : 0.607004) + "\n"
  "  translation " + std::to_string(x) + " " + std::to_string(y) + " 25\n}";

  children->importMFNodeFromString(-1, nodeString);
  Node *node = robot->getFromDef(defName);
  if (!node) {
    std::cerr << "Failed to create pole: " << defName << '\n';
  }
  return node;
}

int main(int argc, char **argv) {
  Supervisor *robot = new Supervisor();
  Node *rootNode = robot->getRoot();
  Field *childrenField = rootNode->getField("children");
  Receiver *receiver = robot->getReceiver("receiver");
  receiver->enable(TIME_STEP);

  Node *marker = robot->getFromDef("MARKER");
  if (!marker) {
    std::cerr << "Marker node does not exist";
    delete robot;
    return 1;
  }
  
  std::string markerString = marker->exportString();
  if (markerString.find("DEF MARKER") == std::string::npos) {
    std::cerr << "DEF MARKER not found in exportString()\n";
    delete robot;
    return 1;
  }

  // read new markers from file
  std::ifstream csvFile("/Users/irislitiu/Webots-sims/translations.csv");
  if (!csvFile.is_open()) {
    std::cerr << "Failed to open markers csv\n";
    delete robot;
    return 1;
  }
  
  std::string line;
  int counter = 1;
  bool firstLine = true;

  while (std::getline(csvFile, line)) {
    if (firstLine) {firstLine = false; continue;}

    std::stringstream ss(line);
    std::string entry;
    std::vector<std::string> entries;

    while (std::getline(ss, entry, ',')) {
      entries.push_back(entry);
    }
    for (std::string entry : entries) {
      std::cout << entry << ", ";
    }
    std::cout << '\n';

    Node *marker = createMarker(
      robot,
      childrenField,
      entries[1],             
      counter,                 
      std::stod(entries[2]),  
      std::stod(entries[3]),   
      0.0                     
    );

    if (marker) {
      std::cout << entries[1] << " marker created successfully\n";
    }
    counter++;
  }

  csvFile.close();


  csvFile.open("/Users/irislitiu/Webots-sims/adjacency_map_nodes_correct.csv");
  if (!csvFile.is_open()) {
    std::cerr << "Failed to open csv\n";
    delete robot;
    return 1;
  }
  
  std::map<std::string, Coord> poleNames;
  firstLine = true;
  while (std::getline(csvFile, line)) {
    if (firstLine) {firstLine = false; continue;}

    std::stringstream ss(line);
    std::string entry;
    std::vector<std::string> entries;

    while (std::getline(ss, entry, ',')) {
      entries.push_back(entry);
    }
    for (std::string entry : entries) {
      std::cout << entry << ", ";
    }

    std::cout << '\n';
    Node *pole = createPole(
      robot,
      childrenField,
      entries[4] == "True",
      entries[1],                          
      std::stod(entries[2]),  
      std::stod(entries[3])                 
    );

    std::string defName = "POLE_MARKER_" + entries[1];
    Coord coord = {std::stod(entries[2]), std::stod(entries[3])};
    poleNames[defName] = coord;
  }
  csvFile.close();

  std::vector<Coord> pathCoords;
  while (robot->step(TIME_STEP) != -1) {
    if (receiver->getQueueLength() > 0) {
      const void *data = receiver->getData();
      int size = receiver->getDataSize();

      if (size == sizeof(Coord)) {
        Coord coord;
        memcpy(&coord, data, sizeof(Coord));
        printf("received coord: (%.2f, %.2f)\n", coord.x, coord.y);
        pathCoords.push_back(coord);
      } else {
        std::cerr << "Ignoring packet that is not of size " << sizeof(Coord);
      }
      receiver->nextPacket();
    } else if (receiver->getQueueLength() == 0 && pathCoords.size() > 0) {
      double yellow[3] = {1.0, 1.0, 0.0};
      for (Coord coord : pathCoords) {
        coord.x -= OFFSET_X;
        coord.y -= OFFSET_Y;
        for (const auto& [name, poleCoords] : poleNames) {
          if (abs(poleCoords.x - coord.x) < COORD_THRESH && abs(poleCoords.y - coord.y) < COORD_THRESH) {
            Node *pole = robot->getFromDef(name);
            Field *colorField = pole->getField("baseColor");
            if (!colorField) {
              std::cerr << "No baseColor field on " << name << '\n';
              break;
            }
            colorField->setSFColor(yellow);
          }
        }
      }
      pathCoords.clear();
    }
  };

  delete robot;
  return 0;
}
