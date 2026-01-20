#include <webots/Supervisor.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#define TIME_STEP 32
using namespace webots;

int main(int argc, char **argv) {
  Supervisor *robot = new Supervisor();
  Node *rootNode = robot->getRoot();
  Field *childrenField = rootNode->getField("children");

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
  std::ifstream csvFile("/Users/irislitiu/Webots_sims/translations.csv");
  if (!csvFile.is_open()) {
    std::cerr << "Failed to open csv\n";
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

    std::string markerName = "DEF MARKER_" + std::to_string(counter);
    std::string curMarkerString = markerString;
    size_t pos = curMarkerString.find("DEF MARKER");
    if (pos == std::string::npos) {
      std::cerr << "DEF MARKER not found in markerString\n";
      continue;
    }
    curMarkerString.replace(pos, 10, markerName);
    childrenField->importMFNodeFromString(-1, curMarkerString);
    robot->step(TIME_STEP);

    int newIndex = childrenField->getCount() - 1;
    Node *markerClone = childrenField->getMFNode(newIndex);
    if (!markerClone) {
      std::cerr << "marker NOT found after import\n";
    } else {
      std::cout << "marker successfully created\n";
      double pos[3] = {std::stod(entries[2]), std::stod(entries[3]), 0};
      markerClone->getField("translation")->setSFVec3f(pos);
    }

    counter++;
  }

  csvFile.close();
  int timeStep = (int)robot->getBasicTimeStep();
  while (robot->step(timeStep) != -1) {

  };

  delete robot;
  return 0;
}
