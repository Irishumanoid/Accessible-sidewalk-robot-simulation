#include <webots/Supervisor.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <format>


#define TIME_STEP 32
using namespace webots;

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
  int timeStep = (int)robot->getBasicTimeStep();
  while (robot->step(timeStep) != -1) {

  };

  delete robot;
  return 0;
}
