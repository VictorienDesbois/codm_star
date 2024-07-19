#include <iostream>
#include <fstream>
#include <string>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "instance_loader.hpp"


InstanceLoader::InstanceLoader() {}

void InstanceLoader::file_exists(const std::string& path) {
  std::ifstream f(path.c_str());
  if (!f.good()) {
    throw std::runtime_error("File " + std::string(path) + " not found.");
  }
}

void InstanceLoader::load_edge(const rapidxml::xml_node<>* graph, GraphType graph_type) {
  for (const rapidxml::xml_node<>* edge = graph->first_node("edge"); edge != NULL; edge = edge->next_sibling()) {
    
    rapidxml::xml_attribute<>* attr = edge->first_attribute("source");
    if (attr == NULL) continue;
    AgentPosition s = (AgentPosition)std::stoi(attr->value() + 1);
    
    attr = attr->next_attribute("target");
    AgentPosition t = (AgentPosition)std::stoi(attr->value() + 1);

    switch(graph_type) {
      case Move: moves_graph.add_edge(s, t, 1.0); break;
      case Comm: comm_graph.add_edge(s, t); break;
    }
  }
} 

void InstanceLoader::load_xml(const std::string& filepath, const std::string& graph_location) {
  file_exists(filepath);

  std::ifstream infile(filepath);

  std::string line;
  std::string type;
  std::string smove_file;
  std::string scomm_file;

  std::getline(infile, line);
  std::istringstream iss1(line);
  iss1 >> type >> smove_file;
  smove_file = graph_location + smove_file;
  std::getline(infile, line);
  std::istringstream iss2(line);
  iss2 >> type >> scomm_file;
  scomm_file = graph_location + scomm_file;

  const char* move_file = smove_file.data();
  const char* comm_file = scomm_file.data();

  // movements
  file_exists(std::string(move_file));
  
  // communications
  file_exists(std::string(comm_file));

  // xml move graph
  rapidxml::file<> xmlFile(move_file);
  rapidxml::xml_document<> move_doc;
  move_doc.parse<0>(xmlFile.data());

  const rapidxml::xml_node<>* move_graph_xml = move_doc.first_node("graphml")->first_node("graph");

  // Creating the nodes
  for (const rapidxml::xml_node<>* node = move_graph_xml->first_node("node"); node != NULL; node = node->next_sibling()) {
    
    rapidxml::xml_attribute<>* attr = node->first_attribute("id");
    if (attr == NULL) continue;
    std::string id(attr->value());
    id.erase(0, 1);
    
    const rapidxml::xml_node<>* data = node->first_node("data");
    float y = std::stof(data->value()) - 0.5;
    data = data->next_sibling();
    float x = std::stof(data->value()) - 0.5;

    AgentPosition node_id = (AgentPosition)std::stoi(id.c_str());
    std::vector<float> position = {x, y};

    if (data->next_sibling() != 0) {
      data = data->next_sibling();
      float z = std::stof(data->value()) - 0.5;
      position.push_back(z);
    }

    moves_graph.add_node(node_id, position);
    comm_graph.add_node(node_id, position);
  }

  // Loading the movement edges
  load_edge(move_graph_xml, Move);

  // xml comm graph
  xmlFile = rapidxml::file<>(comm_file);
  rapidxml::xml_document<> com_doc;
  com_doc.parse<0>(xmlFile.data());

  const rapidxml::xml_node<>* comm_graph_xml = com_doc.first_node("graphml")->first_node("graph");

  // Loading the communication edges
  load_edge(comm_graph_xml, Comm);

  std::string startConf;
  std::getline(infile, startConf);

  size_t startI = 0;
  auto endI = startConf.find(" ");

  // Creating the start configuration
  while (endI != std::string::npos) {
    std::string s = startConf.substr(startI, endI - startI);
    
    if (s != "start") {
      start.push_back(std::stoi(s));
    }

    startI = endI + 1;
    endI = startConf.find(" ", startI);
  }

  std::string goalConf;
  std::getline(infile, goalConf);

  startI = 0;
  endI = goalConf.find(" ");

  // Creating the goal configuration
  while (endI != std::string::npos) {
    std::string s = goalConf.substr(startI, endI - startI);

    if (s != "goal") { 
      goal.push_back(std::stoi(s));
    }

    startI = endI + 1;
    endI = goalConf.find(" ", startI);
  }
}