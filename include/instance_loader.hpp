#pragma once

#include <sstream>
#include <vector>
#include <string>
#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>

#include "graph.hpp"


class InstanceLoader {
  public:
    // constructors
    InstanceLoader();

    // methods
    MovesGraph get_movement_graph() { return moves_graph; }
    CommunicationsGraph get_comm_graph() { return comm_graph; }
    Configuration get_start() { return start; }
    Configuration get_goal() { return goal; }

    /**
     * @brief Load a CMAPF instance from an XML file.
     * 
     * @param filepath The path to the XML file.
     * @param graph_location The location of the graph within the XML file.
     */
    void load_xml(const std::string& filepath, const std::string& graph_location);

  private:
    enum GraphType { Move, Comm };

    // attributes
    MovesGraph moves_graph;
    CommunicationsGraph comm_graph;
    Configuration start;
    Configuration goal;


    /**
     * @brief Check if a file exists at the specified path.
     * 
     * @param path The path to the file.
     */
    void file_exists(const std::string& path);
    

    /**
     * @brief Load edge information from an XML node into a graph.
     * 
     * @param graph A pointer to the XML node containing graph data.
     * @param graph_type The type of the graph to be loaded.
     */
    void load_edge(const rapidxml::xml_node<>* graph, GraphType graph_type);
};