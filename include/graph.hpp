#pragma once

#include <iostream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "cmapf_types.hpp"

using namespace cmapf;


// Graph interface
template<typename AdjList>

/**
 * A graph class that manages its associated embedding in
 * a space of a given dimension.
 */
class Graph {
	public:
		// constructor
		Graph(){}
		
    // methods
    /**
     * @brief Adds a node with a specified position and associated costs.
     *
     * @param position The position coordinate.
     * @param costs A vector of costs associated with the given position.
     */
    virtual void add_node(AgentPosition, std::vector<float>) = 0;
    

    /**
     * @brief Adds an edge between two positions.
     *
     * @param u The first position.
     * @param v The second position.
     *
     */
    virtual void add_edge(AgentPosition, AgentPosition) = 0;
    
    
    /**
     * @brief Retrieves the set of neighboring positions for a given agent position.
     *
     * @param pos The position coordinate, represented as an `AgentPosition` object.
     * @return A `std::unordered_set` of neighboring positions, each represented as an `AgentPosition` object.
     */
    std::unordered_set<AgentPosition> get_neighbors_set(AgentPosition pos) {
      std::unordered_set<AgentPosition> result;

      auto neighbor_it = boost::adjacent_vertices(pos, adj_list_);
      for (AgentPosition neighbor_pos: make_iterator_range(neighbor_it)) {
        result.insert(neighbor_pos);
      }
      result.insert(pos);
      
      return result;
    }


    /**
     * @brief Computes the Euclidean distance between two agent positions.
     *
     * @param n1 The first position, represented as an `AgentPosition` object.
     * @param n2 The second position, represented as an `AgentPosition` object.
     * @return The Euclidean distance between the two positions as a `float`.
     */
    float euclidean_distance(AgentPosition n1, AgentPosition n2) {
      assert(positions_.find(n1) != positions_.end() && positions_.find(n2) != positions_.end());
      
      std::vector<float> p1 = positions_[n1];
      std::vector<float> p2 = positions_[n2];
      const size_t dimension = p1.size();
      float result = 0; 

      for (int i = 0; i < dimension; i++) {
        result += (p1.at(i) - p2.at(i)) * (p1.at(i) - p2.at(i));
      }

      return sqrt(result);
    }


    /**
     * @brief Retrieves a map of positions and their associated coordinates.
     *
     * @return A `std::map` where each key is an `AgentPosition` and the corresponding value is a `std::vector<float>` 
     *         representing the coordinate associated with that position.
     */
    const std::map<AgentPosition, std::vector<float>> get_positions() { 
      return positions_; 
    }


    /**
     * @brief Retrieves the coordinate associated with a given position.
     *
     * @param node The position of the agent, represented as an `AgentPosition` object.
     * @return A `std::vector<float>` containing the coordinate associated with the specified position.
     */    
    const std::vector<float> get_position(AgentPosition node) {
      return positions_.at(node);
    }


    /**
     * @brief Retrieves the agent position corresponding to a given coordinate.
     *
     * @param position A `std::vector<float>` representing the coordinates of the position.
     * @return The agent position corresponding to the given coordinates, represented as an `AgentPosition` object.
     */
    const AgentPosition get_node(std::vector<float> position) {
      if (nodes_.find(position) != nodes_.end()) {
        return nodes_[position];
      }

      return -1;
    }


    /**
     * @brief Retrieves the number of nodes in the graph.
     *
     * @return The count of nodes as an `int`.
     */
    const int node_count() { 
      return positions_.size(); 
    }


    /**
     * @brief Retrieves the adjacency list of the graph.
     *
     * @return A constant reference to the adjacency list, represented as an `AdjList` object.
     */    
    const AdjList& get_adj_list() { 
      return adj_list_; 
    }

  protected:
    std::map<AgentPosition, std::vector<float>> positions_;
    std::unordered_map<std::vector<float>, AgentPosition> nodes_;
    AdjList adj_list_;
};

// Implementations of the interface
class MovesGraph: public Graph<Moves> {
  public:
    MovesGraph();

    void add_node(AgentPosition node_id, std::vector<float> position) override;
    void add_edge(AgentPosition s, AgentPosition t) override;
    void add_edge(AgentPosition s, AgentPosition t, float cost);
};

class CommunicationsGraph: public Graph<Communications> {
  public:
    CommunicationsGraph();

    void add_node(AgentPosition node_id, std::vector<float> position) override;
    void add_edge(AgentPosition s, AgentPosition t) override;
    bool is_configuration_connected(Configuration config);
};