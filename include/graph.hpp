#pragma once

#include <iostream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "cmapf_types.hpp"

using namespace cmapf;


// Graph interface
template<typename AdjList>
class Graph {
	public:
		// constructor
		Graph(){}
		
    // methods
    virtual void add_node(AgentPosition, std::vector<float>) = 0;
    virtual void add_edge(AgentPosition, AgentPosition) = 0;
    
    // access to the graph
    std::unordered_set<AgentPosition> get_neighbors_set(AgentPosition pos) {
      std::unordered_set<AgentPosition> result;

      auto neighbor_it = boost::adjacent_vertices(pos, adj_list_);
      for (AgentPosition neighbor_pos: make_iterator_range(neighbor_it)) {
        result.insert(neighbor_pos);
      }
      result.insert(pos);
      
      return result;
    }

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

    const std::map<AgentPosition, std::vector<float>> get_positions() { 
      return positions_; 
    }

    const std::vector<float> get_position(AgentPosition node) {
      return positions_.at(node);
    }

    const AgentPosition get_node(std::vector<float> position) {
      if (nodes_.find(position) != nodes_.end()) {
        return nodes_[position];
      }

      return -1;
    }

    const int node_count() { 
      return positions_.size(); 
    }

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