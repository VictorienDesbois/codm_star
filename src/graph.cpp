#include <queue>

#include "graph.hpp"


//// Movements ////

MovesGraph::MovesGraph() {}

void MovesGraph::add_node(AgentPosition node_id, std::vector<float> position) {
  // Possibility for the agents to wait (stay in their position).
  boost::add_edge(node_id, node_id, 0.0, adj_list_);

	positions_[node_id] = position;
  nodes_[position] = node_id;
}

void MovesGraph::add_edge(AgentPosition s, AgentPosition t) {
  add_edge(s, t, 1.0);
}

void MovesGraph::add_edge(AgentPosition s, AgentPosition t, float cost = 1.0) {
  boost::add_edge(s, t, cost, adj_list_);
  boost::add_edge(t, s, cost, adj_list_);
}



//// Communications ////

CommunicationsGraph::CommunicationsGraph() {}

void CommunicationsGraph::add_node(AgentPosition node_id, std::vector<float> position) {
  boost::add_edge(node_id, node_id, adj_list_);

	positions_[node_id] = position;
}

void CommunicationsGraph::add_edge(AgentPosition s, AgentPosition t) {
  boost::add_edge(s, t, adj_list_);
  boost::add_edge(t, s, adj_list_);
}

bool CommunicationsGraph::is_configuration_connected(Configuration config) {
	// Check if the configuration is connected or not.
  
  assert(config.size() > 0);
  int nb_agents = config.size();
  std::set<AgentPosition> visited_agent;
	std::queue<AgentPosition> current_neighbors;
  AgentPosition current_pos;

  current_neighbors.push(config.at(0));
  config.erase(config.begin());
  int nb_visited_agent = 1;

  while(!current_neighbors.empty()){
  	current_pos = current_neighbors.front();
    auto neighbor = boost::adjacent_vertices(current_pos, adj_list_);
    current_neighbors.pop();

  	while(neighbor.first != neighbor.second) {
      auto neighbor_pos = *(neighbor.first);
      auto search = find(config.begin(), config.end(), neighbor_pos);
      if (search != config.end()) {
        current_neighbors.push(neighbor_pos);
        config.erase(search);
        nb_visited_agent++;
      }
      neighbor.first++;
  	}
  }

  return nb_visited_agent == nb_agents;
}