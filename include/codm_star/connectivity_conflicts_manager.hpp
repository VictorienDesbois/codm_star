#pragma once

#include "cmapf_types.hpp"
#include "graph.hpp"


namespace codm_star {

  struct ConnectivityManager {

    /**
     * @brief Given a configuration ``c``, compute a set of meta-agent
     * that contains the connected group of agents within the ``c``.
     * 
     * @param c a Configuration.
     * @param comm_graph The communication graph.
     * @return Return the connected group of agents as a set of MetaAgents.
     */
    MetaAgents get_connected_groups(
      const Configuration &c,
      CommunicationsGraph &comm_graph
    ) {
      assert(c.size() > 0);

      MetaAgents connected_groups;
      const Communications &adj_list = comm_graph.get_adj_list();
      
      std::unordered_set<Agent> all_agents;
      for (Agent a = 0; a < c.size(); a++) { 
        all_agents.insert(a); 
      }

      std::queue<AgentPosition> current_neighbors;
      AgentPosition current_pos;

      while(!all_agents.empty()) {
        Agent a = *all_agents.begin();

        all_agents.erase(a);
        current_neighbors.push(c.at(a));

        MetaAgent current_ma;
        current_ma.insert(a);

        while(!current_neighbors.empty()) {
          current_pos = current_neighbors.front();
          current_neighbors.pop();

          auto neighbors = boost::adjacent_vertices(current_pos, comm_graph.get_adj_list());
          for (AgentPosition neighbor_pos: make_iterator_range(neighbors)) {

            std::vector<Agent> to_erase;

            for (Agent current_agent: all_agents) {
              
              if (c.at(current_agent) == neighbor_pos) {

                if (neighbor_pos != current_pos) {
                  current_neighbors.push(neighbor_pos);
                }
                current_ma.insert(current_agent);
                to_erase.push_back(current_agent);
              }
            }

            for (Agent agent_erase: to_erase) {
              all_agents.erase(agent_erase);
            }
          }
        }

        connected_groups.push_back(current_ma);
      }

      return connected_groups;
    }


    /**
     * @brief Given two configurations `s` and `t`, compute a set of disconnected
     *        pairs of meta-agents between the two configurations.
     * 
     * This method analyzes the given configurations and the communications graph
     * to identify pairs of meta-agents that are disconnected between the two
     * configurations. It returns a set of meta-agent pairs that are connected
     * in the first configuration but are disconnected in the other.
     * 
     * @param s The first Configuration to compare.
     * @param t The second Configuration to compare.
     * @param comm_graph The communication graph.
     * @return Returns a set of disconnected meta-agent pairs between the configurations `s`
     *         and `t`.
     */
    MetaAgents get_disconnected_couple(
      const Configuration &s,
      const Configuration &t,
      CommunicationsGraph &comm_graph
    ) {
      assert(s.size() == t.size());
      const Communications &adj_list = comm_graph.get_adj_list();
      MetaAgents meta_agents_set;
      
      std::vector<std::pair<Agent, Agent>> source_pairs = get_connected_agents(s, adj_list);
      MetaAgents connected_groups = get_connected_groups(t, comm_graph);

      for (std::pair<Agent, Agent> current_pair: source_pairs) {
        Agent a = current_pair.first;
        Agent b = current_pair.second;
        MetaAgent current_ma;

        int id_a = -1;
        int id_b = -1;

        int i = 0;
        for (MetaAgent ma: connected_groups) {
          if (ma.find(a) != ma.end()) { id_a = i; }
          if (ma.find(b) != ma.end()) { id_b = i; }
          i++;

          if (id_a != -1 && id_b != -1 && id_a != id_b) {
            current_ma.insert(a);
            current_ma.insert(b);
            meta_agents_set.push_back(current_ma);
            break;
          }
        }
      }

      return meta_agents_set;
    }


    /**
     * @brief Given two configurations return the meta-agent set of
     * agents that needs to be coupled during a planning iteration.
     * 
     * @param s The first Configuration to compare.
     * @param t The second Configuration to compare.
     * @param comm_graph Communication graphs.
     * @return Returns a set of coupled meta-agents.
     */
    MetaAgents get_coupled_agent(
      const Configuration &s,
      const Configuration &t,
      CommunicationsGraph &comm_graph
    ) {
      assert(s.size() == t.size());
      const Communications &adj_list = comm_graph.get_adj_list();
      MetaAgents meta_agents_set;
      
      std::vector<std::pair<Agent, Agent>> source_pairs = get_connected_agents(s, adj_list);
      std::vector<std::pair<Agent, Agent>> target_pairs = get_connected_agents(t, adj_list);

      for (std::pair<Agent, Agent> current_pair: source_pairs) {
        if (std::find(target_pairs.begin(), target_pairs.end(), current_pair) == target_pairs.end()) {
          MetaAgent current_ma;
          current_ma.insert(current_pair.first);
          current_ma.insert(current_pair.second);
          meta_agents_set.push_back(current_ma);
        }
      }

      return meta_agents_set;
    }


    /**
     * @brief Check and return all connected pairs of agent inside a configuration.
     * 
     * @param c The Configuration representing the current state of agents.
     * @param adj_list The adjacency list representing the communication or connection
     *                 relationships between agents.
     * @return Returns a vector of pairs of connected agents, where each pair consists
     *         of two agents that are connected according to the adjacency list.
     */
    std::vector<std::pair<Agent, Agent>> get_connected_agents(
      const Configuration &c,
      const Communications &adj_list 
    ) {
      assert(c.size() > 0);
      
      std::vector<std::pair<Agent,Agent>> result;

      const int nb_agents = c.size();
      
      AgentPosition current_pos;
      std::pair<Agent, Agent> current_pair;
      std::pair<Agent, Agent> verify;
      Agent a_neighbor;

      for (Agent a = 0; a < nb_agents; a++) {
        current_pos = c.at(a);

        auto neighbors = boost::adjacent_vertices(current_pos, adj_list);
        for (AgentPosition neighbor_pos: make_iterator_range(neighbors)) {

          auto a_neighbor_it = find(c.begin(), c.end(), neighbor_pos);
          if (a_neighbor_it != c.end()) {
            a_neighbor = std::distance(c.begin(), a_neighbor_it);
            current_pair = std::make_pair<Agent, Agent>(std::move(a), std::move(a_neighbor));
            verify = std::make_pair<Agent, Agent>(std::move(a_neighbor), std::move(a));

            if (std::find(result.begin(), result.end(), verify) == result.end()) {
              result.push_back(current_pair);
            }
          }
        }
      }

      return result;
    }

  };

}