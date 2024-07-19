#pragma once

#include <queue>

#include "shortest_paths.hpp"
#include "meta_agents_manager.hpp"
#include "collision_conflicts_manager.hpp"
#include "connectivity_conflicts_manager.hpp"


namespace codm_star {


  /**
   * @struct ODconfig
   * @brief Configuration for Operator Decomposition search. Contains
   * data such as the current meta-agents or the move assigned during
   * the OD search.
   */
  struct ODconfig {
    Configuration config;

    // manage OD computation
    
    // all the agents that must be treated by OD
    MetaAgent OD_meta_agent;

    // meta-agent has been modified
    bool OD_ma_have_increased;

    // meta-agent has been modified
    bool generated_by_subsolver;

    // all the move assigned during OD
    std::unordered_map<Agent, AgentPosition> assigned_moves;
    
    // the last agent assigned
    Agent last_agent_assigned;

    // subset of OD_meta_agent that have an higher priority than the others 
    MetaAgent priority_OD_agents;

    // connected group
    MetaAgent connected_ma;

    ODconfig() {}
    ODconfig(Configuration c) { 
      config = c; 
      OD_ma_have_increased = false;
      generated_by_subsolver = false;
    }


    // required for the map
    bool operator == (const ODconfig &other) const{
      return (config == other.config) && (assigned_moves == other.assigned_moves);
    }


    // verify if the configuration is not partial
    bool is_standard() {
      return assigned_moves.size() == 0;
    }


    bool is_OD_started() {
      return is_OD_in_process() && generated_by_subsolver;
    }


    // verify if the OD computation
    bool is_OD_in_process() {
      return OD_meta_agent.size() > 0;
    }


    // verify if the OD computation is the last one
    bool is_OD_finished() {
      return OD_meta_agent.size() == 0;
    }


    // get the next agent to treat for OD
    Agent next_agent_to_compute() {
      Agent next_agent = -1;

      for (Agent a: OD_meta_agent) {
        if (assigned_moves.find(a) == assigned_moves.end()) {
          next_agent = a;
          break;
        }
      }

      assert(next_agent != -1);

      return next_agent;
    }

    
    // get the next agent to treat for connected OD
    Agent connected_next_agent_to_compute(
      MovesGraph &topological_graph
    ) {
      if (in_connectivity_crisis()) {
        Agent next_agent = *priority_OD_agents.begin();
        priority_OD_agents.erase(next_agent);
        return next_agent;
      }

      return get_agent_minimum_neighbors(topological_graph); 
    }


    // erase the priority agents from OD meta agent to put it inside the priority meta agent
    void insert_priority_agents(MetaAgent ma) {
      priority_OD_agents.clear();

      for (Agent a: ma) {
        assert(OD_meta_agent.find(a) != OD_meta_agent.end());
        priority_OD_agents.insert(a);
      }
    }


    // if OD manage a problem with connectivity
    bool in_connectivity_crisis() {
      return priority_OD_agents.size() > 0;
    }


    // agent with the minimum neighbors in order to prune unconnactable OD config
    Agent get_agent_minimum_neighbors(
      MovesGraph &topological_graph
    ) {
      Agent next_agent = -1;
      uint64_t min_nb_neighbors = std::numeric_limits<uint64_t>::max();
      uint64_t nb_neighbors = 0;
      Configuration c = config;

      // assigne the moves
      for (const auto& [key, value]: assigned_moves) {
        c.at(key) = value;
      }

      for (Agent a: OD_meta_agent) {
        if (assigned_moves.find(a) == assigned_moves.end()) {
          nb_neighbors = 0;
          AgentPosition p = c.at(a);
          std::unordered_set<AgentPosition> neighbors = topological_graph.get_neighbors_set(p);
          
          for (AgentPosition neighbor_pos: neighbors) {
            auto neighbor_pos_it = std::find(c.begin(), c.end(), neighbor_pos);
            if (neighbor_pos != p && neighbor_pos_it != c.end()) {
              nb_neighbors++;
            }
          }

          // get the agent with the minimum amount of neighbors
          if (nb_neighbors < min_nb_neighbors) {
            min_nb_neighbors = nb_neighbors;
            next_agent = a;
          }
        }
      }

      assert(next_agent != -1);

      return next_agent;
    }


    // check if all potential moves are assigned
    bool is_all_moves_assigned() {
      return assigned_moves.size() == config.size();
    }


    // if all the potential moves are assigned, 
    // replace the current configuration by the moves assigned
    bool assign_moves() {
      if (is_all_moves_assigned()) {
        for (auto it = assigned_moves.begin(); it != assigned_moves.end(); it++) {
          config.at(it->first) = it->second;
        }

        // clear the data related to OD
        assigned_moves.clear();
        OD_meta_agent.clear();

        return true;
      }

      return false;
    }

  };


  /**
   * @struct Node
   * @brief The node that CODM* will explore during the search.
   * Form a graph of the possible configuration explored by CODM* from the source
   * configuration.
   */
  struct Node {
    ODconfig od_config;

    // agents in conflict in nodes that can be 
    // reached from this state
    MetaAgents meta_agents;

    bool visited, side_successor;
    double cost, heuristic;

    // pointer to the optimal node from
    // which we ca reach this node
    std::shared_ptr<Node> best_predecessor;
    // all node that allow to reach this node
    std::set<std::shared_ptr<Node>> predecessors;
    // in case we use recursive calls
    std::shared_ptr<Node> forwards_ptr;
    // planning iteration
    int node_planning_iter;

    Node(ODconfig c) {
      od_config = c;
      visited = false;
      side_successor = false;
      // initial cost is infinity
      cost = std::numeric_limits<double>::max();
      heuristic = 0;
      // init the meta_agents set
      best_predecessor = nullptr;
      // in case we use recursive calls
      forwards_ptr = nullptr;

      node_planning_iter = 0;
    }

    void reset(int current_planning_iter) {
      if (current_planning_iter > node_planning_iter) {
        visited = false;
        side_successor = false;
        // initial cost is infinity
        cost = std::numeric_limits<double>::max();
        heuristic = 0;
        
        best_predecessor = nullptr;
        // update planning iteration
        node_planning_iter = current_planning_iter;

        predecessors.clear();
      }
    }
  };


  /**
   * @struct CompareNode
   * @brief Provides a means to compare the cost of two nodes.
   */
  struct CompareNode {
    bool operator()(const std::shared_ptr<Node> u, const std::shared_ptr<Node> v) const {
      if (u == nullptr || v == nullptr) { return true; }
      return (v->cost + v->heuristic) < (u->cost + u->heuristic);
    }
  };

  typedef std::priority_queue<
    std::shared_ptr<Node>, 
    std::vector<std::shared_ptr<Node>>,
    CompareNode
  > OpenList;


  /**
   * @struct CodmLogs
   * @brief Provides function to print configurations, executions and meta-agents.
   */
  struct CodmLogs {
    CodmLogs() {}

    void log_debug_od_config(ODconfig c) {
      std::string main_configuration = "\n{";
      for (auto p: c.config) {
        main_configuration += "[" + std::to_string(p) + "]" + ",";
      }
      main_configuration += "}\n";

      std::string assigned_moves = "{";
      for (auto it = c.assigned_moves.begin(); it != c.assigned_moves.end(); it++) {
        assigned_moves += "{ " + std::to_string(it->first) + ", " + std::to_string(it->second) + " }";
      }
      assigned_moves += "}";

      std::string od_agents = "\n[";
      for (Agent a: c.OD_meta_agent) {
        od_agents += std::to_string(a) + ", ";
      }
      od_agents += "]"; 

      LOG_DEBUG(main_configuration + assigned_moves + od_agents);
    }


    void log_execution(Execution exec) {
      assert(exec.size() > 0);

      std::vector<std::string> agent_paths;
      int num_agents = exec.at(0).size();

      for (int i = 0; i < num_agents; i++) {
        agent_paths.push_back("[");
      }

      for (size_t j = 0; j < exec.size(); j++) {
        auto config = exec[j];
        for (int i = 0; i < num_agents; i++) {
          agent_paths[i] = agent_paths[i] + std::to_string(config[i]) + ",";
        }
      }

      std::string result = "{";
      for (auto current_path: agent_paths) {
        result = result + current_path + "],";
      }
      result = result + "}";

      LOG_DEBUG(result);
    }


    void log_debug_meta_agents(std::shared_ptr<Node> n) {
      std::string meta_agents = "";
      for (MetaAgent ma: n->meta_agents) {
        meta_agents += "[";
        for (Agent a: ma) {
          meta_agents += std::to_string(a) + ",";
        }
        meta_agents += "],";
      }

      meta_agents += " | ";

      meta_agents += "[";
      for (Agent a: n->od_config.OD_meta_agent) {
        meta_agents += std::to_string(a) + ",";
      }
      meta_agents += "]";

      LOG_DEBUG(meta_agents);
    }
  };

}
