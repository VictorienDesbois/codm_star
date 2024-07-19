#pragma once

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/reverse_graph.hpp>

#include "graph.hpp"


namespace cmapf{

  typedef boost::property_map<Moves, boost::edge_weight_t>::type EdgeWeight;

  class ShortestPaths {
    public:
      /**
       * Constructors
       */
      ShortestPaths(const Moves &adj_list, const Agent agent, const AgentPosition target) {
        movement_adj_list_ = adj_list;
        agent_ = agent;
        t_ = target;

        init();
      }

      ShortestPaths(ShortestPaths parent, const AgentPosition target) {
        movement_adj_list_ = parent.movement_adj_list_;
        agent_ = parent.agent_;
        t_ = target;

        init();
      }

      void init() {
        costs_ = std::vector<double>(boost::num_vertices(movement_adj_list_));
        predecessors_ = std::vector<int>(boost::num_vertices(movement_adj_list_));
        
        boost::dijkstra_shortest_paths(
          boost::make_reverse_graph(movement_adj_list_), t_,
          boost::predecessor_map(&predecessors_[0]).distance_map(&costs_[0]) 
        );

        edge_weight_map_ = boost::get(boost::edge_weight_t(), movement_adj_list_);
      }


      /**
       * getters
       */
      Agent get_agent() { 
        return agent_; 
      }
      
      AgentPosition get_goal() { 
        return t_; 
      }

      Moves get_adj_list() {
        return movement_adj_list_;
      }
      
      double get_cost(AgentPosition p) { 
        return costs_[p]; 
      }
      
      double get_edge_cost(AgentPosition u, AgentPosition v) {
        return boost::get(edge_weight_map_, boost::edge(u,v,movement_adj_list_).first);
      }

      AgentPosition get_step(AgentPosition p) {
        return predecessors_[p];
      }


      /**
       * setters
       */
      void update_cost(AgentPosition p, double new_cost) {
        costs_[p] = new_cost;
      }


      /**
       * @brief Return the shortest path from a source position to the target
       *        using Dijkstra's Algorithm.
       * 
       * @param s The source position of the agent.
       * @return A Path object representing the shortest path from the source position to the target.
       */
      Path get_shortest_path(AgentPosition s) {
        Path path;
        AgentPosition p = s;
         
        while (p != predecessors_[p]) {
          path.push_back(p);
          p = predecessors_[p];
        }
        path.push_back(p);

        return path;
      }

    private:
      // id of the agent
      Agent agent_;
      // goal position of the agent
      AgentPosition t_;

      Moves movement_adj_list_;
      std::vector<double> costs_;
      std::vector<int> predecessors_;
      EdgeWeight edge_weight_map_;
      Path shortest_path_;
  };

}