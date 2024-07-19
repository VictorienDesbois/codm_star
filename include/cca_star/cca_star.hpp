#pragma once

#include <list>
#include <cmath>
#include <stack>
#include <vector>
#include <random>
#include <iostream>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/reverse_graph.hpp>

#include "codm_types.hpp"


namespace cca_star {

  struct CAnode {
    public:
      AgentPosition node;
      AgentPosition successor;
      bool has_path;
      size_t suffix_length;
      size_t h;
      bool dead_end;

      CAnode() {
        node = std::numeric_limits<AgentPosition>::max();
      }

      CAnode(
        AgentPosition node_, 
        AgentPosition successor_, 
        bool has_path_, 
        size_t suffix_length_, 
        size_t heuristic_, 
        bool dead_end_ = false
      ) {
        node = node_;
        successor = successor_;
        has_path = has_path_;
        suffix_length = suffix_length_;
        h = heuristic_;
        dead_end = dead_end_;
      }

      // Compare the cost of two nodes depending of their respective heuristic and distance covered.
      bool operator < (const CAnode &b) const {
        return (this->has_path > b.has_path) ||
               (this->has_path && b.has_path && (this->suffix_length < b.suffix_length)) ||
               (!this->has_path && !b.has_path && (b.suffix_length < this->suffix_length)) ||
               (this->has_path == b.has_path && (this->suffix_length == b.suffix_length) && this->h <= b.h);
      }

      inline bool defined() const {
        return node < std::numeric_limits<AgentPosition>::max();
      }
  };


  class CCAstar {
    public:
      // constructors
      CCAstar();

      CCAstar(
        std::shared_ptr<MovesGraph> movement_graph,
        std::shared_ptr<CommunicationsGraph> comm_graph,
        std::vector<std::shared_ptr<ShortestPaths>> policies,
        bool check_swapping_conflicts = false
      );

      ~CCAstar();
        
      /**
       * @brief Compute an execution path to reach the goal.
       * 
       * Uses the CCA* algorithm to compute an execution path from the source configuration `s`
       * to the target configuration `t` for the given set of agents. CCA* is based on CA*, an
       * algorithm that tries to compute a path for a given agent, while taking in account
       * collisions (and in our case connectivity) constraints. The priority order in which
       * agents are treated is selected randomly. The algorithm is incomplete because each agent
       * will try to reach their optimal path withtout taking in account the complete configuration.
       * 
       * @param s The source Configuration.
       * @param t The target Configuration.
       * @param ma The MetaAgent representing the set of agents for which the path will be computed.
       * @return The computed Execution.
       */
      Execution search(Configuration s, Configuration t, MetaAgent ma);


      /**
       * @brief Set the heuristic type for the CCA* algorithm.
       * 
       * @param h_type An integer representing the heuristic type.
       */
      void set_heuristic_type(int h_type);


      /**
       * @brief Set the shuffle type for the CCA* algorithm. The order
       * of the agents is determined by the shuffle type.
       * 
       * @param shuffle_type An integer representing the shuffle type.
       */
      void set_shuffle_type(int shuffle_type);

    private:
      std::shared_ptr<MovesGraph> movement_graph_;
      std::shared_ptr<CommunicationsGraph> comm_graph_;
      std::vector<std::shared_ptr<ShortestPaths>> all_policies_;
      std::vector<std::shared_ptr<ShortestPaths>> policies_;
      size_t max_path_size_;
      bool check_swapping_conflicts_;

      enum { SHORTESTPATH, BIRDEYE };
      int heuristic_type_;

      enum { SEPARATE, MERGED };
      int shuffle_type_;


      /**
       * @brief Deterministically shuffle agents.
       * 
       * Shuffles the agent vector in a deterministic manner.
       * 
       * @param agent_vector A reference to the vector of agents to shuffle.
       */
      void deterministic_random_shuffle(std::vector<Agent> &agent_vector);


      /**
       * @brief Shuffle agents randomly.
       * 
       * Shuffles the agent vector in a random manner.
       * 
       * @param agent_vector A reference to the vector of agents to shuffle.
       */
      void standard_random_shuffle(std::vector<Agent> &agent_vector);


      /**
       * @brief Shuffle agents and return the randomized vector.
       * 
       * Shuffles the agents and returns the shuffled vector.
       * 
       * @param nb_agents The number of agents to shuffle.
       * @param s The source Configuration.
       * @param t The target Configuration.
       * @return A vector of shuffled agents.
       */
      std::vector<Agent> shuffle_agents(uint nb_agents, Configuration s, Configuration t);


      /**
       * @brief Get the shortest path between two positions using Dijkstra's algorithm.
       * 
       * Computes the shortest path between two positions for a given agent.
       * 
       * @param a The agent for which to compute the path.
       * @param s The start position.
       * @param t The target position.
       * @return The computed shortest Path.
       */
      Path get_shortest_path(Agent a, AgentPosition s, AgentPosition t);


      /**
       * @brief Get the heuristic value.
       * 
       * Computes the heuristic value for the given agent and positions as proposed by the policy.
       * 
       * @param a The agent for which to compute the heuristic.
       * @param s The start position.
       * @param t The target position.
       * @return The computed heuristic value.
       */
      size_t get_heuristic(Agent a, AgentPosition s, AgentPosition t);
   

      /**
       * @brief Initialize the cluster.
       * 
       * Each cluster is also called a frame. We compute the set of nodes in the last frame which has a path
       * to goal which stays in this frame (the frame represent the valid path of all agents inside the frame).
       * Then we compute the best path from each node of each frame backwards. At each frame t, and node n, we assign to n the successor 
       * from frame t+1 which has, if possible, an actual path to goal and in this case the one admitting the shortest one, 
       * and otherwise a successor node with the longest path with the least h value. Some nodes of frame t might have no successors in
       * frame t+1. These are marked as deadends, and assigned a heuristics h value. 
       * At the end of the computation, the start node at frame 0 has been assigned the best path. We follow this path, and possibly 
       * complete it with a suffix path within the last frame.
       * 
       * @param start The start Configuration.
       * @param shuffled A reference to the vector of shuffled agents.
       * @param nb_agents The number of agents.
       * @return An unordered set of agent positions representing the initialized cluster.
       */
      std::unordered_set<AgentPosition> init_cluster(const Configuration &start, std::vector<Agent> &shuffled, uint nb_agents);

     
      /**
       * @brief Check and clean up the execution.
       * 
       * Cleans up the execution paths and removes any invalid paths based on the path size and number of agents.
       * 
       * @param path_size The maximum path size.
       * @param nb_agents The number of agents.
       * @param paths A reference to the vector of paths.
       * @param shuffled A reference to the vector of shuffled agents.
       * @return The cleaned execution.
       */
      Execution execution_cleaner(int path_size, int nb_agents, std::vector<Path> &paths, std::vector<Agent> &shuffled);


      /**
       * @brief Extend neighborhoods with new agents paths.
       * 
       * @param neighborhoods A reference to the vector of neighborhoods.
       * @param paths A reference to the vector of paths.
       * @param path_size A reference to the path size.
       */
      void extend_neighborhoods(
        std::vector<std::unordered_set<AgentPosition>> &neighborhoods,
        std::vector<Path> &paths,
        int &path_size
      );


      /**
       * @brief Shorten neighborhoods. Remove some part of paths.
       * 
       * @param neighborhoods A reference to the vector of neighborhoods.
       * @param paths A reference to the vector of paths.
       * @param path_size A reference to the path size.
       */
      void shorten_neighborhoods(
        std::vector<std::unordered_set<AgentPosition>> &neighborhoods,
        std::vector<Path> &paths,
        int &path_size
      );


      /**
       * @brief Constrain neighborhoods.
       * 
       * @param neighborhoods A reference to the vector of neighborhoods.
       * @param paths A reference to the vector of paths.
       * @param current_agent The current agent being considered.
       */
      void constrain_neighborhoods(
        std::vector<std::unordered_set<AgentPosition>> &neighborhoods,
        std::vector<Path> &paths,
        Agent current_agent
      );


      /**
       * @brief Compute an execution path to reach the goal.
       * 
       * @param start The start Configuration.
       * @param goal The goal Configuration.
       * @param max_path_size The maximum path size (default is -1, meaning no limit).
       * @return The computed Execution.
       */
      Execution compute_execution(
        const Configuration& start, 
        const Configuration& goal, 
        size_t max_path_size = -1
      );


      /**
       * @brief Check if a position is a swapping conflict.
       * 
       * Determines whether the given position is a swapping conflict at a specific time step.
       * 
       * @param previous_p The previous agent position.
       * @param current_p The current agent position.
       * @param paths A reference to the vector of paths.
       * @param time_step The specific time step to check.
       * @return True if there is a swapping conflict, false otherwise.
       */
      bool is_position_a_swapping_conflict(
        AgentPosition previous_p,
        AgentPosition current_p,
        std::vector<Path> &paths,
        size_t time_step
      );


      /**
       * @brief Generate a potential path for an agent.
       * 
       * Generates a potential path for the given agent towards the goal, updating the prefix and neighborhoods.
       * 
       * @param agent The agent for which to generate the path.
       * @param prefix_h A reference to the prefix vector.
       * @param neighborhoods A reference to the vector of neighborhoods.
       * @param paths A reference to the vector of paths.
       * @param goal The goal position for the agent.
       */
      void generate_potential_path(
        Agent agent,
        std::vector<std::vector<CAnode>> &prefix_h,
        std::vector<std::unordered_set<AgentPosition>> &neighborhoods,
        std::vector<Path> &paths,
        AgentPosition goal
      );


      /**
       * @brief Computes a path for a single agent from the start position to the goal, 
       * considering neighborhoods and existing paths constraints.
       * 
       * @param agent The agent for which to compute the path.
       * @param start The start position.
       * @param goal The goal position.
       * @param neighborhoods A reference to the vector of neighborhoods.
       * @param paths A reference to the vector of paths.
       * @return The computed Path.
       */
      Path compute_path(
        Agent agent,
        AgentPosition start,
        AgentPosition goal,
        std::vector<std::unordered_set<AgentPosition>> &neighborhoods,
        std::vector<Path> &paths
      );
  };

}
