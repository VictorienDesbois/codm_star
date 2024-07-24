#pragma once

#include <unordered_map>
#include <functional>
#include <memory>
#include <exception>
#include <stack>
#include <chrono>
#include <ctime>
#include <optional>

#include "codm_types.hpp"
#include "subplanners_manager.hpp"


namespace std{
  template <> 
  struct hash<codm_star::ODconfig>{
    size_t operator()(const codm_star::ODconfig &val) const{
      size_t hash = boost::hash_range(val.config.cbegin(), val.config.cend());
      size_t hash_assign = boost::hash_range(val.assigned_moves.cbegin(), val.assigned_moves.cend());
      
      // merge the hash
      boost::hash_combine<size_t>(hash, hash_assign);
      return hash;
    }
  };

  template <> 
  struct hash<std::vector<cmapf::Agent>>{
    size_t operator()(const std::vector<cmapf::Agent> &val) const{
      size_t hash = boost::hash_range(val.cbegin(), val.cend());      
      return hash;
    }
  };
}

namespace codm_star {

  class ConnectedODMStar {
    public:

      /**
       * @brief Constructor for the ConnectedODMStar class.
       * 
       * @param movement_graph A shared pointer to the MovesGraph object, representing the movement graph.
       * @param comm_graph A shared pointer to the CommunicationsGraph object, representing the communications graph.
       * @param subsolver_type An integer specifying the type of subsolver to use.
       * @param activate_logs A boolean indicating whether to activate logging.
       * @param check_swapping_conflicts A boolean indicating whether to check for swapping conflicts.
       * @param inflation_factor A double representing the inflation factor for path planning.
       */
      ConnectedODMStar(
        std::shared_ptr<MovesGraph> movement_graph,
        std::shared_ptr<CommunicationsGraph> comm_graph,
        int subsolver_type = CCA_STAR,
        int optim_type = NO_OPTIM,
        bool activate_logs = false,
        bool check_swapping_conflicts = false,
        double inflation_factor = 10.0);


      /**
       * @brief Constructor for the ConnectedODMStar class. Create a recursive planner from
       * a parent ConnectedODMStar object.
       * 
       * @param ma A meta-agent.
       * @param parent A reference to a parent ConnectedODMStar object.
       */
      ConnectedODMStar(
        std::vector<Agent> ma,
        ConnectedODMStar &parent);


      /**
       * @brief Perform a search from the starting configuration to the target configuration.
       * 
       * @param s The starting Configuration object.
       * @param t The target Configuration object.
       * @param iterations_limit The maximum number of iterations allowed.
       * @param time_limit The maximum time allowed in seconds.
       * @return An Execution object containing the result of the search.
       */
      Execution search(
        Configuration s, 
        Configuration t,
        uint64_t iterations_limit = std::numeric_limits<uint64_t>::max(),
        uint32_t time_limit = std::numeric_limits<uint32_t>::max());


      /**
       * 
       */
      Execution bidirectional_search(
        Configuration s, 
        Configuration t,
        std::optional<std::pair<uint64_t, double>> iterations_parameters,
        std::optional<std::pair<uint32_t, double>> time_parameters,
        bool activate_score = true);


      /**
       * Compute the MetaAgents generate by two OD configurations
       */
      MetaAgents od_collisions(ODconfig &s, ODconfig &t);

    private:

      ////////////////
      // ATTRIBUTES //
      ////////////////

      // parameter(s)
      double inflation_factor_;
      bool activate_logs_;
      bool check_swapping_conflicts_;
      int subsolver_type_;
      int optim_type_;

      // graph
      std::shared_ptr<MovesGraph> movement_graph_;
      std::shared_ptr<CommunicationsGraph> comm_graph_;

      // agents
      uint64_t nb_agents_;
      std::vector<Agent> agents_;
      MetaAgents all_agents_;  

      // policies for each agents
      std::vector<std::shared_ptr<ShortestPaths>> policies_;
      std::vector<uint64_t> positions_score_;

      // map of all node already explored
      std::unordered_map<ODconfig, std::shared_ptr<Node>> explored_;

      // manage the sub computation of the main planner
      SubplannersManager subplanners_;
      
      // the type of the current conflict
      int current_conflict_;

      // measures
      std::map<Configuration, uint64_t> config_count_;
      uint64_t nb_iterations;

      // recursive calls management
      std::shared_ptr<std::unordered_map<std::vector<Agent>, 
        std::shared_ptr<ConnectedODMStar>>> recursive_subplanners_;
      int planning_iter_;
      bool recursive_call_;

      // enums
      enum { // type of subsolver
        CCA_STAR, 
        RECURSION, 
        NAIVE 
      };

      enum { // type of optimization
        NO_OPTIM,
        BIDIR,
        BIDIR_SCORE
      };

      enum { // type of conflict
        NO_CONFLICT, 
        COLLISION, 
        CONNECTIVITY 
      };

      /////////////
      // METHODS //
      /////////////

      /**
       * @brief Create a node and insert it in the "explored" map if the configuration
       * associated to the node does not exist. 
       * (for more information check the hash function at the start of this file)
       * Return the existing node if the configuration is inside the "explored" map
       *
       * @param c A configuration.
       * @return A shared pointer to a Node object.
       */
      std::shared_ptr<Node> get_node(ODconfig c);

      
      /**
       * @brief Compute the individual path of each agent.
       * 
       * @param t The target configuration.
       */
      void preprocess_agents_individual_paths(ODconfig t);


      /**
       * @brief Calculate the heuristic value for a given configuration.
       * 
       * @param c A configuration.
       * @return The heuristic value as a double.
       */
      double get_heuristic(ODconfig c);


      /**
       * @brief Calculate the cost of the edge between two configurations.
       * 
       * @param s The starting ODconfig object.
       * @param t The target ODconfig object.
       * @return The edge cost as a double.
       */
      double get_edge_cost(ODconfig s, ODconfig t);


      /**
       * @brief Visit of the get_successors collection.
       * If this ODconfig is interesting (good heuristic, not visited, etc.), 
       * create a node and add it to the open list.
       * 
       * @param s The current successor to visit.
       * @param n A shared pointer to the current Node object.
       * @param new_meta_agents The MetaAgents object to be updated.
       * @param open_list The OpenList object to be updated.
       * @param ma_manager The MetaAgentManager object needed to merge meta-agents.
       * @param co_manager The ConnectivityManager object needed to check connectivity properties about the node.
       */
      void visit_successor(
        ODconfig s,
        std::shared_ptr<Node> n,
        MetaAgents &new_meta_agents, 
        OpenList &open_list,
        MetaAgentManager &ma_manager,
        ConnectivityManager &co_manager);


      /**
       * @brief Perform backpropagation from a given node and update the meta-agents of the
       * predecessors of this node.
       * 
       * @param n A shared pointer to the current Node object.
       * @param new_meta_agents The MetaAgents object to be used during backpropagation.
       * @param open The OpenList object to be updated.
       */
      void back_propagation(
        std::shared_ptr<Node> n, 
        MetaAgents new_meta_agents, 
        OpenList &open);


      /**
       * @brief Handle the case when no successor is found for a node: re-open
       * some nodes.
       * 
       * @param open_size_before_it The size of the OpenList before this operation.
       * @param n A shared pointer to the current Node object.
       * @param open The OpenList object to be updated.
       */
      void no_successor(
        size_t open_size_before_it,
        std::shared_ptr<Node> n,
        OpenList &open);


      /**
       * @brief Get the standard ancestor of a node.
       * 
       * @param an od configuration.
       * @return the standard ancestor of a node.
      */
      std::optional<std::shared_ptr<Node>> get_standard_node(
        ODconfig c);


      /**
       * @brief Perform backpropagation to re-open the re-openable nodes
       * (i.e. nodes with potential successors to be added to open)
       * from an initial node n.
       * 
       * @param n A shared pointer to the current Node object.
       */
      void back_propagation_ma_od(std::shared_ptr<Node> n);


      /**
       * @brief Generate the successors configurations of a given node.
       * 
       * @param n A shared pointer to the current Node object.
       * @return A vector of successor ODconfig objects.
       */
      std::vector<ODconfig> get_successors(std::shared_ptr<Node> n);


      /**
       * @brief Compute the successors of the node n using Operator Decomposition (OD).
       * Select the smallest agent a (given by "next_agent_to_compute") and return all the
       * possible futur positions for this agent.
       * 
       * @param n The current node.
       * @return The successors of n considering the agent a.
       */
      std::vector<ODconfig> od_successors(ODconfig c);


      /**
       * @brief Determine the successor configuration using subsolvers.
       * 
       * @param n A shared pointer to the current Node object.
       * @return The successor ODconfig object.
       */
      ODconfig subsolver_successor(std::shared_ptr<Node> n);


      /**
       * @brief Compute sub-solvers for a given node.
       * 
       * @param n A shared pointer to the current Node object.
       * @return A vector of ODconfig objects representing the results of the sub-solvers.
       */
      std::vector<ODconfig> compute_sub_solvers(std::shared_ptr<Node> n); 


      /**
       * @brief Get the next configuration from the subsolver for a given meta-agent.
       * 
       * @param ma The MetaAgent object.
       * @param c The current Configuration object.
       * @return An optional Configuration object representing the next configuration.
       */
      std::optional<Configuration> get_next_config_from_subsolver(
        MetaAgent ma, 
        Configuration c);


      /**
       * @brief Run Operator Decompostion for a given configuration c.
       * Manage the optimization depending of the type of CODM*.
       * 
       * @param c The current ODconfig object.
       * @return A vector of ODconfig objects representing the computed OD configurations.
       */
      std::vector<ODconfig> compute_od(std::shared_ptr<Node> n);


      /**
       * @brief Compute subsolvers for a given node.
       * Manage the optimization depending of the type of CODM*.
       * 
       * @param n A shared pointer to the current Node object.
       * @return A vector of ODconfig objects representing the results of the subsolvers.
       */
      std::vector<ODconfig> compute_subsolvers(std::shared_ptr<Node> n);


      /**
       * @brief Compute a random successor configuration for a given node.
       * 
       * @param n A shared pointer to the current Node object.
       * @return An ODconfig object representing a random successor.
       */
      ODconfig get_random_successor(std::shared_ptr<Node> n);


      /**
       * @brief Check if two successive configurations are valid, i.e., do not contain conflicts.
       * 
       * @param pred_c The predecessor Configuration.
       * @param c The current Configuration to be checked.
       * @return true if the configurations are valid (no conflicts), false otherwise.
       */
       bool check_config_validity(Configuration pred_c, Configuration c);


      /**
       * @brief Return the execution with n as the target node.
       * 
       * @param n A shared pointer to the Node object.
       * @return An Execution object representing the execution starting from the node's configuration.
       */
      Execution get_execution(std::shared_ptr<Node> n);


      // CODM* optimizations //
      /////////////////////////

      /**
       * @brief Assign scores to each cell based on its neighborhood.
       * 
       * This function evaluates each cell's score depending on the ease of movement for an agent
       * at that cell's position. Cells that are more accessible or advantageous for movement
       * will receive higher scores.
       */
      void preprocess_topological_graph();


      /**
       * @brief Give the score computed by the pre-processing step of the topological
       * graph.
       * 
       * @param c A configuration.
       * @return the score of the configuration.
       */
      uint64_t get_configuration_score(
        Configuration c
      );

      // ODrM* optimizations //
      /////////////////////////

      /**
       * @brief Get the next optimal step after the given node.
       * 
       * @param n A shared pointer to the Node object.
       * @return The optimal Configuration associated with the given node.
       */
      Configuration get_node_step(std::shared_ptr<Node> n);
         

      /**
       * @brief Get the next optimal step after `s` in the path from `s` to `t`.
       *
       * @param s The starting ODconfig object.
       * @param t The target ODconfig object.
       * @return The Configuration representing the step `s+1` in the path optimal from `s` to `t`.
       */
      Configuration get_step(ODconfig s, ODconfig t);


      /**
       * @brief Initialize the backtrace path from a node.
       * 
       * @param n A shared pointer to the Node object from which to start the backtrace.
       * @return An Execution object representing the initialized backtrace path.
       */
      Execution back_trace_path_init(std::shared_ptr<Node> n);


      /**
       * @brief Perform backtrace along a path from a node. For each best predecessor of the node `n` assign `n` as the forwards_ptr of `n->best_predecessor` (allow a computation shortcut for ODrM\*: if a node contains a non null forward_ptr, then we can reach the target node `t` from this node and the search can be stopped).
       * 
       * @param n A shared pointer to the current Node object.
       * @param s A shared pointer to the starting Node object for backtrace.
       */
      void back_trace_path(std::shared_ptr<Node> n, std::shared_ptr<Node> s);
  };

}